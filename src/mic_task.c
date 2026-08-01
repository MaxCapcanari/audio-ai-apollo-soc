//*****************************************************************************
//
//! @file mic_task.c
//!
//! @brief Mic record/playback: AUDADC analog mic input, I2S0 TX speaker output.
//!
//! Press BUTTON1 (SW2)   -> record REC_SECONDS of audio from CMM-2718AB MEMS mic.
//! Press again           -> play back through MAX98357A speaker via I2S0 TX.
//! Long press (2 s)      -> play 1 kHz test tone.
//!
//! After recording, audio is resampled 23438 Hz -> 16 kHz mono int16 in
//! g_pcmBuf and encoded to Opus 32 kbit/s CBR in g_opusBuf for the BLE
//! stream service.
//!
//! Wiring (Apollo4p Blue KXR EVB):
//!   CMM-2718AB OUT -> 100 nF series cap -> AUDADC SE0 (PGA channel A)
//!   CMM-2718AB VDD <- VDDAUDA / AUDA (~1.7 V), 100 nF to GND
//!   MAX98357 BCLK  -> GPIO 11 (I2S0_CLK)
//!   MAX98357 LRC   -> GPIO 13 (I2S0_WS)
//!   MAX98357 DIN   -> GPIO 12 (I2S0_SDOUT)
//!   MAX98357 SD    -> 3.3 V (enable)
//!
//! Sample rates:
//!   AUDADC capture: HFRC 48 MHz / div32 / 64 = 23,437.5 Hz
//!   I2S TX:         HFRC 1.5 MHz BCLK / 64 bits per stereo frame = 23,437.5 Hz
//!   PCM storage:    16 kHz mono int16 (Opus encoder native rate)
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "FreeRTOS.h"
#include "task.h"

#include "mic_task.h"
#include "ae_api.h"         // Ambiq Opus encoder (audio_enc_init / audio_enc_encode_frame)
#ifndef KWS_DISABLE
#include "kws_inference.h"  // TFLite KWS wrapper
#include "ns_audio_mfcc.h"  // MFCC feature extraction
#endif

//*****************************************************************************
// Configuration
//*****************************************************************************

#define MIC_I2S_MODULE      0

// I2S0 TX GPIO pins (speaker only — no SDIN: capture is on AUDADC)
#define MIC_CLK_PIN         11
#define MIC_CLK_FUNC        AM_HAL_PIN_11_I2S0_CLK
#define MIC_WS_PIN          13
#define MIC_WS_FUNC         AM_HAL_PIN_13_I2S0_WS
#define MIC_SDOUT_PIN       12
#define MIC_SDOUT_FUNC      AM_HAL_PIN_12_I2S0_SDOUT
#define AMP_SD_PIN          8      // MAX98357A SD (shutdown) — low=off, high=on

// I2S TX DMA: 256 words = 128 stereo frames per ISR completion (~5.46 ms).
#define MIC_DMA_SAMPLES     256

// AUDADC DMA words per completion. SAMPMODE_MED + 4 slots emits 2 DMA words
// per scan (A-pair + B-pair). 256 words = 128 scans = 128 channel-A samples
// per completion (~5.46 ms — one ISR call every ~5.46 ms, so ~183/sec).
#define AUDADC_DMA_WORDS    256
#define AUDADC_MONO_SAMPLES 128

// AUDADC slot bit-19 selects A-pair (0) or B-pair (1) within each DMA word.
#define MIC_SRC_A0          0
#define MIC_SRC_A1          1
#define MIC_SRC_B0          2
#define MIC_SRC_B1          3
#define MIC_RECORD_SOURCE   MIC_SRC_A0   // SE0 = analog mic

// AUDADC gain (dB). PREAMP_GAIN is per-channel preamp; CHANNEL_GAIN is the
// internal PGA stage. Tuned for CMM-2718AB at conversational distance.
#define PREAMP_GAIN_DB      18  //originally was 12.Changing to 18 hd no effect. Looks like it is getting clobbered. 
#define CHANNEL_GAIN_DB     24   // orignally was 6, 24 works well

// CMM-2718AB requires 1.6-3.6 V. Apollo MICBIAS trim values below bypass mode
// only reach ~0.9-1.5 V; trim 63 selects bypass mode (VDDAUDA, ~1.7 V).
#define MIC_BIAS_TRIM       63

// BUTTON1 (SW2) = GPIO 19. BUTTON0 (SW1, GPIO 17) is reserved for BLE messages.
#define BTN_PIN             19

// Playback: int16 PCM -> 24-bit I2S TX. Left-shift maps int16 full scale into
// the int24 range; soft limiter knees in well below clipping so loud peaks
// compress instead of hard-clipping the 24-bit packer.
#define PLAY_VOL_SHIFT      9                  // int16 << 9 -> 25-bit, soft-limited
#define PLAY_LIMIT_START    0x500000
#define PLAY_LIMIT_MAX      0x7FFFFF

// Native AUDADC / I2S sample rate
#define MIC_NATIVE_RATE     23438

// PCM storage rate — Opus encoder requires 16 kHz mono int16 input.
#define REC_PCM_RATE        16000
#define REC_SECONDS         10
#define REC_PCM_SAMPLES     (REC_PCM_RATE * REC_SECONDS)

// Opus encoder framing (fixed by SDK library: 20 ms @ 16 kHz, CBR 32 kbit/s)
#define OPUS_FRAME_SAMPLES  320
#define OPUS_FRAME_BYTES    80
#define OPUS_NUM_FRAMES     (REC_SECONDS * 50)
#define OPUS_BUF_BYTES      (OPUS_NUM_FRAMES * OPUS_FRAME_BYTES)

// Resampler steps in 16.16 Q-format.
//   Capture:  out=16000 from in=23438 -> step = 23438/16000 ≈ 1.4649
//   Playback: out=23438 from in=16000 -> step = 16000/23438 ≈ 0.6826
#define CAP_STEP_Q16  (uint32_t)(((uint64_t)MIC_NATIVE_RATE << 16) / REC_PCM_RATE)
#define PLAY_STEP_Q16 (uint32_t)(((uint64_t)REC_PCM_RATE   << 16) / MIC_NATIVE_RATE)

// Per-second stats: 23438 Hz / 128 mono samples per ISR ≈ 183 ISR calls/sec.
#define SEC_CHUNKS          183

#ifndef KWS_DISABLE
#define KWS_FRAME_SAMPLES   320     // 20 ms @ 16 kHz
#define KWS_MFCC_COEFFS     10
#define KWS_MFCC_FBANK_BINS 40
#define KWS_MFCC_FRAME_POW2 512

// neuralSPOT ns_mfcc_map_arena / ns_fbanks_map_arena multiply pointer-arithmetic
// offsets by sizeof(T) on a T* pointer, inflating the layout by 4×. The
// buffers don't overlap as long as the arena is large enough; 64 KB is plenty.
#define MFCC_ARENA_BYTES    (64 * 1024)
#endif

//*****************************************************************************
// DMA buffers — shared SRAM, 16-byte aligned
//*****************************************************************************

#ifndef KWS_DISABLE
// KWS ping-pong (ISR fills one while task reads the other)
AM_SHARED_RW static int16_t  g_kwsBuf[2][KWS_FRAME_SAMPLES];
static volatile uint32_t     g_kwsWriteBuf = 0;
static volatile uint32_t     g_kwsWritePos = 0;
static volatile uint32_t     g_kwsReadyBuf = 0xFF;

AM_SHARED_RW static uint8_t  g_mfccArena[MFCC_ARENA_BYTES];
static float                 g_mfccFeatures[KWS_NUM_FRAMES * KWS_NUM_COEFFS];
static uint32_t              g_mfccFrameCount = 0;
static ns_mfcc_cfg_t         g_mfccCfg;

// Software gain applied only on the KWS feed — brings the AUDADC PCM into the
// range Google Speech Commands was trained on without touching the recording
// path. 4x ≈ +12 dB; saturates to int16 full scale.
#define KWS_SW_GAIN          2

// Energy gate: peak |sample| over the 1-second KWS window must exceed this to
// invoke the model. Below it, force-classify as silence — eliminates the
// "go"/"yes" hallucinations the DS-CNN model emits on AUDADC residual noise.
#define KWS_PEAK_GATE        2000
static volatile int32_t      g_kwsWindowPeak = 0;
#endif

// AUDADC ping+pong contiguous buffer (HAL alternates halves automatically).
AM_SHARED_RW static uint32_t g_audadcBufRaw[2 * AUDADC_DMA_WORDS + 3];

// I2S TX ping/pong
AM_SHARED_RW static uint32_t g_txBufRawA[2 * MIC_DMA_SAMPLES + 3];
AM_SHARED_RW static uint32_t g_txBufRawB[2 * MIC_DMA_SAMPLES + 3];

//*****************************************************************************
// Recording buffers (in shared SRAM)
//   g_pcmBuf  — 16 kHz int16 mono PCM, used for on-device playback
//   g_opusBuf — Opus 32 kbit/s CBR, ready for BLE transmission
//*****************************************************************************

AM_SHARED_RW static int16_t g_pcmBuf[REC_PCM_SAMPLES];
AM_SHARED_RW static uint8_t g_opusBuf[OPUS_BUF_BYTES];


static volatile uint32_t g_opusLen = 0;

//*****************************************************************************
// State
//*****************************************************************************

static void *g_pI2SHandle    = NULL;
static void *g_pAUDADCHandle = NULL;

typedef enum { MODE_LISTENING, MODE_IDLE, MODE_RECORDING, MODE_PLAYING, MODE_TESTTONE } mic_mode_t;

#ifdef KWS_DISABLE
#define MIC_MODE_INITIAL  MODE_IDLE
#else
#define MIC_MODE_INITIAL  MODE_LISTENING
#endif

static volatile mic_mode_t g_mode     = MIC_MODE_INITIAL;
static volatile uint32_t   g_recPos   = 0;
static volatile uint32_t   g_recLen   = 0;

static volatile bool       g_recDone  = false;
static volatile bool       g_playDone = false;

// Test tone: 1 kHz square wave, 2 seconds at native rate.
#define TONE_FREQ           1000
#define TONE_HALF_PERIOD    (MIC_NATIVE_RATE / (TONE_FREQ * 2))
#define TONE_AMPLITUDE      0x200000
#define TONE_DURATION       (MIC_NATIVE_RATE * 2)
static volatile uint32_t   g_tonePos  = 0;
static volatile bool       g_toneDone = false;

// Capture-side resampler state (23438 Hz int16-scaled -> 16000 Hz int16)
static volatile int32_t   g_capLast       = 0;
static volatile uint32_t  g_capAccum      = 0;

// DC blocker (single-pole IIR HPF). The CMM-2718AB has a measurable DC bias
// at the AUDADC pin; leaving it in skews FFT bin 0 and any subsequent feature.
//   y[n] = x[n] - x[n-1] + R * y[n-1]
// R = 32440/32768 ≈ 0.9900 → corner ≈ 37 Hz at 23.4 kHz (well below speech).
#define DC_BLOCK_R_Q15  32440
static volatile int32_t   g_dcPrevIn      = 0;
static volatile int32_t   g_dcPrevOut     = 0;

// HYER INSERT HERE
// Anti-alias / de-hiss biquad LPF (Butterworth 2nd order, fc~4 kHz @ 23.438 kHz,
// Q=0.707). Content above 8 kHz folds back into the audio band as hiss through
// the 23.438->16 kHz decimator. 4 kHz cutoff preserves speech (0.3-3.4 kHz).
//   y[n] = b0(x[n] + x[n-2]) + b1*x[n-1] + a1_n*y[n-1] + a2_n*y[n-2]
#define LPF_B0     0.1615f
#define LPF_B1     0.3231f
#define LPF_A1_N   0.5871f
#define LPF_A2_N  -0.2333f
static volatile float     g_lpfX1         = 0.0f;
static volatile float     g_lpfX2         = 0.0f;
static volatile float     g_lpfY1         = 0.0f;
static volatile float     g_lpfY2         = 0.0f;

// Downward expander (soft noise gate). Below the threshold the gain is squared,
// so noise-floor samples get heavy attenuation while speech passes unchanged.
// Asymmetric envelope (fast attack, slow release) avoids pumping on transients.
//   gain(env) = 1                   if env >= GATE_THRESH
//   gain(env) = (env/GATE_THRESH)^2 if env <  GATE_THRESH  (clipped to GATE_FLOOR)
// Raise GATE_THRESH if hiss is still audible in pauses; lower it if soft word
// onsets get chopped.
#define GATE_THRESH        3500.0f  // 2000.0f worked OK
#define GATE_FLOOR         0.03f  // was 0.03f
#define GATE_ATTACK_A      0.005f     // ~9 ms attack  //was 0.20f
#define GATE_RELEASE_A     0.0004f    // ~55 ms release  // was 0.0015f
static volatile float     g_gateEnv       = 0.0f;

// Playback-side resampler state (16000 Hz int16 -> 23438 Hz int24)
static volatile uint32_t  g_playPhaseQ16  = 0;
static volatile uint32_t  g_playSrcIdx    = 0;

// One-shot playback capture for debugging the reconstruction path.
static volatile uint32_t g_dbgPlayCapture = 0;   // 1 = arm, ISR sets 2 when full
#define DBG_PLAY_N  48
AM_SHARED_RW static int32_t  g_dbgPlayOut[DBG_PLAY_N];   // interpolated output
AM_SHARED_RW static int16_t  g_dbgPlaySrc[DBG_PLAY_N];   // g_pcmBuf[g_playSrcIdx]
AM_SHARED_RW static uint16_t g_dbgPlayPhase[DBG_PLAY_N]; // phase at that step
static volatile uint32_t g_dbgPlayPos = 0;

// Playback-side reconstruction LPF. Linear interpolation from 16 kHz to
// 23.438 kHz leaves corner artifacts and aliasing that sound metallic on
// speech. Same 4 kHz biquad as capture; separate state so the two don't
// interfere.
static volatile float     g_plpfX1        = 0.0f;
static volatile float     g_plpfX2        = 0.0f;
static volatile float     g_plpfY1        = 0.0f;
static volatile float     g_plpfY2        = 0.0f;

// Per-second debug stats
static volatile uint32_t g_dmaBufCount  = 0;
static volatile int32_t  g_secSumAbs    = 0;
static volatile int32_t  g_secAvg       = 0;
static volatile int32_t  g_secPeak      = 0;
static volatile int32_t  g_secPeakLast  = 0;
static volatile uint32_t g_printReady   = 0;
static volatile uint32_t g_audadcIsrCount = 0;
static bool              g_micDebug     = true;

//*****************************************************************************
// I2S TX-only configuration (master, drives MAX98357 speaker amp)
//*****************************************************************************

static am_hal_i2s_transfer_t g_sTransfer;

static am_hal_i2s_io_signal_t g_sIOConfig =
{
    .eFyncCpol = AM_HAL_I2S_IO_FSYNC_CPOL_HIGH,
    .eTxCpol   = AM_HAL_I2S_IO_TX_CPOL_FALLING,
    .eRxCpol   = AM_HAL_I2S_IO_RX_CPOL_RISING,
};

static am_hal_i2s_data_format_t g_sDataConfig =
{
    .ePhase                   = AM_HAL_I2S_DATA_PHASE_SINGLE,
    .eDataDelay               = 0x1,
    .ui32ChannelNumbersPhase1 = 2,
    .ui32ChannelNumbersPhase2 = 2,
    .eDataJust                = AM_HAL_I2S_DATA_JUSTIFIED_LEFT,
    .eChannelLenPhase1        = AM_HAL_I2S_FRAME_WDLEN_32BITS,
    .eChannelLenPhase2        = AM_HAL_I2S_FRAME_WDLEN_32BITS,
    .eSampleLenPhase1         = AM_HAL_I2S_SAMPLE_LENGTH_24BITS,
    .eSampleLenPhase2         = AM_HAL_I2S_SAMPLE_LENGTH_24BITS,
};

static am_hal_i2s_config_t g_sI2SConfig =
{
    .eClock = eAM_HAL_I2S_CLKSEL_HFRC_1_5MHz,
    .eDiv3  = 0,
    .eASRC  = 0,
    .eMode  = AM_HAL_I2S_IO_MODE_MASTER,
    .eXfer  = AM_HAL_I2S_XFER_TX,
    .eData  = &g_sDataConfig,
    .eIO    = &g_sIOConfig,
};

//*****************************************************************************
// AUDADC configuration
//*****************************************************************************

static am_hal_audadc_dma_config_t g_sAUDADCDMAConfig =
{
    .bDynamicPriority         = true,
    .ePriority                = AM_HAL_AUDADC_PRIOR_SERVICE_IMMED,
    .bDMAEnable               = true,
    .ui32SampleCount          = AUDADC_DMA_WORDS,
    .ui32TargetAddress        = 0,
    .ui32TargetAddressReverse = 0,
};

static am_hal_audadc_gain_config_t g_sAudadcGainConfig =
{
    .ui32LGA      = 0,
    .ui32HGADELTA = 0,
    .ui32LGB      = 0,
    .ui32HGBDELTA = 0,
    .eUpdateMode  = AM_HAL_AUDADC_GAIN_UPDATE_IMME,
};

//*****************************************************************************
// Helpers
//*****************************************************************************

// AUDADC DMA word layout: 12-bit sample left-justified in bits[15:4] (low slot)
// and bits[31:20] (high slot). Cast to int16 sign-extends; result is full
// int16-scaled (~±32K) so no further shift is needed before resampling.
static inline int32_t extract_audadc_low_sample(uint32_t raw)
{
    return (int32_t)((int16_t)(raw & 0xFFF0));
}

static inline int32_t extract_audadc_high_sample(uint32_t raw)
{
    return (int32_t)((int16_t)((raw >> 16) & 0xFFF0));
}

static inline int32_t extract_record_sample(uint32_t raw, uint32_t src)
{
    return (src == MIC_SRC_A0 || src == MIC_SRC_B0)
           ? extract_audadc_low_sample(raw)
           : extract_audadc_high_sample(raw);
}

static inline bool audadc_word_matches_src(uint32_t raw, uint32_t src)
{
    bool isB = ((raw >> 19) & 1) != 0;
    return isB ? (src == MIC_SRC_B0 || src == MIC_SRC_B1)
               : (src == MIC_SRC_A0 || src == MIC_SRC_A1);
}

static inline uint32_t pack_i2s24_tx(int32_t sample)
{
    if (sample >  0x7FFFFF) sample =  0x7FFFFF;
    if (sample < -0x800000) sample = -0x800000;
    return (uint32_t)(sample & 0x00FFFFFF);
}

static inline int32_t soft_limit_i2s24(int32_t sample)
{
    if (sample > PLAY_LIMIT_START)
    {
        sample = PLAY_LIMIT_START + ((sample - PLAY_LIMIT_START) >> 2);
        if (sample > PLAY_LIMIT_MAX) sample = PLAY_LIMIT_MAX;
    }
    else if (sample < -PLAY_LIMIT_START)
    {
        sample = -PLAY_LIMIT_START + ((sample + PLAY_LIMIT_START) >> 2);
        if (sample < -PLAY_LIMIT_MAX) sample = -PLAY_LIMIT_MAX;
    }
    return sample;
}

//*****************************************************************************
// ISR — AUDADC0 (capture path: DC-block + resample to 16 kHz int16)
//*****************************************************************************

void am_audadc0_isr(void)
{
    uint32_t mask;
    am_hal_audadc_interrupt_status(g_pAUDADCHandle, &mask, false);
    am_hal_audadc_interrupt_clear(g_pAUDADCHandle, mask);

    g_audadcIsrCount++;

    if (!(mask & AM_HAL_AUDADC_INT_DCMP))
    {
        return;
    }

    am_hal_audadc_interrupt_service(g_pAUDADCHandle, &g_sAUDADCDMAConfig);

    const uint32_t *rxBuf =
        (const uint32_t *)am_hal_audadc_dma_get_buffer(g_pAUDADCHandle);

    int32_t sumAbs = 0;
    for (uint32_t i = 0; i < AUDADC_DMA_WORDS; i++)
    {
        if (!audadc_word_matches_src(rxBuf[i], MIC_RECORD_SOURCE)) continue;

        int32_t xo_raw = extract_record_sample(rxBuf[i], MIC_RECORD_SOURCE);

        // DC blocker: y[n] = x[n] - x[n-1] + R * y[n-1]
        int32_t xo_dc = xo_raw - g_dcPrevIn +
                        (int32_t)(((int64_t)DC_BLOCK_R_Q15 * g_dcPrevOut) >> 15);
        g_dcPrevIn  = xo_raw;
        g_dcPrevOut = xo_dc;

        float xf = (float)xo_dc;
        float yf = LPF_B0 * (xf + g_lpfX2)
                 + LPF_B1 * g_lpfX1
                 + LPF_A1_N * g_lpfY1
                 + LPF_A2_N * g_lpfY2;
        g_lpfX2 = g_lpfX1;
        g_lpfX1 = xf;
        g_lpfY2 = g_lpfY1;
        g_lpfY1 = yf;
        int32_t xo = (int32_t)yf;
		//int32_t xo = xo_dc;

        float abs_y = (yf >= 0.0f) ? yf : -yf;
        if (abs_y > g_gateEnv)
            g_gateEnv += GATE_ATTACK_A  * (abs_y - g_gateEnv);
        else
            g_gateEnv += GATE_RELEASE_A * (abs_y - g_gateEnv);

        int32_t ao = (xo < 0 ? -xo : xo);
		// END dc BLOCKER CODE
        sumAbs += ao;
        if (ao > g_secPeak) g_secPeak = ao;

        // Resample 23438 -> 16000 (active in LISTENING and RECORDING)
        if (g_mode == MODE_LISTENING || g_mode == MODE_RECORDING)
        {
            g_capAccum += 65536u;
            if (g_capAccum >= CAP_STEP_Q16)
            {
                g_capAccum -= CAP_STEP_Q16;
                int32_t frac = (int32_t)g_capAccum;
                int32_t diff = xo - g_capLast;
                int32_t s16  = g_capLast +
                               (int32_t)(((int64_t)diff * frac) >> 16);
                if (s16 >  32767) s16 =  32767;
                if (s16 < -32768) s16 = -32768;

                if (g_mode == MODE_RECORDING && g_recPos < REC_PCM_SAMPLES)
                {
                    float gate_gain;
                    if (g_gateEnv >= GATE_THRESH)
                    {
                        gate_gain = 1.0f;
                    }
                    else
                    {
                        float r = g_gateEnv * (1.0f / GATE_THRESH);
                        gate_gain = r * r;
                        if (gate_gain < GATE_FLOOR) gate_gain = GATE_FLOOR;
                    }
                    int32_t s16g = (int32_t)((float)s16 * gate_gain);
                    if (s16g >  32767) s16g =  32767;
                    if (s16g < -32768) s16g = -32768;
                    g_pcmBuf[g_recPos++] = (int16_t)s16g;
                }
#ifndef KWS_DISABLE
                else if (g_mode == MODE_LISTENING && g_kwsReadyBuf == 0xFF)
                {
                    int32_t kg = (int32_t)s16 * KWS_SW_GAIN;
                    if (kg >  32767) kg =  32767;
                    if (kg < -32768) kg = -32768;
                    int32_t ag = (kg < 0) ? -kg : kg;
                    if (ag > g_kwsWindowPeak) g_kwsWindowPeak = ag;

                    uint32_t wb = g_kwsWriteBuf;
                    g_kwsBuf[wb][g_kwsWritePos++] = (int16_t)kg;
                    if (g_kwsWritePos == KWS_FRAME_SAMPLES)
                    {
                        g_kwsWritePos = 0;
                        g_kwsReadyBuf = wb;
                        g_kwsWriteBuf = wb ^ 1u;
                    }
                }
#endif
            }
        }
        g_capLast = xo;
    }

    // Auto-stop recording when buffer is full
    if (g_mode == MODE_RECORDING && g_recPos >= REC_PCM_SAMPLES)
    {
        g_recLen  = g_recPos;
        g_mode    = MODE_IDLE;
        g_recDone = true;
    }

    // Per-second stats
    g_secSumAbs += sumAbs;
    g_dmaBufCount++;
    if (g_dmaBufCount >= SEC_CHUNKS)
    {
        g_secAvg      = g_secSumAbs / (SEC_CHUNKS * AUDADC_MONO_SAMPLES);
        g_secPeakLast = g_secPeak;
        g_secSumAbs   = 0;
        g_secPeak     = 0;
        g_dmaBufCount = 0;
        g_printReady  = 1;
    }
}

//*****************************************************************************
// ISR — I2S0 (TX-only: playback / test tone / silence)
//*****************************************************************************

void am_dspi2s0_isr(void)
{
    uint32_t status;
    am_hal_i2s_interrupt_status_get(g_pI2SHandle, &status, true);
    am_hal_i2s_interrupt_clear(g_pI2SHandle, status);
    am_hal_i2s_interrupt_service(g_pI2SHandle, status, &g_sI2SConfig);

    if (!(status & AM_HAL_I2S_INT_TXDMACPL))
    {
        return;
    }

    uint32_t *txBuf =
        (uint32_t *)am_hal_i2s_dma_get_buffer(g_pI2SHandle, AM_HAL_I2S_XFER_TX);

    for (uint32_t i = 0; i < MIC_DMA_SAMPLES; i += 2)
    {
        if (g_mode == MODE_PLAYING && g_playSrcIdx < g_recLen)
        {
            int16_t s0 = g_pcmBuf[g_playSrcIdx];
            int16_t s1 = (g_playSrcIdx + 1 < g_recLen)
                         ? g_pcmBuf[g_playSrcIdx + 1] : s0;
            int32_t out = (int32_t)s0 +
                          (int32_t)(((int64_t)(s1 - s0) * (int64_t)g_playPhaseQ16) >> 16);

            float pxf = (float)out;
            float pyf = LPF_B0 * (pxf + g_plpfX2)
                      + LPF_B1 * g_plpfX1
                      + LPF_A1_N * g_plpfY1
                      + LPF_A2_N * g_plpfY2;
            g_plpfX2 = g_plpfX1;
            g_plpfX1 = pxf;
            g_plpfY2 = g_plpfY1;
            g_plpfY1 = pyf;
            out = (int32_t)pyf;
			
			if (g_dbgPlayCapture == 1 && g_dbgPlayPos < DBG_PLAY_N)
            {
                g_dbgPlayOut[g_dbgPlayPos]   = out;
                g_dbgPlaySrc[g_dbgPlayPos]   = s0;
                g_dbgPlayPhase[g_dbgPlayPos] = (uint16_t)g_playPhaseQ16;
                g_dbgPlayPos++;
                if (g_dbgPlayPos >= DBG_PLAY_N) g_dbgPlayCapture = 2;
            }

            int32_t s24 = soft_limit_i2s24(out << PLAY_VOL_SHIFT);
            uint32_t tx = pack_i2s24_tx(s24);
            txBuf[i]     = tx;
            txBuf[i + 1] = tx;

            g_playPhaseQ16 += PLAY_STEP_Q16;
            while (g_playPhaseQ16 >= 65536u)
            {
                g_playPhaseQ16 -= 65536u;
                g_playSrcIdx++;
            }
        }
        else if (g_mode == MODE_TESTTONE && g_tonePos < TONE_DURATION)
        {
            int32_t val = ((g_tonePos / TONE_HALF_PERIOD) & 1)
                          ? -TONE_AMPLITUDE : TONE_AMPLITUDE;
            uint32_t tx = pack_i2s24_tx(val);
            txBuf[i]     = tx;
            txBuf[i + 1] = tx;
            g_tonePos++;
        }
        else
        {
            txBuf[i]     = 0;
            txBuf[i + 1] = 0;
        }
    }

    if (g_mode == MODE_PLAYING && g_playSrcIdx >= g_recLen)
    {
        g_mode     = MODE_IDLE;
        g_playDone = true;
    }
    if (g_mode == MODE_TESTTONE && g_tonePos >= TONE_DURATION)
    {
        g_mode     = MODE_IDLE;
        g_toneDone = true;
    }
}

//*****************************************************************************
// Init
//*****************************************************************************

static void mic_i2s_tx_init(uint32_t txPtrA, uint32_t txPtrB)
{
    am_hal_gpio_pincfg_t outCfg = { .GP.cfg_b.eGPOutCfg = 1, .GP.cfg_b.ePullup = 0 };

    outCfg.GP.cfg_b.uFuncSel = MIC_CLK_FUNC;
    am_hal_gpio_pinconfig(MIC_CLK_PIN, outCfg);

	// MAX98357A SD control — plain push-pull GPIO output (function 3 = GPIO on
    // Apollo4), NOT an I2S alt-function like the pins above. Start LOW = amp off.
    am_hal_gpio_pincfg_t ampSdCfg = { 0 };
    ampSdCfg.GP.cfg_b.eGPOutCfg = 1;   // push-pull output
    ampSdCfg.GP.cfg_b.ePullup   = 0;   // no pullup
    ampSdCfg.GP.cfg_b.uFuncSel  = AM_HAL_PIN_8_GPIO;   // = 3, GPIO function for pin 8
    am_hal_gpio_pinconfig(AMP_SD_PIN, ampSdCfg);
    am_hal_gpio_state_write(AMP_SD_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);

    outCfg.GP.cfg_b.uFuncSel = MIC_WS_FUNC;
    am_hal_gpio_pinconfig(MIC_WS_PIN, outCfg);

    outCfg.GP.cfg_b.uFuncSel = MIC_SDOUT_FUNC;
    am_hal_gpio_pinconfig(MIC_SDOUT_PIN, outCfg);

    am_hal_i2s_initialize(MIC_I2S_MODULE, &g_pI2SHandle);
    am_hal_i2s_power_control(g_pI2SHandle, AM_HAL_I2S_POWER_ON, false);
    am_hal_i2s_configure(g_pI2SHandle, &g_sI2SConfig);
    am_hal_i2s_enable(g_pI2SHandle);

    g_sTransfer.ui32RxTotalCount        = 0;
    g_sTransfer.ui32RxTargetAddr        = 0;
    g_sTransfer.ui32RxTargetAddrReverse = 0;
    g_sTransfer.ui32TxTotalCount        = MIC_DMA_SAMPLES;
    g_sTransfer.ui32TxTargetAddr        = txPtrA;
    g_sTransfer.ui32TxTargetAddrReverse = txPtrB;

    am_hal_i2s_dma_configure(g_pI2SHandle, &g_sI2SConfig, &g_sTransfer);

    NVIC_SetPriority(I2S0_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(I2S0_IRQn);
}

static void mic_audadc_init(uint32_t dmaPtrA, uint32_t dmaPtrB)
{
    // Power up AUDADC analog front-end. mic_bias must be enabled even though
    // CMM-2718AB has its own integrated bias; it powers internal converter
    // biasing and without it the AUDADC produces zero output.
    am_hal_audadc_refgen_powerup();
    am_hal_audadc_pga_powerup(0);
    am_hal_audadc_pga_powerup(1);
    am_hal_audadc_pga_powerup(2);
    am_hal_audadc_pga_powerup(3);
    am_hal_audadc_gain_set(0, 2 * PREAMP_GAIN_DB);
    am_hal_audadc_gain_set(1, 2 * PREAMP_GAIN_DB);
    am_hal_audadc_gain_set(2, 2 * PREAMP_GAIN_DB);
    am_hal_audadc_gain_set(3, 2 * PREAMP_GAIN_DB);
    am_hal_audadc_micbias_powerup(MIC_BIAS_TRIM);
    am_util_delay_ms(400);   // bias/refgen settle (per Ambiq example)

    am_hal_audadc_config_t audadcCfg =
    {
        .eClock         = AM_HAL_AUDADC_CLKSEL_HFRC_48MHz,
        .ePolarity      = AM_HAL_AUDADC_TRIGPOL_RISING,
        .eTrigger       = AM_HAL_AUDADC_TRIGSEL_SOFTWARE,
        .eClockMode     = AM_HAL_AUDADC_CLKMODE_LOW_LATENCY,
        .ePowerMode     = AM_HAL_AUDADC_LPMODE0,
        .eRepeat        = AM_HAL_AUDADC_REPEATING_SCAN,
        .eRepeatTrigger = AM_HAL_AUDADC_RPTTRIGSEL_INT,
        .eSampMode      = AM_HAL_AUDADC_SAMPMODE_MED,
    };

    // IRTT: HFRC 48 MHz / div32 / (63+1) = 23,437.5 Hz — matches I2S TX rate.
    am_hal_audadc_irtt_config_t irttCfg =
    {
        .bIrttEnable      = true,
        .eClkDiv          = AM_HAL_AUDADC_RPTT_CLK_DIV32,
        .ui32IrttCountMax = 63,
    };

    am_hal_audadc_initialize(0, &g_pAUDADCHandle);
    am_hal_audadc_power_control(g_pAUDADCHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_audadc_configure(g_pAUDADCHandle, &audadcCfg);
    am_hal_audadc_configure_irtt(g_pAUDADCHandle, &irttCfg);
    am_hal_audadc_enable(g_pAUDADCHandle);
    am_hal_audadc_irtt_enable(g_pAUDADCHandle);

    // 4 slots all 12-bit. Audio comes in on SE0 (channel A LG); SE1-SE3 are
    // configured per the standard Ambiq reference and the ISR ignores them.
    am_hal_audadc_slot_config_t slotCfg =
    {
        .eMeasToAvg     = AM_HAL_AUDADC_SLOT_AVG_1,
        .ui32TrkCyc     = 34,
        .ePrecisionMode = AM_HAL_AUDADC_SLOT_12BIT,
        .bWindowCompare = false,
        .bEnabled       = true,
    };
    slotCfg.eChannel = AM_HAL_AUDADC_SLOT_CHSEL_SE0;
    am_hal_audadc_configure_slot(g_pAUDADCHandle, 0, &slotCfg);
    slotCfg.eChannel = AM_HAL_AUDADC_SLOT_CHSEL_SE1;
    am_hal_audadc_configure_slot(g_pAUDADCHandle, 1, &slotCfg);
    slotCfg.eChannel = AM_HAL_AUDADC_SLOT_CHSEL_SE2;
    am_hal_audadc_configure_slot(g_pAUDADCHandle, 2, &slotCfg);
    slotCfg.eChannel = AM_HAL_AUDADC_SLOT_CHSEL_SE3;
    am_hal_audadc_configure_slot(g_pAUDADCHandle, 3, &slotCfg);

    g_sAudadcGainConfig.ui32LGA      = (uint32_t)(CHANNEL_GAIN_DB * 2 + 12);
    g_sAudadcGainConfig.ui32HGADELTA = 0;
    g_sAudadcGainConfig.ui32LGB      = (uint32_t)(CHANNEL_GAIN_DB * 2 + 12);
    g_sAudadcGainConfig.ui32HGBDELTA = 0;
    g_sAudadcGainConfig.eUpdateMode  = AM_HAL_AUDADC_GAIN_UPDATE_IMME;
    am_hal_audadc_internal_pga_config(g_pAUDADCHandle, &g_sAudadcGainConfig);

    g_sAUDADCDMAConfig.ui32TargetAddress        = dmaPtrA;
    g_sAUDADCDMAConfig.ui32TargetAddressReverse = dmaPtrB;
    am_hal_audadc_configure_dma(g_pAUDADCHandle, &g_sAUDADCDMAConfig);

    am_hal_audadc_interrupt_enable(g_pAUDADCHandle,
        AM_HAL_AUDADC_INT_DERR | AM_HAL_AUDADC_INT_DCMP);

    NVIC_SetPriority(AUDADC0_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(AUDADC0_IRQn);
}

// Enable the MAX98357A only during playback/tone; keep it shut down (quiet)
// during listening/recording/idle so its Class-D switching can't couple noise
// into the mic. Tracks the last state to avoid redundant GPIO writes.
static void amp_update_state(void)
{
    static int lastOn = -1;
    int on = (g_mode == MODE_PLAYING || g_mode == MODE_TESTTONE) ? 1 : 0;
    if (on != lastOn)
    {
        am_hal_gpio_state_write(AMP_SD_PIN,
            on ? AM_HAL_GPIO_OUTPUT_SET : AM_HAL_GPIO_OUTPUT_CLEAR);
        lastOn = on;
    }
}
//*****************************************************************************
// FreeRTOS task
//*****************************************************************************

void MicTask(void *pvParameters)
{
    (void)pvParameters;

    // Align I2S TX buffers to 16 bytes
    uint32_t txPtrA = (uint32_t)((uint32_t)(g_txBufRawA + 3) & ~0xF);
    uint32_t txPtrB = (uint32_t)((uint32_t)(g_txBufRawB + 3) & ~0xF);
    memset((void *)txPtrA, 0, MIC_DMA_SAMPLES * sizeof(uint32_t));
    memset((void *)txPtrB, 0, MIC_DMA_SAMPLES * sizeof(uint32_t));

    // Align AUDADC ping-pong buffers (contiguous in one allocation)
    uint32_t audadcBase = (uint32_t)((uint32_t)(g_audadcBufRaw + 3) & ~0xF);
    uint32_t dmaPtrA    = audadcBase;
    uint32_t dmaPtrB    = audadcBase + AUDADC_DMA_WORDS * sizeof(uint32_t);

    // BUTTON1 (SW2, GPIO 19): input with 100K pull-up, active low
    am_hal_gpio_pincfg_t btnCfg = {0};
    btnCfg.GP.cfg_b.eGPOutCfg = 0;
    btnCfg.GP.cfg_b.eGPInput  = 1;
    btnCfg.GP.cfg_b.ePullup   = 6;   // AM_HAL_GPIO_PIN_PULLUP_100K
    btnCfg.GP.cfg_b.uFuncSel  = 3;
    am_hal_gpio_pinconfig(BTN_PIN, btnCfg);

    am_util_stdio_printf("[mic] task entered\r\n");

#ifndef KWS_DISABLE
    // Defer KWS init briefly so BLE can finish reset/advertising bring-up.
    am_util_stdio_printf("[mic] waiting 3s before kws_init\r\n");
    vTaskDelay(pdMS_TO_TICKS(3000));

    am_util_stdio_printf("[mic] >kws_init\r\n");
    kws_init();
    am_util_stdio_printf("[mic] <kws_init\r\n");

    g_mfccCfg.api              = &ns_mfcc_V1_0_0;
    g_mfccCfg.arena            = g_mfccArena;
    g_mfccCfg.sample_frequency = REC_PCM_RATE;
    g_mfccCfg.num_fbank_bins   = KWS_MFCC_FBANK_BINS;
    g_mfccCfg.low_freq         = 20;
    g_mfccCfg.high_freq        = 4000;
    g_mfccCfg.num_frames       = KWS_NUM_FRAMES;
    g_mfccCfg.num_coeffs       = KWS_MFCC_COEFFS;
    g_mfccCfg.num_dec_bits     = 0;
    g_mfccCfg.frame_shift_ms   = 20;
    g_mfccCfg.frame_len_ms     = 30;
    g_mfccCfg.frame_len        = KWS_FRAME_SAMPLES;
    g_mfccCfg.frame_len_pow2   = KWS_MFCC_FRAME_POW2;
    am_util_stdio_printf("[mic] >ns_mfcc_init\r\n");
    ns_mfcc_init(&g_mfccCfg);
    am_util_stdio_printf("[mic] <ns_mfcc_init\r\n");
#endif

    am_util_stdio_printf("[mic] >mic_audadc_init\r\n");
    mic_audadc_init(dmaPtrA, dmaPtrB);
    am_util_stdio_printf("[mic] <mic_audadc_init\r\n");

    am_util_stdio_printf("[mic] >mic_i2s_tx_init\r\n");
    mic_i2s_tx_init(txPtrA, txPtrB);
    am_util_stdio_printf("[mic] <mic_i2s_tx_init\r\n");

    am_hal_interrupt_master_enable();

    // Kick off AUDADC repeating scan; IRTT takes over after first trigger.
    am_hal_audadc_sw_trigger(g_pAUDADCHandle);
    am_util_stdio_printf("[mic] AUDADC capture started\r\n");

    // I2S TX runs continuously; outputs silence until playback or tone.
    am_hal_i2s_dma_transfer_start(g_pI2SHandle, &g_sI2SConfig);
    am_util_stdio_printf("[mic] I2S TX DMA started\r\n");

#ifndef KWS_DISABLE
    am_util_stdio_printf("[mic] Listening for \"%s\". Short press=record, long press(2s)=test tone.\n",
                         kws_label_names[KWS_TRIGGER_IDX]);
#else
    am_util_stdio_printf("[mic] Ready. Short press=record/play, long press(2s)=test tone.\n");
#endif

    TickType_t xLastWake  = xTaskGetTickCount();
    bool     btnPrev      = false;
    uint32_t btnDead      = 0;
    uint32_t btnHoldCnt   = 0;
    uint32_t diagTicks    = 0;

    while (1)
    {
        vTaskDelayUntil(&xLastWake, 10);
		
		amp_update_state();

        // ---- Button polling: short press vs long press (2 sec) ----
        uint32_t btnVal;
        am_hal_gpio_state_read(BTN_PIN, AM_HAL_GPIO_INPUT_READ, &btnVal);
        bool btnDown = (btnVal == 0);

        if (btnDead > 0)
        {
            btnDead--;
        }
        else if (btnDown && !btnPrev)
        {
            btnHoldCnt = 0;
        }
        else if (btnDown && btnPrev)
        {
            btnHoldCnt++;
            if (btnHoldCnt == 200 && (g_mode == MODE_IDLE || g_mode == MODE_LISTENING))
            {
                static const int16_t sineLut[16] = {
                        0,  3033,  5605,  7368,  8000,  7368,  5605,  3033,
                        0, -3033, -5605, -7368, -8000, -7368, -5605, -3033
                };
                for (uint32_t n = 0; n < REC_PCM_SAMPLES; n++)
                {
                    uint32_t p300  = (n * 300u  * 16u / REC_PCM_RATE) & 15u;
                    uint32_t p900  = (n * 900u  * 16u / REC_PCM_RATE) & 15u;
                    uint32_t p2000 = (n * 2000u * 16u / REC_PCM_RATE) & 15u;

                    int32_t mix = ((int32_t)sineLut[p300]  * 4) / 10
                                + ((int32_t)sineLut[p900]  * 4) / 10
                                + ((int32_t)sineLut[p2000] * 2) / 10;

                    if (mix >  32767) mix =  32767;
                    if (mix < -32768) mix = -32768;
                    g_pcmBuf[n] = (int16_t)mix;
                }
                g_recLen  = REC_PCM_SAMPLES;
                g_recDone = true;
                g_mode    = MODE_IDLE;
                am_util_stdio_printf("[mic] Synth multi-tone loaded into g_pcmBuf. Short press to play.\n");
                btnDead = 5;
            }
        }
        else if (!btnDown && btnPrev)
        {
            if (btnHoldCnt < 200 && (g_mode == MODE_IDLE || g_mode == MODE_LISTENING))
            {
                btnDead = 5;
                if (g_recLen > 0)
                {
                    g_playSrcIdx   = 0;
                    g_playPhaseQ16 = 0;
					g_dbgPlayPos     = 0;
                    g_dbgPlayCapture = 1;
                    g_mode         = MODE_PLAYING;
                    am_util_stdio_printf("[mic] Playing %lu PCM samples (%lu sec)...\n",
                                         (unsigned long)g_recLen,
                                         (unsigned long)(g_recLen / REC_PCM_RATE));
                }
                else
                {
                    g_recPos    = 0;
                    g_capAccum  = 0;
                    g_opusLen   = 0;
                    g_mode      = MODE_RECORDING;
                    am_util_stdio_printf("[mic] Recording %d sec...\n", REC_SECONDS);
                }
            }
            btnHoldCnt = 0;
        }
        btnPrev = btnDown;

#ifndef KWS_DISABLE
        // ---- KWS: process one 20 ms frame when the ISR hands one over ----
        uint32_t readyBuf = g_kwsReadyBuf;
        if (readyBuf != 0xFF && g_mode == MODE_LISTENING)
        {
            float confidence = 0.0f;
            ns_mfcc_compute(&g_mfccCfg,
                            (const int16_t *)g_kwsBuf[readyBuf],
                            &g_mfccFeatures[g_mfccFrameCount * KWS_MFCC_COEFFS]);
            g_mfccFrameCount++;
            g_kwsReadyBuf = 0xFF;

            if (g_mfccFrameCount >= KWS_NUM_FRAMES)
            {
                g_mfccFrameCount = 0;
                int32_t winPeak = g_kwsWindowPeak;
                g_kwsWindowPeak = 0;

                int keyword = -1;
                if (winPeak >= KWS_PEAK_GATE)
                {
                    int top2_idx = -1;
                    float top2_conf = 0.0f;
                    keyword = kws_run_top2(g_mfccFeatures,
                                           &keyword, &confidence,
                                           &top2_idx, &top2_conf);
                    if (keyword >= 0 && keyword != KWS_SILENCE_IDX && keyword != KWS_UNKNOWN_IDX)
                    {
                        am_util_stdio_printf("[kws] \"%s\" %.0f%% (2nd=\"%s\" %.0f%%, peak=%ld)\n",
                                            kws_label_names[keyword], confidence * 100.0f,
                                            (top2_idx >= 0 ? kws_label_names[top2_idx] : "?"),
                                            top2_conf * 100.0f,
                                            (long)winPeak);
                    }
                }
                if (keyword == KWS_TRIGGER_IDX && confidence >= KWS_THRESHOLD)
                {
                    am_util_stdio_printf("[kws] TRIGGER -> recording %d sec...\n",
                                        REC_SECONDS);
                    g_recPos   = 0;
                    g_capAccum = 0;
                    g_opusLen  = 0;
                    g_mode     = MODE_RECORDING;
                }
            }
        }
#endif

        // ---- ISR notifications ----
        if (g_recDone)
        {
            g_recDone = false;

            // Log raw PCM stats for diagnostics
            int32_t peakAbs = 0;
            for (uint32_t k = 0; k < g_recLen; k++)
            {
                int32_t a = (g_pcmBuf[k] < 0) ? -g_pcmBuf[k] : g_pcmBuf[k];
                if (a > peakAbs) peakAbs = a;
            }
            am_util_stdio_printf("[mic] PCM @16kHz: %lu samples, peak=%ld\n",
                                 (unsigned long)g_recLen, (long)peakAbs);
 
			// --- RMS over the ENTIRE recording ---
            {
                uint64_t sumSquares = 0;
                for (uint32_t k = 0; k < g_recLen; k++)
                {
                    int32_t s = g_pcmBuf[k];
                    sumSquares += (uint64_t)((int64_t)s * (int64_t)s);
                }
                uint32_t meanSquare = (g_recLen > 0)
                                    ? (uint32_t)(sumSquares / g_recLen) : 0;
                uint32_t rms = 0;
                if (meanSquare > 0)
                {
                    rms = meanSquare;
                    for (int it = 0; it < 16; it++)
                        rms = (rms + meanSquare / rms) / 2;   // integer sqrt
                }
                am_util_stdio_printf("[mic] RMS (full): %lu  over %lu samples\n",
                                     (unsigned long)rms, (unsigned long)g_recLen);
            }
			
			// --- RMS with the first and last second trimmed off ---
            // Skips startup/shutdown transients that skew the noise figure.
            {
                uint32_t trim = REC_PCM_RATE;          // 1 second of samples
                if (g_recLen > (2 * trim))             // only if long enough
                {
                    uint32_t startIdx = trim;
                    uint32_t endIdx   = g_recLen - trim;   // exclusive
                    uint32_t nTrimmed = endIdx - startIdx;

                    uint64_t sumSquares = 0;
                    int32_t  peakTrim   = 0;
                    for (uint32_t k = startIdx; k < endIdx; k++)
                    {
                        int32_t s = g_pcmBuf[k];
                        int32_t a = (s < 0) ? -s : s;
                        if (a > peakTrim) peakTrim = a;
                        sumSquares += (uint64_t)((int64_t)s * (int64_t)s);
                    }
                    uint32_t meanSquare = (uint32_t)(sumSquares / nTrimmed);
                    uint32_t rms = 0;
                    if (meanSquare > 0)
                    {
                        rms = meanSquare;
                        for (int it = 0; it < 16; it++)
                            rms = (rms + meanSquare / rms) / 2;   // integer sqrt
                    }
                    am_util_stdio_printf("[mic] RMS (trimmed): %lu  peak=%ld  over %lu samples (%lus-%lus)\n",
                                         (unsigned long)rms,
                                         (long)peakTrim,
                                         (unsigned long)nTrimmed,
                                         (unsigned long)(startIdx / REC_PCM_RATE),
                                         (unsigned long)(endIdx / REC_PCM_RATE));
                }
                else
                {
                    am_util_stdio_printf("[mic] RMS (trimmed): recording too short to trim\n");
                }
            }

            // Encode to Opus (32 kbit/s CBR, 20 ms frames, 16 kHz mono)
            uint32_t totalFrames = g_recLen / OPUS_FRAME_SAMPLES;
            if (totalFrames > OPUS_NUM_FRAMES) totalFrames = OPUS_NUM_FRAMES;

            am_util_stdio_printf("[mic] Encoding %lu Opus frames...\n",
                                 (unsigned long)totalFrames);
            audio_enc_init(0);

            uint32_t opusBytes = 0;
            for (uint32_t f = 0; f < totalFrames; f++)
            {
                int n = audio_enc_encode_frame(
                            (short *)&g_pcmBuf[f * OPUS_FRAME_SAMPLES],
                            OPUS_FRAME_SAMPLES,
                            &g_opusBuf[opusBytes]);
                if (n <= 0 || opusBytes + n > OPUS_BUF_BYTES) break;
                opusBytes += (uint32_t)n;
            }
            g_opusLen = opusBytes;
            am_util_stdio_printf("[mic] Opus encode done: %lu bytes (~%lu kbit/s)\n",
                                 (unsigned long)opusBytes,
                                 (unsigned long)((opusBytes * 8) / REC_SECONDS / 1000));
            am_util_stdio_printf("[mic] Press short=play (PCM), hold 2s=test tone.\n");
#ifndef KWS_DISABLE
            g_mfccFrameCount = 0;
            g_kwsReadyBuf    = 0xFF;
            g_kwsWritePos    = 0;
#endif
            g_mode           = MIC_MODE_INITIAL;
        }
        if (g_toneDone)
        {
            g_toneDone = false;
            am_util_stdio_printf("[mic] Test tone done.\n");
#ifndef KWS_DISABLE
            g_mfccFrameCount = 0;
            g_kwsReadyBuf    = 0xFF;
            g_kwsWritePos    = 0;
#endif
            g_mode           = MIC_MODE_INITIAL;
        }
        if (g_playDone)
        {
            g_playDone = false;
            am_util_stdio_printf("[mic] Playback done.\n");
#ifndef KWS_DISABLE
            g_mfccFrameCount = 0;
            g_kwsReadyBuf    = 0xFF;
            g_kwsWritePos    = 0;
#endif
            g_mode           = MIC_MODE_INITIAL;
        }
		
        if (g_dbgPlayCapture == 2)
        {
            g_dbgPlayCapture = 0;
            am_util_stdio_printf("[play] idx  src    out    phase\n");
            for (uint32_t d = 0; d < DBG_PLAY_N; d++)
            {
                am_util_stdio_printf("  %2lu  %6d %6ld  %5u\n",
                                     (unsigned long)d,
                                     (int)g_dbgPlaySrc[d],
                                     (long)g_dbgPlayOut[d],
                                     (unsigned)g_dbgPlayPhase[d]);
            }
        }

        // ---- AUDADC liveness diagnostic every 3 seconds ----
        diagTicks++;
        if (diagTicks >= 300)
        {
            diagTicks = 0;
            if (g_micDebug)
            {
                am_util_stdio_printf("[mic] audadc_isr_count=%lu recPos=%lu\n",
                                     (unsigned long)g_audadcIsrCount,
                                     (unsigned long)g_recPos);
            }
        }

        // ---- Per-second stats ----
        if (g_printReady)
        {
            g_printReady = 0;
            if (g_micDebug)
            {
                const char *mstr = (g_mode == MODE_RECORDING) ? "REC" :
                                   (g_mode == MODE_PLAYING)   ? "PLAY" :
                                   (g_mode == MODE_TESTTONE)  ? "TONE" : "IDLE";
                am_util_stdio_printf("[mic] avg=%ld peak=%ld mode=%s\n",
                                     (long)g_secAvg, (long)g_secPeakLast, mstr);
            }
        }
    }
}

//*****************************************************************************
// Public accessor for the encoded Opus buffer (used by the BLE stream svc).
//*****************************************************************************
const uint8_t *MicTaskGetOpusData(uint32_t *pLen)
{
    if (pLen != NULL)
    {
        *pLen = g_opusLen;
    }
    return g_opusBuf;
}
