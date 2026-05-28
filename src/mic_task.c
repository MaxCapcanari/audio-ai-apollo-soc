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
#include <math.h>
#include <float.h>

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
#define PREAMP_GAIN_DB      12
#define CHANNEL_GAIN_DB     6

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
#define REC_SECONDS         5
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
// Chewallow MFCC pipeline mirrors python_speech_features.mfcc:
//   winlen=0.01, winstep=0.01, nfft=512, nfilt=32, numcep=23
// Per frame: 160 audio samples (10 ms @ 16 kHz), FFT padded to 512.
// We compute 23 raw MFCC coefficients then drop coefficient 0 → 22 retained.
#define KWS_FRAME_SAMPLES       160     // 10 ms @ 16 kHz
#define KWS_MFCC_RAW_COEFFS     23      // ns_mfcc output count, before drop
#define KWS_MFCC_COEFFS         22      // model input coeffs (= KWS_NUM_COEFFS)
#define KWS_MFCC_FBANK_BINS     32
#define KWS_MFCC_FRAME_POW2     512

// neuralSPOT ns_mfcc_map_arena / ns_fbanks_map_arena multiply pointer-arithmetic
// offsets by sizeof(T) on a T* pointer, inflating the layout by 4×. The
// theoretical layout for chewallow is ~94 KB; 128 KB gives margin.
#define MFCC_ARENA_BYTES        (64 * 1024)
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
// Final z-score normalized MFCC tensor fed to the model: 250 frames × 22 coeffs.
AM_SHARED_RW static float    g_mfccFeatures[KWS_NUM_FRAMES * KWS_NUM_COEFFS];
// Per-frame scratch: ns_mfcc emits 23 coeffs; we copy coeffs 1..22 into the
// feature block, dropping coefficient 0 per the training pipeline.
static float                 g_mfccRawScratch[KWS_MFCC_RAW_COEFFS];
static uint32_t              g_mfccFrameCount = 0;
static ns_mfcc_cfg_t         g_mfccCfg;

// Software gain applied only on the KWS feed. The chewallow model expects
// earpiece-captured audio (typical peaks 200..5000 in int16); the AUDADC air
// mic delivers similar levels at conversational distance, so 1x by default.
#define KWS_SW_GAIN          1

// Energy gate: peak |sample| over the 2.5-second window must exceed this to
// invoke the model. Skips dead-silence inference, doesn't replace the model's
// own "nothing" judgment.
#define KWS_PEAK_GATE        200
static volatile int32_t      g_kwsWindowPeak = 0;

// python_speech_features.mfcc defaults that the training script uses.
// ns_mfcc out of the box does Hann-windowed, magnitude (not power) filterbanks,
// no pre-emphasis, no lifter — none of which match the trained pipeline. We
// reuse ns_mfcc's mel filterbank + DCT matrix but compute the rest locally.
#define PSF_PREEMPH_Q15      31785      // round(0.97 * 32768)
#define PSF_CEPLIFTER        22
static int16_t kws_preemph_prev   = 0;
static bool    kws_preemph_first  = true;
static float   kws_lifter[KWS_MFCC_RAW_COEFFS];

// arm_rfft_fast_instance set up by ns_mfcc_init() — we reuse it.
extern arm_rfft_fast_instance_f32 g_mfccRfft;
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

// Anti-alias / de-hiss biquad LPF (Butterworth 2nd order, fc≈4 kHz @ 23.438 kHz,
// Q=0.707). The 12-bit AUDADC has a broadband noise floor; without a low-pass
// before the 23.438→16 kHz decimator, content above 8 kHz folds back into the
// audio band as white-noise hiss. 4 kHz cutoff preserves speech intelligibility
// (which lives in 0.3–3.4 kHz) while killing most of the aliasing energy.
//   y[n] = b0(x[n] + x[n-2]) + b1*x[n-1] + a1_n*y[n-1] + a2_n*y[n-2]
//   (a1_n, a2_n are the negated, a0-normalized denominator coeffs)
#define LPF_B0     0.1615f
#define LPF_B1     0.3231f
#define LPF_A1_N   0.5871f
#define LPF_A2_N  -0.2333f
static volatile float     g_lpfX1         = 0.0f;
static volatile float     g_lpfX2         = 0.0f;
static volatile float     g_lpfY1         = 0.0f;
static volatile float     g_lpfY2         = 0.0f;

// Downward expander (soft noise gate). The 12-bit AUDADC + 18 dB analog gain
// puts the in-band noise floor around 200–500 LSB peak (int16-scaled). Speech
// peaks at 1500–8000 LSB. Below the threshold we square the gain (3:1
// expansion), so noise-floor samples get ~30+ dB of attenuation while speech
// passes unchanged. Asymmetric envelope follower (fast attack, slow release)
// avoids "pumping" on speech transients.
//
//   gain(env) = 1                      if env >= GATE_THRESH
//   gain(env) = (env/GATE_THRESH)^2    if env <  GATE_THRESH  (clipped to GATE_FLOOR)
//
// Tune GATE_THRESH up if hiss is still audible during pauses; down if the
// start of soft words gets chopped. GATE_FLOOR sets the residual hiss level.
#define GATE_THRESH        600.0f
#define GATE_FLOOR         0.03f      // -30 dB residual
#define GATE_ATTACK_A      0.20f      // ~0.4 ms attack @ 23.4 kHz
#define GATE_RELEASE_A     0.0015f    // ~30 ms release @ 23.4 kHz
static volatile float     g_gateEnv       = 0.0f;

// Playback-side resampler state (16000 Hz int16 -> 23438 Hz int24)
static volatile uint32_t  g_playPhaseQ16  = 0;
static volatile uint32_t  g_playSrcIdx    = 0;

// Per-second debug stats
static volatile uint32_t g_dmaBufCount  = 0;
static volatile int32_t  g_secSumAbs    = 0;
static volatile int32_t  g_secAvg       = 0;
static volatile int32_t  g_secPeak      = 0;
static volatile int32_t  g_secPeakLast  = 0;
static volatile uint32_t g_printReady   = 0;
static volatile uint32_t g_audadcIsrCount = 0;
static bool              g_micDebug     = false;

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

#ifndef KWS_DISABLE
//*****************************************************************************
// MFCC compute that matches python_speech_features.mfcc defaults
//   pre-emphasis 0.97, rectangular window, power spectrum, log mel,
//   DCT-II ortho, sinusoidal lifter (L=22)
// Reuses g_mfccCfg's mel filterbank + DCT matrix that ns_mfcc_init built.
//*****************************************************************************

static void kws_mfcc_reset_state(void)
{
    kws_preemph_prev  = 0;
    kws_preemph_first = true;
}

static void kws_mfcc_post_init(void)
{
    // Override ns_mfcc's Hann window with a rectangular window (PSF default).
    for (uint32_t i = 0; i < g_mfccCfg.frame_len; i++) {
        g_mfccCfg.mfccWindowFunction[i] = 1.0f;
    }
    // Sinusoidal lifter table: lift[n] = 1 + (L/2) * sin(pi*n/L), L=22.
    for (int i = 0; i < KWS_MFCC_RAW_COEFFS; i++) {
        kws_lifter[i] = 1.0f + ((float)PSF_CEPLIFTER / 2.0f) *
                                sinf((float)M_PI * (float)i / (float)PSF_CEPLIFTER);
    }
    kws_mfcc_reset_state();
}

static void kws_mfcc_compute_psf(const int16_t *audio_data, float *mfcc_out)
{
    ns_mfcc_cfg_t *cfg = &g_mfccCfg;
    const uint32_t flen  = cfg->frame_len;
    const uint32_t fpow2 = cfg->frame_len_pow2;

    // Pre-emphasis: y[n] = x[n] - 0.97*x[n-1]. The very first sample of the
    // very first frame in a clip is kept as-is (PSF behavior). Result is then
    // normalized int16 -> float in [-1, 1) and written into mfccFrame. No
    // window multiply needed since we forced the window to all 1.0f.
    int16_t prev;
    uint32_t i_start;
    if (kws_preemph_first) {
        cfg->mfccFrame[0] = (float)audio_data[0] / 32768.0f;
        prev = audio_data[0];
        i_start = 1;
        kws_preemph_first = false;
    } else {
        prev = kws_preemph_prev;
        i_start = 0;
    }
    for (uint32_t i = i_start; i < flen; i++) {
        int32_t pe = (int32_t)audio_data[i] -
                     (int32_t)(((int32_t)PSF_PREEMPH_Q15 * (int32_t)prev) >> 15);
        cfg->mfccFrame[i] = (float)pe / 32768.0f;
        prev = audio_data[i];
    }
    kws_preemph_prev = audio_data[flen - 1];

    // Zero-pad to FFT length.
    memset(&cfg->mfccFrame[flen], 0, sizeof(float) * (fpow2 - flen));

    // Real FFT.
    arm_rfft_fast_f32(&g_mfccRfft, cfg->mfccFrame, cfg->mfccBuffer, 0);

    // Power spectrum |X|^2. arm_rfft_fast packs [DC_real, Nyquist_real, ...].
    int32_t half = (int32_t)fpow2 / 2;
    float dc_pow = cfg->mfccBuffer[0] * cfg->mfccBuffer[0];
    float ny_pow = cfg->mfccBuffer[1] * cfg->mfccBuffer[1];
    for (int32_t k = 1; k < half; k++) {
        float re = cfg->mfccBuffer[k * 2];
        float im = cfg->mfccBuffer[k * 2 + 1];
        cfg->mfccBuffer[k] = re * re + im * im;
    }
    cfg->mfccBuffer[0]    = dc_pow;
    cfg->mfccBuffer[half] = ny_pow;

    // Mel filterbank on power spectrum (PSF behavior — NOT magnitude as ns_mfcc).
    for (uint32_t bin = 0; bin < cfg->num_fbank_bins; bin++) {
        float mel_energy = 0.0f;
        int32_t first_index = cfg->fbc.mfccFbankFirst[bin];
        int32_t last_index  = cfg->fbc.mfccFbankLast[bin];
        int j = 0;
        for (int32_t k = first_index; k <= last_index; k++) {
            mel_energy += cfg->mfccBuffer[k] * (*(cfg->fbc.melFBank))[bin][j++];
        }
        if (mel_energy < FLT_MIN) mel_energy = FLT_MIN;
        cfg->mfccEnergies[bin] = logf(mel_energy);
    }

    // DCT-II using the matrix ns_mfcc_init already built (sqrt(2/N) normalized).
    // bin 0 has the wrong normalization vs scipy 'ortho' but the caller drops it.
    for (uint32_t i = 0; i < cfg->num_coeffs; i++) {
        float sum = 0.0f;
        for (uint32_t j = 0; j < cfg->num_fbank_bins; j++) {
            sum += cfg->mfccDCTMatrix[i * cfg->num_fbank_bins + j] *
                   cfg->mfccEnergies[j];
        }
        mfcc_out[i] = sum * kws_lifter[i];
    }
}
#endif

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

        // Anti-alias LPF (4 kHz Butterworth) — kills the AUDADC's high-freq
        // noise floor before it aliases into the 16 kHz output band.
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

        // Envelope follower for the noise gate. Always tracks the LPF'd input
        // so the envelope is warm when recording starts — but the gain itself
        // is only applied below, on the recording-path write. The KWS feed
        // intentionally stays raw so quiet chews/swallows aren't suppressed.
        float abs_y = (yf >= 0.0f) ? yf : -yf;
        if (abs_y > g_gateEnv)
            g_gateEnv += GATE_ATTACK_A  * (abs_y - g_gateEnv);
        else
            g_gateEnv += GATE_RELEASE_A * (abs_y - g_gateEnv);

        int32_t ao = (xo < 0 ? -xo : xo);
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
                    // Recording path only: apply downward expansion using the
                    // envelope tracked above. Speech (env >= GATE_THRESH)
                    // passes unchanged; pauses get squashed.
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
                          (((int32_t)(s1 - s0) * (int32_t)g_playPhaseQ16) >> 16);

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
    uint32_t tBefore = xTaskGetTickCount();
    am_util_stdio_printf("[mic] waiting 3s before kws_init (tick=%lu)\r\n",
                         (unsigned long)tBefore);
    vTaskDelay(pdMS_TO_TICKS(3000));
    uint32_t tAfter = xTaskGetTickCount();
    am_util_stdio_printf("[mic] delay done (tick=%lu, dt=%lums)\r\n",
                         (unsigned long)tAfter,
                         (unsigned long)(tAfter - tBefore));

    am_util_stdio_printf("[mic] >kws_init\r\n");
    kws_init();
    am_util_stdio_printf("[mic] <kws_init\r\n");

    // python_speech_features.mfcc defaults: lowfreq=0, highfreq=samplerate/2.
    // We emit 23 raw coeffs and drop coefficient 0 downstream.
    g_mfccCfg.api              = &ns_mfcc_V1_0_0;
    g_mfccCfg.arena            = g_mfccArena;
    g_mfccCfg.sample_frequency = REC_PCM_RATE;
    g_mfccCfg.num_fbank_bins   = KWS_MFCC_FBANK_BINS;
    g_mfccCfg.low_freq         = 0;
    g_mfccCfg.high_freq        = REC_PCM_RATE / 2;
    g_mfccCfg.num_frames       = KWS_NUM_FRAMES;
    g_mfccCfg.num_coeffs       = KWS_MFCC_RAW_COEFFS;
    g_mfccCfg.num_dec_bits     = 0;
    g_mfccCfg.frame_shift_ms   = 10;
    g_mfccCfg.frame_len_ms     = 10;
    g_mfccCfg.frame_len        = KWS_FRAME_SAMPLES;
    g_mfccCfg.frame_len_pow2   = KWS_MFCC_FRAME_POW2;
    am_util_stdio_printf("[mic] >ns_mfcc_init\r\n");
    ns_mfcc_init(&g_mfccCfg);
    // Override ns_mfcc's defaults so MFCC matches the PSF training pipeline.
    kws_mfcc_post_init();
    am_util_stdio_printf("[mic] <ns_mfcc_init (psf-mode)\r\n");
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
                g_tonePos = 0;
                g_mode    = MODE_TESTTONE;
                am_util_stdio_printf("[mic] TEST TONE: 1kHz square wave, 2 sec...\n");
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
        // ---- KWS: compute one 10 ms MFCC frame when the ISR hands one over ----
        uint32_t readyBuf = g_kwsReadyBuf;
        if (readyBuf != 0xFF && g_mode == MODE_LISTENING)
        {
            // PSF-compatible MFCC: pre-emph + rect window + power filterbank
            // + sinusoidal lifter. Emits 23 raw coeffs; we drop coefficient 0
            // when copying into the feature block.
            kws_mfcc_compute_psf((const int16_t *)g_kwsBuf[readyBuf],
                                 g_mfccRawScratch);
            float *dst = &g_mfccFeatures[g_mfccFrameCount * KWS_MFCC_COEFFS];
            for (int c = 0; c < KWS_MFCC_COEFFS; c++) {
                dst[c] = g_mfccRawScratch[c + 1];
            }
            g_mfccFrameCount++;
            g_kwsReadyBuf = 0xFF;

            if (g_mfccFrameCount >= KWS_NUM_FRAMES)
            {
                g_mfccFrameCount = 0;
                kws_mfcc_reset_state();
                int32_t winPeak = g_kwsWindowPeak;
                g_kwsWindowPeak = 0;

                int   topIdx    = -1;
                float topConf   = 0.0f;
                int   top2Idx   = -1;
                float top2Conf  = 0.0f;

                if (winPeak >= KWS_PEAK_GATE)
                {
                    // Per-sample z-score across the full 250x22 feature block
                    // (matches the notebook's preprocessing pipeline).
                    const int N = KWS_NUM_FRAMES * KWS_MFCC_COEFFS;
                    double sum = 0.0;
                    for (int i = 0; i < N; i++) sum += g_mfccFeatures[i];
                    float mean = (float)(sum / (double)N);

                    double sqsum = 0.0;
                    for (int i = 0; i < N; i++) {
                        float d = g_mfccFeatures[i] - mean;
                        sqsum += (double)d * (double)d;
                    }
                    float stdv = (float)sqrt(sqsum / (double)N);
                    if (stdv < 1e-6f) stdv = 1e-6f;

                    for (int i = 0; i < N; i++) {
                        g_mfccFeatures[i] = (g_mfccFeatures[i] - mean) / stdv;
                    }

                    uint32_t inferT0 = xTaskGetTickCount();
                    kws_run_top2(g_mfccFeatures,
                                 &topIdx,  &topConf,
                                 &top2Idx, &top2Conf);
                    uint32_t inferDt = xTaskGetTickCount() - inferT0;
                    if (topIdx >= 0) {
                        am_util_stdio_printf("[kws] \"%s\" %.0f%% (2nd=\"%s\" %.0f%%, peak=%ld, infer=%lums)\n",
                                             kws_label_names[topIdx], topConf * 100.0f,
                                             (top2Idx >= 0 ? kws_label_names[top2Idx] : "?"),
                                             top2Conf * 100.0f,
                                             (long)winPeak,
                                             (unsigned long)inferDt);
                    }
                } else if (topIdx >= 0) {
                    am_util_stdio_printf("[kws] \"%s\" %.0f%% (2nd=\"%s\" %.0f%%, peak=%ld)\n",
                                         kws_label_names[topIdx], topConf * 100.0f,
                                         (top2Idx >= 0 ? kws_label_names[top2Idx] : "?"),
                                         top2Conf * 100.0f,
                                         (long)winPeak);
                }

                if (topIdx == KWS_TRIGGER_IDX && topConf >= KWS_THRESHOLD)
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
            kws_mfcc_reset_state();
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
            kws_mfcc_reset_state();
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
            kws_mfcc_reset_state();
#endif
            g_mode           = MIC_MODE_INITIAL;
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
