//*****************************************************************************
//
//! @file mic_task.c
//!
//! @brief Mic record/playback via I2S0, controlled by BUTTON0 (SW1).
//!
//! Press BUTTON0 -> record 2 seconds of audio from INMP441 mic.
//! Press again   -> play it back through MAX98357A speaker.
//! Alternates record / play on each press.
//!
//! Wiring (Apollo4p Blue KXR EVB):
//!   INMP441 SD    -> GPIO 14  (I2S0_SDIN)
//!   INMP441 SCK   -> GPIO 11  (I2S0_CLK)
//!   INMP441 WS    -> GPIO 13  (I2S0_WS)   L/R pin = GND
//!   MAX98357 DIN  -> GPIO 12  (I2S0_SDOUT)
//!   MAX98357 BCLK -> GPIO 11  (shared)
//!   MAX98357 LRC  -> GPIO 13  (shared)
//!
//! Sample rate: ~23.4 kHz  (HFRC 1.5 MHz BCLK / 64 bits per stereo frame)
//! Bit depth:   24-bit signed PCM (right-justified in 32-bit DMA word)
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
#include "ae_api.h"     // Ambiq Opus encoder (audio_enc_init / audio_enc_encode_frame)

//*****************************************************************************
// Configuration
//*****************************************************************************

#define MIC_I2S_MODULE      0
#define MIC_DMA_SAMPLES     256     // words per DMA buffer (128 stereo frames)

// I2S0 GPIO pin assignments
#define MIC_CLK_PIN         11
#define MIC_CLK_FUNC        AM_HAL_PIN_11_I2S0_CLK
#define MIC_WS_PIN          13
#define MIC_WS_FUNC         AM_HAL_PIN_13_I2S0_WS
#define MIC_SDIN_PIN        14
#define MIC_SDIN_FUNC       AM_HAL_PIN_14_I2S0_SDIN
#define MIC_SDOUT_PIN       12
#define MIC_SDOUT_FUNC      AM_HAL_PIN_12_I2S0_SDOUT

// BUTTON1 (SW2) = GPIO 19 — free since I2S0 uses GPIOs 11-14.
// BUTTON0 (SW1, GPIO 17) is reserved for BLE messages.
#define BTN_PIN             19

// Playback gain multiplier.  1 = unity, 2 = +6 dB, 4 = +12 dB, 8 = +18 dB.
// Values above 1 will be soft-clipped to 24-bit range.
#define PLAY_GAIN           8

// Native I2S sample rate (set by HFRC clock / 64 bits per stereo frame)
#define MIC_NATIVE_RATE     23438

// PCM storage rate — Opus encoder requires 16 kHz mono input
#define REC_PCM_RATE        16000
#define REC_SECONDS         10
#define REC_PCM_SAMPLES     (REC_PCM_RATE * REC_SECONDS)      // 480000 mono int16

// Opus encoder framing (fixed by SDK library: 20 ms @ 16 kHz, CBR 32 kbit/s)
#define OPUS_FRAME_SAMPLES  320                               // 20 ms @ 16 kHz
#define OPUS_FRAME_BYTES    80                                // 32 kbit/s * 20 ms / 8
#define OPUS_NUM_FRAMES     (REC_SECONDS * 50)                // 1500 frames in 30 s
#define OPUS_BUF_BYTES      (OPUS_NUM_FRAMES * OPUS_FRAME_BYTES) // 120000 bytes

// Resampler step in 16.16 Q-format: how many input samples to consume
// per output sample.
//   capture:  out=16000, in=23438  -> step = 23438/16000 ≈ 1.4649
//   playback: out=23438, in=16000  -> step = 16000/23438 ≈ 0.6826
#define CAP_STEP_Q16  (uint32_t)(((uint64_t)MIC_NATIVE_RATE << 16) / REC_PCM_RATE)
#define PLAY_STEP_Q16 (uint32_t)(((uint64_t)REC_PCM_RATE   << 16) / MIC_NATIVE_RATE)

// Per-second stats reporting (in DMA buffers; one DMA buf = 128 stereo frames
// at 23438 Hz ≈ 5.46 ms, so ~183 buffers per second)
#define SEC_CHUNKS          183

//*****************************************************************************
// DMA buffers — must be in shared RAM and 16-byte aligned
//*****************************************************************************

AM_SHARED_RW static uint32_t g_rxBufRawA[2 * MIC_DMA_SAMPLES + 3];
AM_SHARED_RW static uint32_t g_rxBufRawB[2 * MIC_DMA_SAMPLES + 3];
AM_SHARED_RW static uint32_t g_txBufRawA[2 * MIC_DMA_SAMPLES + 3];
AM_SHARED_RW static uint32_t g_txBufRawB[2 * MIC_DMA_SAMPLES + 3];

//*****************************************************************************
// Recording buffers (in shared SRAM)
//   g_pcmBuf  — 16 kHz int16 mono PCM, used for on-device playback
//   g_opusBuf — Opus 32 kbit/s CBR, ready for BLE transmission
//*****************************************************************************

AM_SHARED_RW static int16_t g_pcmBuf[REC_PCM_SAMPLES];   // 960 KB
AM_SHARED_RW static uint8_t g_opusBuf[OPUS_BUF_BYTES];   // 120 KB

// Encoded Opus length (0 until a recording has been encoded).
static volatile uint32_t g_opusLen = 0;

//*****************************************************************************
// State
//*****************************************************************************

static void *g_pI2SHandle = NULL;

typedef enum { MODE_IDLE, MODE_RECORDING, MODE_PLAYING, MODE_TESTTONE } mic_mode_t;

static volatile mic_mode_t g_mode     = MODE_IDLE;
static volatile uint32_t   g_recPos   = 0;
static volatile uint32_t   g_recLen   = 0;
static volatile bool       g_nextPlay = false;
static volatile int32_t    g_recDC    = 0;     // DC offset to subtract during playback

// ISR -> task notification
static volatile bool       g_recDone  = false;
static volatile bool       g_playDone = false;

// Test tone state (1 kHz square wave, 2 seconds at native I2S rate)
#define TONE_FREQ           1000
#define TONE_HALF_PERIOD    (MIC_NATIVE_RATE / (TONE_FREQ * 2))  // ~11 samples
#define TONE_AMPLITUDE      0x200000   // ~25% of 24-bit full scale
#define TONE_DURATION       (MIC_NATIVE_RATE * 2)  // 2 seconds
static volatile uint32_t   g_tonePos  = 0;
static volatile bool       g_toneDone = false;

// Resampler state
//   Capture (record): 23438 Hz int24 -> 16000 Hz int16
//     Each input sample adds 65536 to g_capAccum.  When g_capAccum reaches
//     CAP_STEP_Q16 (~96000), emit one output sample and subtract.
//   Playback: 16000 Hz int16 -> 23438 Hz int24
//     Each output sample advances g_playPhaseQ16 by PLAY_STEP_Q16 (~44740).
//     When phase wraps past 65536, advance the source index.
static volatile int32_t   g_capLast       = 0;     // last input sample (24-bit signed)
static volatile uint32_t  g_capAccum      = 0;     // Q16 accumulator
static volatile uint32_t  g_playPhaseQ16  = 0;     // 0..65535 fractional position
static volatile uint32_t  g_playSrcIdx    = 0;     // index into g_pcmBuf during playback

// Raw DMA debug: even + odd channel words from start of each second
static volatile uint32_t g_debugEven[4] = {0};
static volatile uint32_t g_debugOdd[4]  = {0};

// Per-second debug stats
static volatile uint32_t g_dmaBufCount  = 0;
static volatile int32_t  g_secSumAbs    = 0;
static volatile int32_t  g_secAvg       = 0;
static volatile int32_t  g_secPeak      = 0;
static volatile int32_t  g_secPeakLast  = 0;
static volatile uint32_t g_printReady   = 0;
static bool              g_micDebug     = false;  // set true to enable per-second stats

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
    .eXfer  = AM_HAL_I2S_XFER_RXTX,
    .eData  = &g_sDataConfig,
    .eIO    = &g_sIOConfig,
};

//*****************************************************************************
// Helpers
//*****************************************************************************

static inline int32_t extract_sample(uint32_t raw)
{
    return (int32_t)(raw << 8) >> 8;
}


//*****************************************************************************
// ISR — I2S0
//*****************************************************************************

void am_dspi2s0_isr(void)
{
    uint32_t ui32Status;
    am_hal_i2s_interrupt_status_get(g_pI2SHandle, &ui32Status, true);
    am_hal_i2s_interrupt_clear(g_pI2SHandle, ui32Status);
    am_hal_i2s_interrupt_service(g_pI2SHandle, ui32Status, &g_sI2SConfig);

    if (ui32Status & AM_HAL_I2S_INT_RXDMACPL)
    {
        const uint32_t *rxBuf =
            (const uint32_t *)am_hal_i2s_dma_get_buffer(g_pI2SHandle, AM_HAL_I2S_XFER_RX);
        uint32_t *txBuf =
            (uint32_t *)am_hal_i2s_dma_get_buffer(g_pI2SHandle, AM_HAL_I2S_XFER_TX);

        // Capture raw DMA words at start of each second for format diagnostics
        if (g_dmaBufCount == 0)
        {
            g_debugEven[0] = rxBuf[0]; g_debugEven[1] = rxBuf[2];
            g_debugEven[2] = rxBuf[4]; g_debugEven[3] = rxBuf[6];
            g_debugOdd[0]  = rxBuf[1]; g_debugOdd[1]  = rxBuf[3];
            g_debugOdd[2]  = rxBuf[5]; g_debugOdd[3]  = rxBuf[7];
        }

        int32_t sumOdd = 0;
        for (uint32_t i = 0; i < MIC_DMA_SAMPLES; i += 2)
        {
            int32_t xo = extract_sample(rxBuf[i + 1]);   // odd = right/WS-LOW
            int32_t ao = (xo < 0 ? -xo : xo) >> 8;
            sumOdd += ao;
            if (ao > g_secPeak) g_secPeak = ao;

            // ---- Record: decimate 23438 Hz -> 16000 Hz, store as int16 ----
            if (g_mode == MODE_RECORDING)
            {
                g_capAccum += 65536u;
                if (g_capAccum >= CAP_STEP_Q16)
                {
                    g_capAccum -= CAP_STEP_Q16;
                    if (g_recPos < REC_PCM_SAMPLES)
                    {
                        // Linear interpolation between previous and current input
                        int32_t frac = (int32_t)g_capAccum;     // 0..CAP_STEP_Q16-1
                        int32_t diff = xo - g_capLast;
                        int32_t s24  = g_capLast +
                                       (int32_t)(((int64_t)diff * frac) >> 16);
                        // 24-bit signed -> 16-bit signed (drop LSBs)
                        int32_t s16  = s24 >> 8;
                        if (s16 >  32767) s16 =  32767;
                        if (s16 < -32768) s16 = -32768;
                        g_pcmBuf[g_recPos++] = (int16_t)s16;
                    }
                }
            }
            g_capLast = xo;

            // ---- TX output ----
            if (g_mode == MODE_PLAYING && g_playSrcIdx < g_recLen)
            {
                // Interpolate 16 kHz int16 PCM up to 23438 Hz int24
                int16_t s0 = g_pcmBuf[g_playSrcIdx];
                int16_t s1 = (g_playSrcIdx + 1 < g_recLen)
                             ? g_pcmBuf[g_playSrcIdx + 1] : s0;
                int32_t out = (int32_t)s0 +
                              (((int32_t)(s1 - s0) * (int32_t)g_playPhaseQ16) >> 16);

                // 16-bit -> 24-bit with gain, clamp to 24-bit range
                int32_t s24 = out * PLAY_GAIN;
                if (s24 >  32767) s24 =  32767;
                if (s24 < -32768) s24 = -32768;
                s24 <<= 8;
                uint32_t tx = (uint32_t)s24 & 0x00FFFFFF;
                txBuf[i]     = tx;
                txBuf[i + 1] = tx;

                // Advance phase
                g_playPhaseQ16 += PLAY_STEP_Q16;
                while (g_playPhaseQ16 >= 65536u)
                {
                    g_playPhaseQ16 -= 65536u;
                    g_playSrcIdx++;
                }
            }
            else if (g_mode == MODE_TESTTONE && g_tonePos < TONE_DURATION)
            {
                // 1 kHz square wave: positive half then negative half
                int32_t val = ((g_tonePos / TONE_HALF_PERIOD) & 1)
                              ? -TONE_AMPLITUDE : TONE_AMPLITUDE;
                uint32_t tx = (uint32_t)(val) & 0x00FFFFFF;
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

        // Auto-stop recording when buffer is full
        if (g_mode == MODE_RECORDING && g_recPos >= REC_PCM_SAMPLES)
        {
            g_recLen  = g_recPos;
            g_mode    = MODE_IDLE;
            g_recDone = true;
        }

        // Auto-stop playback when done
        if (g_mode == MODE_PLAYING && g_playSrcIdx >= g_recLen)
        {
            g_mode     = MODE_IDLE;
            g_playDone = true;
        }

        // Auto-stop test tone when done
        if (g_mode == MODE_TESTTONE && g_tonePos >= TONE_DURATION)
        {
            g_mode     = MODE_IDLE;
            g_toneDone = true;
        }

        // Per-second stats
        g_secSumAbs += sumOdd;
        g_dmaBufCount++;
        if (g_dmaBufCount >= SEC_CHUNKS)
        {
            g_secAvg      = g_secSumAbs / (SEC_CHUNKS * (MIC_DMA_SAMPLES / 2));
            g_secPeakLast = g_secPeak;
            g_secSumAbs   = 0;
            g_secPeak     = 0;
            g_dmaBufCount = 0;
            g_printReady  = 1;
        }
    }
}

//*****************************************************************************
// Init
//*****************************************************************************

static void mic_i2s_init(uint32_t rxPtrA, uint32_t rxPtrB,
                          uint32_t txPtrA, uint32_t txPtrB)
{
    // Output pins (CLK, WS, SDOUT): push-pull
    am_hal_gpio_pincfg_t outCfg = { .GP.cfg_b.eGPOutCfg = 1, .GP.cfg_b.ePullup = 0 };

    outCfg.GP.cfg_b.uFuncSel = MIC_CLK_FUNC;
    am_hal_gpio_pinconfig(MIC_CLK_PIN, outCfg);

    outCfg.GP.cfg_b.uFuncSel = MIC_WS_FUNC;
    am_hal_gpio_pinconfig(MIC_WS_PIN, outCfg);

    outCfg.GP.cfg_b.uFuncSel = MIC_SDOUT_FUNC;
    am_hal_gpio_pinconfig(MIC_SDOUT_PIN, outCfg);

    // Input pin (SDIN)
    am_hal_gpio_pincfg_t inCfg = { .GP.cfg_b.eGPOutCfg = 0, .GP.cfg_b.eGPInput = 1, .GP.cfg_b.ePullup = 0 };
    inCfg.GP.cfg_b.uFuncSel = MIC_SDIN_FUNC;
    am_hal_gpio_pinconfig(MIC_SDIN_PIN, inCfg);

    am_hal_i2s_initialize(MIC_I2S_MODULE, &g_pI2SHandle);
    am_hal_i2s_power_control(g_pI2SHandle, AM_HAL_I2S_POWER_ON, false);
    am_hal_i2s_configure(g_pI2SHandle, &g_sI2SConfig);
    am_hal_i2s_enable(g_pI2SHandle);

    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFRC2_START, false);
    am_util_delay_us(500);

    g_sTransfer.ui32RxTotalCount        = MIC_DMA_SAMPLES;
    g_sTransfer.ui32RxTargetAddr        = rxPtrA;
    g_sTransfer.ui32RxTargetAddrReverse = rxPtrB;
    g_sTransfer.ui32TxTotalCount        = MIC_DMA_SAMPLES;
    g_sTransfer.ui32TxTargetAddr        = txPtrA;
    g_sTransfer.ui32TxTargetAddrReverse = txPtrB;

    am_hal_i2s_dma_configure(g_pI2SHandle, &g_sI2SConfig, &g_sTransfer);

    NVIC_SetPriority(I2S0_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(I2S0_IRQn);
    am_hal_interrupt_master_enable();
}

//*****************************************************************************
// FreeRTOS task
//*****************************************************************************

void MicTask(void *pvParameters)
{
    (void)pvParameters;

    uint32_t rxPtrA = (uint32_t)((uint32_t)(g_rxBufRawA + 3) & ~0xF);
    uint32_t rxPtrB = (uint32_t)((uint32_t)(g_rxBufRawB + 3) & ~0xF);
    uint32_t txPtrA = (uint32_t)((uint32_t)(g_txBufRawA + 3) & ~0xF);
    uint32_t txPtrB = (uint32_t)((uint32_t)(g_txBufRawB + 3) & ~0xF);

    memset((void *)txPtrA, 0, MIC_DMA_SAMPLES * sizeof(uint32_t));
    memset((void *)txPtrB, 0, MIC_DMA_SAMPLES * sizeof(uint32_t));

    // BUTTON1 (SW2, GPIO 19): input with 100K pull-up, active low
    am_hal_gpio_pincfg_t btnCfg = {0};
    btnCfg.GP.cfg_b.eGPOutCfg = 0;
    btnCfg.GP.cfg_b.eGPInput  = 1;
    btnCfg.GP.cfg_b.ePullup   = 6;   // AM_HAL_GPIO_PIN_PULLUP_100K
    btnCfg.GP.cfg_b.uFuncSel  = 3;   // GPIO
    am_hal_gpio_pinconfig(BTN_PIN, btnCfg);

    mic_i2s_init(rxPtrA, rxPtrB, txPtrA, txPtrB);
    am_hal_i2s_dma_transfer_start(g_pI2SHandle, &g_sI2SConfig);

    am_util_stdio_printf("[mic] Ready. Short press=record/play, long press(2s)=test tone.\n");

    TickType_t xLastWake = xTaskGetTickCount();
    bool     btnPrev    = false;
    uint32_t btnDead    = 0;
    uint32_t btnHoldCnt = 0;   // counts 10ms ticks while held

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
            // Button just pressed — start counting
            btnHoldCnt = 0;
        }
        else if (btnDown && btnPrev)
        {
            // Button held
            btnHoldCnt++;
            if (btnHoldCnt == 200 && g_mode == MODE_IDLE) // 200 * 10ms = 2 sec
            {
                // Long press -> test tone
                g_tonePos = 0;
                g_mode    = MODE_TESTTONE;
                am_util_stdio_printf("[mic] TEST TONE: 1kHz square wave, 2 sec...\n");
                btnDead = 5;
            }
        }
        else if (!btnDown && btnPrev)
        {
            // Button released
            if (btnHoldCnt < 200 && g_mode == MODE_IDLE)
            {
                // Short press -> record/play
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
                    g_capLast   = 0;
                    g_opusLen   = 0;
                    g_mode      = MODE_RECORDING;
                    am_util_stdio_printf("[mic] Recording %d sec...\n", REC_SECONDS);
                }
            }
            btnHoldCnt = 0;
        }
        btnPrev = btnDown;

        // ---- ISR notifications ----
        if (g_recDone)
        {
            g_recDone = false;

            // ---- Buffer analysis on the 16 kHz int16 PCM ----
            int64_t sum = 0;
            int16_t bMin = g_pcmBuf[0], bMax = g_pcmBuf[0];
            for (uint32_t k = 0; k < g_recLen; k++)
            {
                int16_t v = g_pcmBuf[k];
                sum += v;
                if (v < bMin) bMin = v;
                if (v > bMax) bMax = v;
            }
            int16_t bMean = (int16_t)(sum / (int64_t)g_recLen);

            // Remove DC in-place so playback (and the Opus encoder) sees a
            // zero-mean signal.
            for (uint32_t k = 0; k < g_recLen; k++)
            {
                int32_t v = (int32_t)g_pcmBuf[k] - bMean;
                if (v >  32767) v =  32767;
                if (v < -32768) v = -32768;
                g_pcmBuf[k] = (int16_t)v;
            }

            am_util_stdio_printf("[mic] PCM @16kHz: %lu samples, mean=%d, min=%d max=%d\n",
                                 (unsigned long)g_recLen,
                                 (int)bMean, (int)bMin, (int)bMax);

            // ---- Encode to Opus (32 kbit/s CBR, 20 ms frames, 16 kHz mono) ----
            uint32_t totalFrames = g_recLen / OPUS_FRAME_SAMPLES;
            if (totalFrames > OPUS_NUM_FRAMES) totalFrames = OPUS_NUM_FRAMES;

            am_util_stdio_printf("[mic] Encoding %lu Opus frames...\n",
                                 (unsigned long)totalFrames);
            audio_enc_init(0);   // 0 = no per-frame header

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
        }
        if (g_toneDone)
        {
            g_toneDone = false;
            am_util_stdio_printf("[mic] Test tone done. If clean -> TX path OK, noise is from mic.\n");
        }
        if (g_playDone)
        {
            g_playDone = false;
            am_util_stdio_printf("[mic] Playback done.\n");
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
                am_util_stdio_printf("[mic] avg=%ld peak=%ld mode=%s even=%08X %08X odd=%08X %08X\n",
                                     (long)g_secAvg, (long)g_secPeakLast, mstr,
                                     g_debugEven[0], g_debugEven[1],
                                     g_debugOdd[0],  g_debugOdd[1]);
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
