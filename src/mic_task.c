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

// Volume right-shift for playback.  0 = full, 1 = -6 dB, 2 = -12 dB.
#define PLAY_VOL_SHIFT      1

// Recording length
#define REC_SAMPLE_RATE     23438
#define REC_SECONDS         5
#define REC_SAMPLES         (REC_SAMPLE_RATE * REC_SECONDS)   // 46876 mono samples

// Per-second stats reporting
#define SEC_CHUNKS          183

//*****************************************************************************
// DMA buffers — must be in shared RAM and 16-byte aligned
//*****************************************************************************

AM_SHARED_RW static uint32_t g_rxBufRawA[2 * MIC_DMA_SAMPLES + 3];
AM_SHARED_RW static uint32_t g_rxBufRawB[2 * MIC_DMA_SAMPLES + 3];
AM_SHARED_RW static uint32_t g_txBufRawA[2 * MIC_DMA_SAMPLES + 3];
AM_SHARED_RW static uint32_t g_txBufRawB[2 * MIC_DMA_SAMPLES + 3];

//*****************************************************************************
// Recording buffer — ~183 KB for 2 seconds of mono 24-bit audio
//*****************************************************************************

AM_SHARED_RW static int32_t g_recBuf[REC_SAMPLES];

//*****************************************************************************
// State
//*****************************************************************************

static void *g_pI2SHandle = NULL;

typedef enum { MODE_IDLE, MODE_RECORDING, MODE_PLAYING } mic_mode_t;

static volatile mic_mode_t g_mode     = MODE_IDLE;
static volatile uint32_t   g_recPos   = 0;
static volatile uint32_t   g_recLen   = 0;
static volatile bool       g_nextPlay = false;

// ISR -> task notification
static volatile bool       g_recDone  = false;
static volatile bool       g_playDone = false;

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

            // ---- Record: store mono sample ----
            if (g_mode == MODE_RECORDING && g_recPos < REC_SAMPLES)
            {
                g_recBuf[g_recPos++] = xo;
            }

            // ---- TX output ----
            if (g_mode == MODE_PLAYING && g_recPos < g_recLen)
            {
                int32_t s = g_recBuf[g_recPos++];
                uint32_t tx = (uint32_t)((int32_t)(s >> PLAY_VOL_SHIFT)) & 0x00FFFFFF;
                txBuf[i]     = tx;
                txBuf[i + 1] = tx;
            }
            else
            {
                txBuf[i]     = 0;
                txBuf[i + 1] = 0;
            }
        }

        // Auto-stop recording when buffer is full
        if (g_mode == MODE_RECORDING && g_recPos >= REC_SAMPLES)
        {
            g_recLen  = g_recPos;
            g_mode    = MODE_IDLE;
            g_recDone = true;
        }

        // Auto-stop playback when done
        if (g_mode == MODE_PLAYING && g_recPos >= g_recLen)
        {
            g_mode     = MODE_IDLE;
            g_playDone = true;
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
    else if (ui32Status & AM_HAL_I2S_INT_TXDMACPL)
    {
        uint32_t *txBuf =
            (uint32_t *)am_hal_i2s_dma_get_buffer(g_pI2SHandle, AM_HAL_I2S_XFER_TX);
        for (uint32_t i = 0; i < MIC_DMA_SAMPLES; i++)
            txBuf[i] = 0;
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

    am_util_stdio_printf("[mic] Ready. Press BUTTON1 (SW2) to record %d sec.\n", REC_SECONDS);

    TickType_t xLastWake = xTaskGetTickCount();
    bool     btnPrev = false;
    uint32_t btnDead = 0;

    while (1)
    {
        vTaskDelayUntil(&xLastWake, 10);

        // ---- Button polling with 50 ms debounce ----
        uint32_t btnVal;
        am_hal_gpio_state_read(BTN_PIN, AM_HAL_GPIO_INPUT_READ, &btnVal);
        bool btnDown = (btnVal == 0);

        if (btnDead > 0)
        {
            btnDead--;
        }
        else if (btnDown && !btnPrev)
        {
            btnDead = 5;

            if (g_mode == MODE_IDLE)
            {
                if (g_recLen > 0)
                {
                    // Already have a recording — play it again
                    g_recPos = 0;
                    g_mode   = MODE_PLAYING;
                    am_util_stdio_printf("[mic] Playing %lu samples...\n",
                                         (unsigned long)g_recLen);
                }
                else
                {
                    // No recording yet — record once
                    g_recPos = 0;
                    g_mode   = MODE_RECORDING;
                    am_util_stdio_printf("[mic] Recording %d sec...\n", REC_SECONDS);
                }
            }
        }
        btnPrev = btnDown;

        // ---- ISR notifications ----
        if (g_recDone)
        {
            g_recDone = false;
            am_util_stdio_printf("[mic] Recording done (%lu samples). Press to play.\n",
                                 (unsigned long)g_recLen);

            // Dump 32 consecutive samples from 0.5s into recording to inspect waveform.
            uint32_t dumpStart = REC_SAMPLE_RATE / 2;
            if (dumpStart + 32 <= g_recLen)
            {
                am_util_stdio_printf("[mic] Samples @%lu:\n", (unsigned long)dumpStart);
                for (int d = 0; d < 32; d++)
                {
                    am_util_stdio_printf("  [%d] %ld\n", d, (long)g_recBuf[dumpStart + d]);
                }
            }
        }
        if (g_playDone)
        {
            g_playDone = false;
            am_util_stdio_printf("[mic] Playback done. Press to record.\n");
        }

        // ---- Per-second stats ----
        if (g_printReady)
        {
            g_printReady = 0;
            const char *mstr = (g_mode == MODE_RECORDING) ? "REC" :
                               (g_mode == MODE_PLAYING)   ? "PLAY" : "IDLE";
            am_util_stdio_printf("[mic] avg=%ld peak=%ld mode=%s even=%08X %08X odd=%08X %08X\n",
                                 (long)g_secAvg, (long)g_secPeakLast, mstr,
                                 g_debugEven[0], g_debugEven[1],
                                 g_debugOdd[0],  g_debugOdd[1]);
        }
    }
}
