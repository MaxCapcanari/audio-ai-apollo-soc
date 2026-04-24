//*****************************************************************************
//
//! @file mic_task.c
//!
//! @brief Mic record/playback: AUDADC analog mic input, I2S0 TX output.
//!
//! Press BUTTON1 (SW2) -> record 5 seconds from analog mic via AUDADC.
//! Press again         -> play back through MAX98357A speaker via I2S0 TX.
//! Long press (2 s)    -> play 1 kHz test tone.
//!
//! Wiring (Apollo4p Blue KXR EVB):
//!   Analog electret mic OUT -> AUDADC SE0 analog input pin (dedicated, no GPIO config)
//!   Mic bias               <- AUDADC MIC_BIAS output (~1.8 V)
//!   MAX98357 DIN   -> GPIO 12  (I2S0_SDOUT)
//!   MAX98357 BCLK  -> GPIO 11  (I2S0_CLK)
//!   MAX98357 LRC   -> GPIO 13  (I2S0_WS)
//!
//! Sample rate: ~23.4 kHz
//!   AUDADC: HFRC 48 MHz / div32 / 64 = 23,437.5 Hz
//!   I2S0 TX: HFRC 1.5 MHz BCLK / 64 bits per stereo frame = 23,437.5 Hz
//! Bit depth: AUDADC 12-bit capture, I2S 24-bit playback
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

// I2S0 TX GPIO pins (speaker output only; no SDIN needed)
#define MIC_CLK_PIN         11
#define MIC_CLK_FUNC        AM_HAL_PIN_11_I2S0_CLK
#define MIC_WS_PIN          13
#define MIC_WS_FUNC         AM_HAL_PIN_13_I2S0_WS
#define MIC_SDOUT_PIN       12
#define MIC_SDOUT_FUNC      AM_HAL_PIN_12_I2S0_SDOUT

// I2S TX DMA: 256 words = 128 stereo frames per completion
#define MIC_DMA_SAMPLES     256

// AUDADC DMA words per completion.
// SAMPMODE_MED + 4 slots = 2 DMA words per scan (A-pair + B-pair).
// 256 words = 128 scans = 128 channel-A mono samples per completion.
#define AUDADC_DMA_WORDS    256
#define AUDADC_MONO_SAMPLES 128   // channel-A (SE0) samples extracted per completion

// BUTTON1 (SW2) = GPIO 19
#define BTN_PIN             19

// Playback left-shift: scales AC signal up to 24-bit range.
// AC amplitude ~145 counts -> << 15 brings it to ~4.7M / 8.4M (56% full scale).
#define PLAY_VOL_SHIFT      15

// AUDADC gain (dB)
#define PREAMP_GAIN_DB      24
#define CHANNEL_GAIN_DB     24

// Recording: 5 seconds at 23437 Hz
#define REC_SAMPLE_RATE     23438
#define REC_SECONDS         5
#define REC_SAMPLES         (REC_SAMPLE_RATE * REC_SECONDS)

// Per-second stats: 23438 Hz / 128 mono samples per completion ≈ 183 completions/sec
#define SEC_CHUNKS          183

//*****************************************************************************
// I2S TX DMA buffers — shared RAM, 16-byte aligned padding
//*****************************************************************************

AM_SHARED_RW static uint32_t g_txBufRawA[MIC_DMA_SAMPLES + 3];
AM_SHARED_RW static uint32_t g_txBufRawB[MIC_DMA_SAMPLES + 3];

//*****************************************************************************
// AUDADC DMA buffer — ping + pong contiguous, 16-byte aligned
//*****************************************************************************

AM_SHARED_RW static uint32_t g_audadcBufRaw[2 * AUDADC_DMA_WORDS + 3];

//*****************************************************************************
// Recording buffer — 5 seconds of 12-bit mono audio stored as int32_t
//*****************************************************************************

AM_SHARED_RW static int32_t g_recBuf[REC_SAMPLES];

//*****************************************************************************
// State
//*****************************************************************************

static void *g_pI2SHandle    = NULL;
static void *g_pAUDADCHandle = NULL;

typedef enum { MODE_IDLE, MODE_RECORDING, MODE_PLAYING, MODE_TESTTONE } mic_mode_t;

static volatile mic_mode_t g_mode     = MODE_IDLE;
static volatile uint32_t   g_recPos   = 0;
static volatile uint32_t   g_recLen   = 0;
static volatile int32_t    g_recDC    = 0;

static volatile bool       g_recDone  = false;
static volatile bool       g_playDone = false;

// Test tone: 1 kHz square wave, 2 seconds
#define TONE_FREQ           1000
#define TONE_HALF_PERIOD    (REC_SAMPLE_RATE / (TONE_FREQ * 2))
#define TONE_AMPLITUDE      0x200000
#define TONE_DURATION       (REC_SAMPLE_RATE * 2)
static volatile uint32_t   g_tonePos  = 0;
static volatile bool       g_toneDone = false;

// Per-second debug stats
static volatile uint32_t g_debugEven[4] = {0};
static volatile uint32_t g_debugOdd[4]  = {0};

static volatile uint32_t g_dmaBufCount  = 0;
static volatile int32_t  g_secSumAbs    = 0;
static volatile int32_t  g_secAvg       = 0;
static volatile int32_t  g_secPeak      = 0;
static volatile int32_t  g_secPeakLast  = 0;
static volatile uint32_t g_printReady   = 0;

// Diagnostic: total AUDADC ISR calls (0 = ISR never fired)
static volatile uint32_t g_audadcIsrCount = 0;

//*****************************************************************************
// I2S TX configuration (master, TX-only)
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
    .eXfer  = AM_HAL_I2S_XFER_TX,      // TX only — no RX/mic pin needed
    .eData  = &g_sDataConfig,
    .eIO    = &g_sIOConfig,
};

//*****************************************************************************
// AUDADC DMA configuration
//*****************************************************************************

static am_hal_audadc_dma_config_t g_sAUDADCDMAConfig =
{
    .bDynamicPriority = true,
    .ePriority        = AM_HAL_AUDADC_PRIOR_SERVICE_IMMED,
    .bDMAEnable       = true,
    .ui32SampleCount  = AUDADC_DMA_WORDS,
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

// Extract signed 12-bit audio sample from a 32-bit AUDADC DMA word.
// AUDADC packs the 12-bit code into bits[15:4]; treating it as int16_t gives
// a signed value centred around -32768 (offset-binary); DC is removed later.
static inline int32_t extract_audadc_sample(uint32_t raw)
{
    return (int32_t)((int16_t)(raw & 0xFFF0));
}

//*****************************************************************************
// ISR — AUDADC0 (recording path)
//*****************************************************************************

void am_audadc0_isr(void)
{
    uint32_t ui32IntMask;
    am_hal_audadc_interrupt_status(g_pAUDADCHandle, &ui32IntMask, false);
    am_hal_audadc_interrupt_clear(g_pAUDADCHandle, ui32IntMask);

    g_audadcIsrCount++;

    // DCMP fires when the DMA transfer count reaches the configured limit.
    // (FIFOOVR1 is a FIFO-overflow flag — it only fires on data loss, not on
    //  normal DMA completion, so it is the wrong trigger for this use case.)
    if (ui32IntMask & AM_HAL_AUDADC_INT_DCMP)
    {
        am_hal_audadc_interrupt_service(g_pAUDADCHandle, &g_sAUDADCDMAConfig);

        const uint32_t *rxBuf =
            (const uint32_t *)am_hal_audadc_dma_get_buffer(g_pAUDADCHandle);

        if (g_dmaBufCount == 0)
        {
            g_debugEven[0] = rxBuf[0]; g_debugEven[1] = rxBuf[2];
            g_debugEven[2] = rxBuf[4]; g_debugEven[3] = rxBuf[6];
            g_debugOdd[0]  = rxBuf[1]; g_debugOdd[1]  = rxBuf[3];
            g_debugOdd[2]  = rxBuf[5]; g_debugOdd[3]  = rxBuf[7];
        }

        // Each DMA word: bit19=0 → channel A (SE0 LG in bits[15:4])
        //                bit19=1 → channel B (SE2 LG in bits[15:4])
        // Alternate A,B,A,B... → extract only channel-A words (AUDADC_MONO_SAMPLES per completion)
        int32_t sumAbs = 0;
        for (uint32_t i = 0; i < AUDADC_DMA_WORDS; i++)
        {
            if ((rxBuf[i] >> 19) & 1) continue;   // skip channel B

            int32_t xo = extract_audadc_sample(rxBuf[i]);
            int32_t ao = (xo < 0 ? -xo : xo) >> 4;
            sumAbs += ao;
            if (ao > g_secPeak) g_secPeak = ao;

            if (g_mode == MODE_RECORDING && g_recPos < REC_SAMPLES)
            {
                g_recBuf[g_recPos++] = xo;
            }
        }

        if (g_mode == MODE_RECORDING && g_recPos >= REC_SAMPLES)
        {
            g_recLen  = g_recPos;
            g_mode    = MODE_IDLE;
            g_recDone = true;
        }

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
}

//*****************************************************************************
// ISR — I2S0 (playback / test-tone TX path)
//*****************************************************************************

void am_dspi2s0_isr(void)
{
    uint32_t ui32Status;
    am_hal_i2s_interrupt_status_get(g_pI2SHandle, &ui32Status, true);
    am_hal_i2s_interrupt_clear(g_pI2SHandle, ui32Status);
    am_hal_i2s_interrupt_service(g_pI2SHandle, ui32Status, &g_sI2SConfig);

    if (ui32Status & AM_HAL_I2S_INT_TXDMACPL)
    {
        uint32_t *txBuf =
            (uint32_t *)am_hal_i2s_dma_get_buffer(g_pI2SHandle, AM_HAL_I2S_XFER_TX);

        for (uint32_t i = 0; i < MIC_DMA_SAMPLES; i += 2)
        {
            if (g_mode == MODE_PLAYING && g_recPos < g_recLen)
            {
                // Scale AC signal to 24-bit range: shift left to fill 24-bit output.
                int32_t s = (int32_t)(g_recBuf[g_recPos++] - g_recDC);
                int32_t scaled = s << PLAY_VOL_SHIFT;
                // Clamp to 24-bit signed range to avoid wrapping artifacts.
                if (scaled >  0x7FFFFF) scaled =  0x7FFFFF;
                if (scaled < -0x800000) scaled = -0x800000;
                uint32_t tx = (uint32_t)(scaled) & 0x00FFFFFF;
                txBuf[i]     = tx;
                txBuf[i + 1] = tx;
            }
            else if (g_mode == MODE_TESTTONE && g_tonePos < TONE_DURATION)
            {
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

        if (g_mode == MODE_PLAYING && g_recPos >= g_recLen)
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
}

//*****************************************************************************
// Init helpers
//*****************************************************************************

static void mic_i2s_init(uint32_t txPtrA, uint32_t txPtrB)
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
    // Power up all 4 PGA channels + reference + mic bias — required for ADC core biasing.
    // mic bias (24 ≈ 1.8V) is needed even without an electret mic; it biases
    // internal AUDADC circuitry and without it the converter produces no output.
    am_hal_audadc_refgen_powerup();
    am_hal_audadc_pga_powerup(0);
    am_hal_audadc_pga_powerup(1);
    am_hal_audadc_pga_powerup(2);
    am_hal_audadc_pga_powerup(3);
    am_hal_audadc_gain_set(0, 2 * PREAMP_GAIN_DB);
    am_hal_audadc_gain_set(1, 2 * PREAMP_GAIN_DB);
    am_hal_audadc_gain_set(2, 2 * PREAMP_GAIN_DB);
    am_hal_audadc_gain_set(3, 2 * PREAMP_GAIN_DB);
    am_hal_audadc_micbias_powerup(24);
    am_util_delay_ms(400);   // allow bias/refgen to settle (per Ambiq example)

    am_hal_audadc_config_t audadcCfg =
    {
        .eClock         = AM_HAL_AUDADC_CLKSEL_HFRC_48MHz,
        .ePolarity      = AM_HAL_AUDADC_TRIGPOL_RISING,
        .eTrigger       = AM_HAL_AUDADC_TRIGSEL_SOFTWARE,
        .eClockMode     = AM_HAL_AUDADC_CLKMODE_LOW_LATENCY,
        .ePowerMode     = AM_HAL_AUDADC_LPMODE0,
        .eRepeat        = AM_HAL_AUDADC_REPEATING_SCAN,
        .eRepeatTrigger = AM_HAL_AUDADC_RPTTRIGSEL_INT,
        .eSampMode      = AM_HAL_AUDADC_SAMPMODE_MED,  // 2 LG + 2 HG channels
    };

    // IRTT: HFRC 48 MHz / div32 / (63+1) = 23,437.5 Hz — matches I2S TX rate
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

    // Configure all 4 slots (SE0-SE3) to match the standard board config.
    // Audio from J18 arrives on channel A (SE0 = LG, SE1 = HG).
    // The ISR extracts only SE0 (channel A, LG) for mono recording.
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

    // Internal PGA gain for all channel pairs
    g_sAudadcGainConfig.ui32LGA      = (uint32_t)(CHANNEL_GAIN_DB * 2 + 12);
    g_sAudadcGainConfig.ui32HGADELTA = 0;
    g_sAudadcGainConfig.ui32LGB      = (uint32_t)(CHANNEL_GAIN_DB * 2 + 12);
    g_sAudadcGainConfig.ui32HGBDELTA = 0;
    g_sAudadcGainConfig.eUpdateMode  = AM_HAL_AUDADC_GAIN_UPDATE_IMME;
    am_hal_audadc_internal_pga_config(g_pAUDADCHandle, &g_sAudadcGainConfig);

    // DMA ping-pong buffers
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

    // Align TX buffers to 16 bytes
    uint32_t txPtrA = (uint32_t)((uint32_t)(g_txBufRawA + 3) & ~0xF);
    uint32_t txPtrB = (uint32_t)((uint32_t)(g_txBufRawB + 3) & ~0xF);
    memset((void *)txPtrA, 0, MIC_DMA_SAMPLES * sizeof(uint32_t));
    memset((void *)txPtrB, 0, MIC_DMA_SAMPLES * sizeof(uint32_t));

    // Align AUDADC ping-pong buffers to 16 bytes
    uint32_t audadcBase = (uint32_t)((uint32_t)(g_audadcBufRaw + 3) & ~0xF);
    uint32_t dmaPtrA    = audadcBase;
    uint32_t dmaPtrB    = audadcBase + AUDADC_DMA_WORDS * sizeof(uint32_t);

    // BUTTON1 (SW2, GPIO 19): input with 100K pull-up, active low
    am_hal_gpio_pincfg_t btnCfg = {0};
    btnCfg.GP.cfg_b.eGPOutCfg = 0;
    btnCfg.GP.cfg_b.eGPInput  = 1;
    btnCfg.GP.cfg_b.ePullup   = 6;
    btnCfg.GP.cfg_b.uFuncSel  = 3;
    am_hal_gpio_pinconfig(BTN_PIN, btnCfg);

    mic_audadc_init(dmaPtrA, dmaPtrB);
    mic_i2s_init(txPtrA, txPtrB);

    am_hal_interrupt_master_enable();

    // Start AUDADC (IRTT takes over after first trigger)
    am_hal_audadc_sw_trigger(g_pAUDADCHandle);

    // Start I2S TX DMA (runs continuously, outputs silence until playback)
    am_hal_i2s_dma_transfer_start(g_pI2SHandle, &g_sI2SConfig);

    am_util_stdio_printf("[mic] Ready. Short press=record/play, long press(2s)=test tone.\n");

    TickType_t xLastWake  = xTaskGetTickCount();
    bool     btnPrev     = false;
    uint32_t btnDead     = 0;
    uint32_t btnHoldCnt  = 0;
    uint32_t diagTicks   = 0;   // for periodic ISR-count diagnostic

    while (1)
    {
        vTaskDelayUntil(&xLastWake, 10);

        // ---- Button polling ----
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
            if (btnHoldCnt == 200 && g_mode == MODE_IDLE)
            {
                g_tonePos = 0;
                g_mode    = MODE_TESTTONE;
                am_util_stdio_printf("[mic] TEST TONE: 1kHz square wave, 2 sec...\n");
                btnDead = 5;
            }
        }
        else if (!btnDown && btnPrev)
        {
            if (btnHoldCnt < 200 && g_mode == MODE_IDLE)
            {
                btnDead = 5;
                if (g_recLen > 0)
                {
                    g_recPos = 0;
                    g_mode   = MODE_PLAYING;
                    am_util_stdio_printf("[mic] Playing %lu samples...\n",
                                         (unsigned long)g_recLen);
                }
                else
                {
                    g_recPos = 0;
                    g_mode   = MODE_RECORDING;
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

            int64_t sum = 0;
            int32_t bMin = g_recBuf[0], bMax = g_recBuf[0];
            for (uint32_t k = 0; k < g_recLen; k++)
            {
                int32_t v = g_recBuf[k];
                sum += v;
                if (v < bMin) bMin = v;
                if (v > bMax) bMax = v;
            }
            int32_t bMean = (int32_t)(sum / (int64_t)g_recLen);
            g_recDC = bMean;

            // RMS on right-shifted values to avoid int64 overflow
            int64_t sqSum = 0;
            for (uint32_t k = 0; k < g_recLen; k++)
            {
                int32_t v = (g_recBuf[k] - bMean) >> 4;
                sqSum += (int64_t)v * v;
            }
            uint32_t rms4 = 0;
            {
                int64_t avg = sqSum / (int64_t)g_recLen;
                uint32_t r = 0, bit = 1u << 15;
                while (bit)
                {
                    uint32_t t = r | bit;
                    if ((int64_t)t * t <= avg) r = t;
                    bit >>= 1;
                }
                rms4 = r;
            }

            am_util_stdio_printf("[mic] BUFFER ANALYSIS:\n");
            am_util_stdio_printf("  samples=%lu  mean=%ld\n",
                                 (unsigned long)g_recLen, (long)bMean);
            am_util_stdio_printf("  min=%ld  max=%ld\n", (long)bMin, (long)bMax);
            am_util_stdio_printf("  rms(ac,>>4)=%lu\n", (unsigned long)rms4);
            am_util_stdio_printf("[mic] If rms is high -> noise is in RECORDING (mic).\n");
            am_util_stdio_printf("[mic] Press short=play, hold 2s=test tone.\n");

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

        // ---- Diagnostic: print AUDADC ISR call count every 3 seconds ----
        diagTicks++;
        if (diagTicks >= 300)   // 300 * 10ms = 3 sec
        {
            diagTicks = 0;
            am_util_stdio_printf("[mic] audadc_isr_count=%lu recPos=%lu\n",
                                 (unsigned long)g_audadcIsrCount,
                                 (unsigned long)g_recPos);
        }

        // ---- Per-second stats ----
        if (g_printReady)
        {
            g_printReady = 0;
            const char *mstr = (g_mode == MODE_RECORDING) ? "REC" :
                               (g_mode == MODE_PLAYING)   ? "PLAY" :
                               (g_mode == MODE_TESTTONE)  ? "TONE" : "IDLE";
            am_util_stdio_printf("[mic] avg=%ld peak=%ld mode=%s dma0=%08X %08X %08X %08X\n",
                                 (long)g_secAvg, (long)g_secPeakLast, mstr,
                                 g_debugEven[0], g_debugEven[1],
                                 g_debugEven[2], g_debugEven[3]);
        }
    }
}
