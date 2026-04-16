//*****************************************************************************
//
//! @file mic_task.c
//!
//! @brief Analog MEMS mic record/playback on Apollo4 Blue Plus KXR EVB.
//!
//! Record : Analog MEMS mic → inverting opamp (gain -20) → J17 pin 4
//!          (LPADC_D0P / SE1, AC-coupled on EVB) → AUDADC slot 0 @ ~16 kHz.
//!
//! Playback: g_recBuf → phase-accumulator resampler → I2S0 TX @ ~23.4 kHz
//!           → MAX98357A speaker (GPIO 11/12/13).
//!
//! Wiring:
//!   Opamp output  -> J17 pin 4  (LPADC_D0P_SE1)
//!   Opamp GND ref -> J17 pin 1 or 2 (GND)
//!   MAX98357 DIN  -> GPIO 12  (I2S0_SDOUT)
//!   MAX98357 BCLK -> GPIO 11  (I2S0_CLK)
//!   MAX98357 LRC  -> GPIO 13  (I2S0_WS)
//!   Button SW2    -> GPIO 19  (active low, 100 k pull-up)
//!
//! Sample rates:
//!   AUDADC record : HFRC2 48 MHz / IRTT_DIV32 / 94 = 15,957 Hz
//!   I2S TX play   : HFRC  1.5 MHz / 64 bits/frame = 23,438 Hz
//!   Resampler     : phase-accumulator (nearest-neighbour, adequate for voice)
//!
//! AUDADC notes:
//!   - SAMPMODE_MED required: SE1 (D0P pin) is the high-gain-path input and is
//!     only accessible in MED mode. LP mode only exposes SE0/SE2.
//!   - All 4 PGA channels must be powered even when only 1 slot is used.
//!   - DMA sample count (480) is a multiple of 12 (the FIFO 75% DMA trigger
//!     threshold). A non-multiple can leave samples stuck in the FIFO and
//!     prevent the DMA completion interrupt from ever firing.
//!   - HFRC2 is started inside audadc_init(), AFTER am_hal_audadc_power_control.
//!     Starting it earlier (e.g. during I2S init) prevents the AUDADC clock
//!     domain from seeing a stable source when configure() writes the clock
//!     selection register.
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
// I2S configuration
//*****************************************************************************

#define I2S_MODULE          0
#define I2S_CLK_PIN         11
#define I2S_CLK_FUNC        AM_HAL_PIN_11_I2S0_CLK
#define I2S_WS_PIN          13
#define I2S_WS_FUNC         AM_HAL_PIN_13_I2S0_WS
#define I2S_SDOUT_PIN       12
#define I2S_SDOUT_FUNC      AM_HAL_PIN_12_I2S0_SDOUT

// Words per DMA ping-pong half.  Stereo: 2 words per frame → 128 frames.
#define I2S_DMA_WORDS       256

//*****************************************************************************
// AUDADC configuration
//*****************************************************************************

// DMA ping-pong half size. MUST be a multiple of 12 (FIFO 75% threshold).
// 480 = 40 × 12.  At 15,957 Hz this is one buffer every ~30 ms.
#define AUDADC_DMA_SAMPLES  480

// AUDADC sample rate = HFRC2_48MHz / IRTT_DIV32 / (IRTT_COUNT + 1)
//                    = 48,000,000 / 32 / 94  =  15,957 Hz
#define AUDADC_IRTT_DIV     AM_HAL_AUDADC_RPTT_CLK_DIV32
#define AUDADC_IRTT_COUNT   93
#define AUDADC_SAMPLE_RATE  15957

// Internal pre-amplifier gain (dB). Must be set on all 4 channels even if
// only one slot is used — HAL requirement.
#define AUDADC_PREAMP_DB    12

// Digital PGA gain code.  code = gain_dB × 2 + 12.
// code 12 = 0 dB.  External opamp supplies ~26 dB so PGA stays at 0 dB.
#define AUDADC_PGA_CODE     12

//*****************************************************************************
// Recording
//*****************************************************************************

#define REC_SECONDS         5
#define REC_SAMPLES         (AUDADC_SAMPLE_RATE * REC_SECONDS)   // 79,785

//*****************************************************************************
// Playback resampler  (phase-accumulator, 16.16 fixed-point)
//
// Each I2S TX sample (23,437.5 Hz) advances the read pointer by:
//   inc = (AUDADC_SAMPLE_RATE / 23437.5) × 65536 = 44,618
// Nearest-neighbour: adequate for voice.
//*****************************************************************************
#define RESAMP_INC          44618UL
#define I2S_TX_RATE         23438U   // approximate; used only for SEC_CHUNKS

//*****************************************************************************
// Test-tone (1 kHz square wave, 2 s)
//*****************************************************************************
#define TONE_FREQ_HZ        1000
#define TONE_HALF_SAMPLES   (AUDADC_SAMPLE_RATE / (TONE_FREQ_HZ * 2))  // ~8
#define TONE_DURATION       (AUDADC_SAMPLE_RATE * 2)                    // 2 s
#define TONE_AMPLITUDE      0x200000   // 25 % of 24-bit full scale

//*****************************************************************************
// Per-second stats tick counts
//*****************************************************************************

// I2S TX fires every (I2S_DMA_WORDS/2) = 128 stereo frames.
// 23438 / 128 ≈ 183 callbacks per second.
#define I2S_SEC_CHUNKS      183

// AUDADC fires every AUDADC_DMA_SAMPLES samples.
// 15957 / 480 ≈ 33 callbacks per second.
#define AUD_SEC_CHUNKS      33

//*****************************************************************************
// Button
//*****************************************************************************
#define BTN_PIN             19

//*****************************************************************************
// Playback volume  (right-shift on 24-bit sample).  1 = –6 dB.
//*****************************************************************************
#define PLAY_VOL_SHIFT      1

//*****************************************************************************
// DMA buffers  (shared RAM, 16-byte aligned via padding)
//*****************************************************************************

// I2S TX ping-pong
AM_SHARED_RW static uint32_t g_i2sTxRaw[I2S_DMA_WORDS * 2 + 4];

// AUDADC ping-pong: contiguous, primary then secondary.
// Laid out exactly like the Ambiq reference example.
AM_SHARED_RW static uint32_t g_audadcRaw[AUDADC_DMA_SAMPLES * 2 + 4];

//*****************************************************************************
// Recording buffer  (5 s × 15,957 Hz × 2 bytes ≈ 160 KB)
//*****************************************************************************
AM_SHARED_RW static int16_t g_recBuf[REC_SAMPLES];

//*****************************************************************************
// HAL handles
//*****************************************************************************
static void *g_pI2S    = NULL;
static void *g_pAUDADC = NULL;

//*****************************************************************************
// State machine
//*****************************************************************************
typedef enum
{
    MODE_IDLE,
    MODE_RECORDING,
    MODE_PLAYING,
    MODE_TESTTONE,
} mic_mode_t;

static volatile mic_mode_t g_mode    = MODE_IDLE;
static volatile uint32_t   g_recPos  = 0;   // AUDADC ISR write cursor
static volatile uint32_t   g_recLen  = 0;   // samples captured
static volatile int16_t    g_recDC   = 0;   // mean subtracted at playback

// Playback resampler (I2S ISR only)
static volatile uint32_t   g_playInt  = 0;  // integer sample index
static volatile uint32_t   g_playFrac = 0;  // 16-bit fractional phase

// Test-tone
static volatile uint32_t   g_tonePos  = 0;

// ISR → task flags
static volatile bool        g_recDone  = false;
static volatile bool        g_playDone = false;
static volatile bool        g_toneDone = false;

// AUDADC ping-pong: handled by am_hal_audadc_dma_get_buffer() which also
// invalidates the Apollo4 DAXI cache before returning the buffer pointer.
// Direct buffer access without this call returns stale cached zeros.

//*****************************************************************************
// Per-second stats — AUDADC input path (updated in AUDADC ISR, always active)
//*****************************************************************************
static volatile uint32_t g_audCnt     = 0;
static volatile int32_t  g_audSumAbs  = 0;
static volatile int32_t  g_audPeak    = 0;
static volatile int16_t  g_audMin     = INT16_MAX;
static volatile int16_t  g_audMax     = INT16_MIN;
// Latched for task to read
static volatile int32_t  g_audAvg     = 0;
static volatile int32_t  g_audPkLast  = 0;
static volatile int16_t  g_audMinLast = 0;
static volatile int16_t  g_audMaxLast = 0;
static volatile bool     g_audPrint   = false;

//*****************************************************************************
// Per-second stats — I2S TX output path (updated in I2S ISR, PLAY/TONE only)
//*****************************************************************************
static volatile uint32_t g_i2sCnt    = 0;
static volatile int32_t  g_i2sSumAbs = 0;
static volatile int32_t  g_i2sPeak   = 0;
static volatile int32_t  g_i2sAvg    = 0;
static volatile int32_t  g_i2sPkLast = 0;
static volatile bool     g_i2sPrint  = false;

//*****************************************************************************
// I2S static configuration
//*****************************************************************************
static am_hal_i2s_transfer_t g_i2sXfer;

static am_hal_i2s_io_signal_t g_i2sIO =
{
    .eFyncCpol = AM_HAL_I2S_IO_FSYNC_CPOL_HIGH,
    .eTxCpol   = AM_HAL_I2S_IO_TX_CPOL_FALLING,
    .eRxCpol   = AM_HAL_I2S_IO_RX_CPOL_RISING,
};

static am_hal_i2s_data_format_t g_i2sData =
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

static am_hal_i2s_config_t g_i2sCfg =
{
    .eClock = eAM_HAL_I2S_CLKSEL_HFRC_1_5MHz,
    .eDiv3  = 0,
    .eASRC  = 0,
    .eMode  = AM_HAL_I2S_IO_MODE_MASTER,
    .eXfer  = AM_HAL_I2S_XFER_TX,      // TX only — RX is handled by AUDADC
    .eData  = &g_i2sData,
    .eIO    = &g_i2sIO,
};

//*****************************************************************************
// AUDADC DMA config  (addresses filled in at runtime)
//*****************************************************************************
static am_hal_audadc_dma_config_t g_audDMA =
{
    .bDynamicPriority         = true,
    .ePriority                = AM_HAL_AUDADC_PRIOR_SERVICE_IMMED,
    .bDMAEnable               = true,
    .ui32SampleCount          = AUDADC_DMA_SAMPLES,
    .ui32TargetAddress        = 0,   // set in audadc_init()
    .ui32TargetAddressReverse = 0,   // set in audadc_init()
};

//*****************************************************************************
//
// I2S0 ISR — TX only (playback resampler + test tone)
//
//*****************************************************************************
void am_dspi2s0_isr(void)
{
    uint32_t status;
    am_hal_i2s_interrupt_status_get(g_pI2S, &status, true);
    am_hal_i2s_interrupt_clear(g_pI2S, status);
    am_hal_i2s_interrupt_service(g_pI2S, status, &g_i2sCfg);

    if (!(status & AM_HAL_I2S_INT_TXDMACPL))
    {
        return;
    }

    uint32_t *tx =
        (uint32_t *)am_hal_i2s_dma_get_buffer(g_pI2S, AM_HAL_I2S_XFER_TX);

    for (uint32_t i = 0; i < I2S_DMA_WORDS; i += 2)
    {
        uint32_t word = 0;

        if (g_mode == MODE_PLAYING && g_playInt < g_recLen)
        {
            // Advance phase accumulator
            g_playFrac += RESAMP_INC;
            if (g_playFrac >= 65536UL)
            {
                g_playFrac -= 65536UL;
                g_playInt++;
            }

            if (g_playInt < g_recLen)
            {
                // Scale 16-bit ADC sample to 24-bit for MAX98357A, subtract DC.
                int32_t s = ((int32_t)g_recBuf[g_playInt] - (int32_t)g_recDC) << 8;
                word = (uint32_t)(s >> PLAY_VOL_SHIFT) & 0x00FFFFFFUL;

                // Stats (abs value, scaled back to 12-bit range for readability)
                int32_t a = (s < 0 ? -s : s) >> 16;
                g_i2sSumAbs += a;
                if (a > g_i2sPeak) g_i2sPeak = a;
            }
        }
        else if (g_mode == MODE_PLAYING && g_playInt >= g_recLen)
        {
            g_mode     = MODE_IDLE;
            g_playDone = true;
        }
        else if (g_mode == MODE_TESTTONE && g_tonePos < TONE_DURATION)
        {
            int32_t val = ((g_tonePos / TONE_HALF_SAMPLES) & 1U)
                          ? -TONE_AMPLITUDE : TONE_AMPLITUDE;
            word = (uint32_t)val & 0x00FFFFFFUL;
            g_tonePos++;
            if (g_tonePos >= TONE_DURATION)
            {
                g_mode     = MODE_IDLE;
                g_toneDone = true;
            }
        }
        // else: silence (word = 0)

        tx[i]     = word;   // left
        tx[i + 1] = word;   // right
    }

    // Per-second I2S TX stats tick
    g_i2sCnt++;
    if (g_i2sCnt >= I2S_SEC_CHUNKS)
    {
        g_i2sAvg    = g_i2sSumAbs / (I2S_SEC_CHUNKS * (I2S_DMA_WORDS / 2));
        g_i2sPkLast = g_i2sPeak;
        g_i2sSumAbs = 0;
        g_i2sPeak   = 0;
        g_i2sCnt    = 0;
        g_i2sPrint  = true;
    }
}

//*****************************************************************************
//
// AUDADC ISR — recording path + live input stats
//
// Completion detection follows the Ambiq reference example exactly:
//   Check FIFOOVR1 interrupt + DMASTAT.DMACPL flag.
//   Also accept DCMP (DMA complete) as an alternative trigger, since with
//   certain timing the completion may arrive via either path.
//
//*****************************************************************************
void am_audadc0_isr(void)
{
    uint32_t status;
    am_hal_audadc_interrupt_status(g_pAUDADC, &status, false);
    am_hal_audadc_interrupt_clear(g_pAUDADC, status);

    // Determine whether a DMA buffer just completed.
    // FIFOOVR1 + DMACPL is the reference pattern.
    // DCMP alone is also a valid completion signal.
    bool dmaDone =
        ((status & AM_HAL_AUDADC_INT_FIFOOVR1) && AUDADCn(0)->DMASTAT_b.DMACPL) ||
        (status & AM_HAL_AUDADC_INT_DCMP);

    if (dmaDone)
    {
        // Re-arm DMA for the next ping-pong buffer first, then retrieve the
        // just-completed buffer via am_hal_audadc_dma_get_buffer().
        // CRITICAL: dma_get_buffer calls am_hal_daxi_control(INVALIDATE) which
        // flushes the Apollo4 DAXI cache.  Without this, DMA writes are not
        // visible to the CPU and every sample reads back as zero.
        am_hal_audadc_interrupt_service(g_pAUDADC, &g_audDMA);
        const uint32_t *buf =
            (const uint32_t *)am_hal_audadc_dma_get_buffer(g_pAUDADC);

        //----------------------------------------------------------------------
        // Live input stats — always active regardless of mode.
        // Operations: add, compare.  No multiply/divide.  Safe in ISR.
        //----------------------------------------------------------------------
        for (uint32_t i = 0; i < AUDADC_DMA_SAMPLES; i++)
        {
            // Each DMA word: upper 16 bits = channel tag, lower 16 = sample.
            int16_t s = (int16_t)(buf[i] & 0xFFFFU);
            int32_t a = (s < 0) ? -(int32_t)s : (int32_t)s;
            g_audSumAbs += a;
            if (a > g_audPeak) g_audPeak = a;
            if (s < g_audMin)  g_audMin  = s;
            if (s > g_audMax)  g_audMax  = s;
        }

        g_audCnt++;
        if (g_audCnt >= AUD_SEC_CHUNKS)
        {
            g_audAvg     = g_audSumAbs / (AUD_SEC_CHUNKS * AUDADC_DMA_SAMPLES);
            g_audPkLast  = g_audPeak;
            g_audMinLast = g_audMin;
            g_audMaxLast = g_audMax;
            g_audSumAbs  = 0;
            g_audPeak    = 0;
            g_audMin     = INT16_MAX;
            g_audMax     = INT16_MIN;
            g_audCnt     = 0;
            g_audPrint   = true;
        }

        //----------------------------------------------------------------------
        // Recording — store samples into g_recBuf.
        //----------------------------------------------------------------------
        if (g_mode == MODE_RECORDING)
        {
            for (uint32_t i = 0; i < AUDADC_DMA_SAMPLES && g_recPos < REC_SAMPLES; i++)
            {
                g_recBuf[g_recPos++] = (int16_t)(buf[i] & 0xFFFFU);
            }

            if (g_recPos >= REC_SAMPLES)
            {
                g_recLen  = g_recPos;
                g_mode    = MODE_IDLE;
                g_recDone = true;
            }
        }
    }

    if (status & AM_HAL_AUDADC_INT_DERR)
    {
        am_util_stdio_printf("[mic] AUDADC DMA error\n");
    }
}

//*****************************************************************************
//
// audadc_init  —  initialise AUDADC, follow reference example sequence exactly
//
// Key ordering rules observed from the Ambiq power-cycling reference:
//   1. pwrctrl_periph_enable
//   2. refgen_powerup
//   3. pga_powerup + gain_set for ALL 4 channels (even unused ones)
//   4. audadc_initialize + power_control(WAKE)
//   5. START HFRC2 HERE (after power_control, before configure)
//   6. audadc_configure
//   7. configure_irtt → enable → irtt_enable → configure_dma → interrupt_enable
//   8. internal_pga_config
//   9. configure_slot(s)
//  10. NVIC_SetPriority + NVIC_EnableIRQ
//
//*****************************************************************************
static bool audadc_init(uint32_t ptrA)
{
    //--------------------------------------------------------------------------
    // 1-3. Power and analog front-end
    //--------------------------------------------------------------------------
    // Note: am_hal_audadc_power_control(WAKE) calls pwrctrl_periph_enable
    // internally (am_hal_audadc.c line 1741), so we do NOT call it separately.
    am_hal_audadc_refgen_powerup();

    // All 4 PGA channels MUST be powered regardless of how many slots are used.
    am_hal_audadc_pga_powerup(0);
    am_hal_audadc_pga_powerup(1);
    am_hal_audadc_pga_powerup(2);
    am_hal_audadc_pga_powerup(3);

    am_hal_audadc_gain_set(0, 2 * AUDADC_PREAMP_DB);
    am_hal_audadc_gain_set(1, 2 * AUDADC_PREAMP_DB);
    am_hal_audadc_gain_set(2, 2 * AUDADC_PREAMP_DB);
    am_hal_audadc_gain_set(3, 2 * AUDADC_PREAMP_DB);

    // No mic-bias: external mic is powered at 3.3 V.

    //--------------------------------------------------------------------------
    // 4. Initialise and wake the AUDADC peripheral.
    //--------------------------------------------------------------------------
    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_initialize(0, &g_pAUDADC))
    {
        am_util_stdio_printf("[mic] AUDADC initialize FAILED\n");
        return false;
    }

    if (AM_HAL_STATUS_SUCCESS !=
        am_hal_audadc_power_control(g_pAUDADC, AM_HAL_SYSCTRL_WAKE, false))
    {
        am_util_stdio_printf("[mic] AUDADC power_control FAILED\n");
        return false;
    }

    //--------------------------------------------------------------------------
    // 5. Start HFRC2 NOW — after power_control, before configure.
    //    The reference example places this call here so the AUDADC clock domain
    //    sees a stable source when configure() writes the CLKSEL register.
    //    Starting HFRC2 earlier (e.g. during I2S init) is insufficient because
    //    the AUDADC power domain was not yet active.
    //--------------------------------------------------------------------------
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFRC2_START, false);
    am_util_delay_us(500);   // allow HFRC2 PLL to lock (reference uses 200 µs;
                              // 500 µs gives extra margin)

    //--------------------------------------------------------------------------
    // 6. Configure AUDADC core.
    //
    //    SAMPMODE_MED is required because SE1 (LPADC_D0P, J17 pin 4) is the
    //    high-gain-path input.  LP mode only activates SE0/SE2.
    //
    //    IRTT rate = 48 MHz / 32 / 94 = 15,957 Hz.
    //--------------------------------------------------------------------------
    am_hal_audadc_config_t cfg =
    {
        .eClock         = AM_HAL_AUDADC_CLKSEL_HFRC2_48MHz,
        .ePolarity      = AM_HAL_AUDADC_TRIGPOL_RISING,
        .eTrigger       = AM_HAL_AUDADC_TRIGSEL_SOFTWARE,
        .eClockMode     = AM_HAL_AUDADC_CLKMODE_LOW_LATENCY,
        .ePowerMode     = AM_HAL_AUDADC_LPMODE0,
        .eRepeat        = AM_HAL_AUDADC_REPEATING_SCAN,
        .eRepeatTrigger = AM_HAL_AUDADC_RPTTRIGSEL_INT,
        .eSampMode      = AM_HAL_AUDADC_SAMPMODE_MED,
    };

    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_configure(g_pAUDADC, &cfg))
    {
        am_util_stdio_printf("[mic] AUDADC configure FAILED\n");
        return false;
    }

    //--------------------------------------------------------------------------
    // 7. IRTT, enable, DMA, interrupts.
    //--------------------------------------------------------------------------
    am_hal_audadc_irtt_config_t irtt =
    {
        .bIrttEnable      = true,
        .eClkDiv          = AUDADC_IRTT_DIV,
        .ui32IrttCountMax = AUDADC_IRTT_COUNT,
    };
    am_hal_audadc_configure_irtt(g_pAUDADC, &irtt);

    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_enable(g_pAUDADC))
    {
        am_util_stdio_printf("[mic] AUDADC enable FAILED\n");
        return false;
    }

    am_hal_audadc_irtt_enable(g_pAUDADC);

    // Contiguous ping-pong: primary at ptrA, secondary at ptrA + count×4.
    // This mirrors the reference example exactly.
    g_audDMA.ui32TargetAddress        = ptrA;
    g_audDMA.ui32TargetAddressReverse = ptrA + (uint32_t)(AUDADC_DMA_SAMPLES * sizeof(uint32_t));

    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_configure_dma(g_pAUDADC, &g_audDMA))
    {
        am_util_stdio_printf("[mic] AUDADC configure_dma FAILED\n");
        return false;
    }

    // Enable all relevant interrupts (matches reference example):
    //   FIFOOVR1 : FIFO ≥ 75% — used with DMACPL to detect completion
    //   FIFOOVR2 : FIFO full   — overflow guard
    //   DCMP     : DMA count = 0 — alternative completion signal
    //   DERR     : DMA error
    am_hal_audadc_interrupt_enable(g_pAUDADC,
        AM_HAL_AUDADC_INT_FIFOOVR1 |
        AM_HAL_AUDADC_INT_FIFOOVR2 |
        AM_HAL_AUDADC_INT_DCMP     |
        AM_HAL_AUDADC_INT_DERR);

    //--------------------------------------------------------------------------
    // 8. Digital PGA gain codes  (0 dB — opamp already provides gain).
    //--------------------------------------------------------------------------
    am_hal_audadc_gain_config_t pga =
    {
        .ui32LGA      = AUDADC_PGA_CODE,
        .ui32HGADELTA = 0,
        .ui32LGB      = AUDADC_PGA_CODE,
        .ui32HGBDELTA = 0,
        .eUpdateMode  = AM_HAL_AUDADC_GAIN_UPDATE_IMME,
    };
    am_hal_audadc_internal_pga_config(g_pAUDADC, &pga);

    //--------------------------------------------------------------------------
    // 9. Slot configuration.
    //
    //    Slot 0: SE1 = LPADC_D0P = J17 pin 4.
    //    Slots 1-3: disabled.  ui32TrkCyc = 30 = AM_HAL_AUDADC_MIN_TRKCYC.
    //--------------------------------------------------------------------------
    am_hal_audadc_slot_config_t slot =
    {
        .eMeasToAvg     = AM_HAL_AUDADC_SLOT_AVG_1,
        .ui32TrkCyc     = 30,
        .ePrecisionMode = AM_HAL_AUDADC_SLOT_12BIT,
        .eChannel       = AM_HAL_AUDADC_SLOT_CHSEL_SE1,
        .bWindowCompare = false,
        .bEnabled       = true,
    };

    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_configure_slot(g_pAUDADC, 0, &slot))
    {
        am_util_stdio_printf("[mic] AUDADC slot 0 configure FAILED\n");
        return false;
    }

    // Do NOT call configure_slot for slots 1-3 even with bEnabled=false.
    // The HAL's g_AUDADCSlotsConfigured counter increments on every call
    // regardless of bEnabled, and the count affects DMA behaviour.

    //--------------------------------------------------------------------------
    // 10. NVIC  (AM_IRQ_PRIORITY_DEFAULT, NOT the FreeRTOS kernel priority)
    //--------------------------------------------------------------------------
    NVIC_SetPriority(AUDADC0_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(AUDADC0_IRQn);

    am_util_stdio_printf("[mic] AUDADC init OK  rate=%d Hz  buf=%d samples\n",
                         AUDADC_SAMPLE_RATE, AUDADC_DMA_SAMPLES);
    return true;
}

//*****************************************************************************
//
// i2s_init  —  TX-only I2S for playback
//
//*****************************************************************************
static void i2s_init(uint32_t txA, uint32_t txB)
{
    am_hal_gpio_pincfg_t out = { .GP.cfg_b.eGPOutCfg = 1, .GP.cfg_b.ePullup = 0 };

    out.GP.cfg_b.uFuncSel = I2S_CLK_FUNC;
    am_hal_gpio_pinconfig(I2S_CLK_PIN, out);

    out.GP.cfg_b.uFuncSel = I2S_WS_FUNC;
    am_hal_gpio_pinconfig(I2S_WS_PIN, out);

    out.GP.cfg_b.uFuncSel = I2S_SDOUT_FUNC;
    am_hal_gpio_pinconfig(I2S_SDOUT_PIN, out);

    am_hal_i2s_initialize(I2S_MODULE, &g_pI2S);
    am_hal_i2s_power_control(g_pI2S, AM_HAL_I2S_POWER_ON, false);
    am_hal_i2s_configure(g_pI2S, &g_i2sCfg);
    am_hal_i2s_enable(g_pI2S);

    memset(&g_i2sXfer, 0, sizeof(g_i2sXfer));
    g_i2sXfer.ui32TxTotalCount        = I2S_DMA_WORDS;
    g_i2sXfer.ui32TxTargetAddr        = txA;
    g_i2sXfer.ui32TxTargetAddrReverse = txB;

    am_hal_i2s_dma_configure(g_pI2S, &g_i2sCfg, &g_i2sXfer);

    NVIC_SetPriority(I2S0_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(I2S0_IRQn);
}

//*****************************************************************************
//
// post_record_analysis  —  run in task context after recording completes
//
//*****************************************************************************
static void post_record_analysis(void)
{
    if (g_recLen == 0)
    {
        return;
    }

    // Mean (DC offset)
    int64_t sum = 0;
    int16_t rMin = g_recBuf[0], rMax = g_recBuf[0];
    for (uint32_t k = 0; k < g_recLen; k++)
    {
        sum += g_recBuf[k];
        if (g_recBuf[k] < rMin) rMin = g_recBuf[k];
        if (g_recBuf[k] > rMax) rMax = g_recBuf[k];
    }
    int32_t mean = (int32_t)(sum / (int64_t)g_recLen);
    g_recDC = (int16_t)mean;

    // AC RMS (integer sqrt)
    int64_t sqSum = 0;
    for (uint32_t k = 0; k < g_recLen; k++)
    {
        int32_t v = (int32_t)g_recBuf[k] - mean;
        sqSum += (int64_t)v * v;
    }
    uint32_t rms = 0;
    {
        int64_t avg = sqSum / (int64_t)g_recLen;
        uint32_t r = 0, bit = 1u << 15;
        while (bit) { uint32_t t = r | bit; if ((int64_t)t*t <= avg) r = t; bit >>= 1; }
        rms = r;
    }

    am_util_stdio_printf("[mic] --- RECORDING COMPLETE ---\n");
    am_util_stdio_printf("[mic]   samples : %lu\n", (unsigned long)g_recLen);
    am_util_stdio_printf("[mic]   mean    : %ld  (DC offset)\n", (long)mean);
    am_util_stdio_printf("[mic]   min/max : %d / %d  span=%d\n",
                         rMin, rMax, (int)(rMax - rMin));
    am_util_stdio_printf("[mic]   rms(AC) : %lu (12-bit scale: noise<10, speech>100)\n",
                         (unsigned long)rms);
    am_util_stdio_printf("[mic] Short press = play.  Hold 2 s = test tone.\n");

    // Dump 16 raw samples from mid-recording for format verification
    uint32_t mid = g_recLen / 2;
    am_util_stdio_printf("[mic]   raw[%lu..%lu]: ", (unsigned long)mid,
                         (unsigned long)(mid + 15));
    for (int d = 0; d < 16; d++)
    {
        am_util_stdio_printf("%d ", g_recBuf[mid + d]);
    }
    am_util_stdio_printf("\n");
}

//*****************************************************************************
//
// MicTask
//
//*****************************************************************************
void MicTask(void *pvParameters)
{
    (void)pvParameters;

    //--------------------------------------------------------------------------
    // Align DMA buffer pointers to 16 bytes.
    //--------------------------------------------------------------------------
    uint32_t i2sTxA = (uint32_t)((uint32_t)(g_i2sTxRaw  + 3) & ~0xFU);
    uint32_t i2sTxB = i2sTxA + (uint32_t)(I2S_DMA_WORDS * sizeof(uint32_t));

    // AUDADC uses a contiguous ping-pong buffer: primary then secondary.
    uint32_t audA   = (uint32_t)((uint32_t)(g_audadcRaw + 3) & ~0xFU);
    // audB is set inside audadc_init() using audA + count×4.

    memset((void *)i2sTxA, 0, I2S_DMA_WORDS * sizeof(uint32_t));
    memset((void *)i2sTxB, 0, I2S_DMA_WORDS * sizeof(uint32_t));

    //--------------------------------------------------------------------------
    // Button GPIO
    //--------------------------------------------------------------------------
    am_hal_gpio_pincfg_t btn = {0};
    btn.GP.cfg_b.eGPOutCfg = 0;
    btn.GP.cfg_b.eGPInput  = 1;
    btn.GP.cfg_b.ePullup   = 6;   // 100 k
    btn.GP.cfg_b.uFuncSel  = 3;   // GPIO function
    am_hal_gpio_pinconfig(BTN_PIN, btn);

    //--------------------------------------------------------------------------
    // Initialise peripherals.
    // Order: I2S first (no HFRC2 dependency), then AUDADC (starts HFRC2 inside).
    //--------------------------------------------------------------------------
    i2s_init(i2sTxA, i2sTxB);

    if (!audadc_init(audA))
    {
        am_util_stdio_printf("[mic] AUDADC init failed -- task suspended\n");
        while (1);
    }

    am_hal_interrupt_master_enable();

    // Start I2S TX streaming (silence until a mode is active).
    am_hal_i2s_dma_transfer_start(g_pI2S, &g_i2sCfg);

    // First software trigger; IRTT keeps repeating automatically after this.
    if (AM_HAL_STATUS_SUCCESS != am_hal_audadc_sw_trigger(g_pAUDADC))
    {
        am_util_stdio_printf("[mic] AUDADC sw_trigger FAILED\n");
    }

    am_util_stdio_printf("[mic] Ready.  Short press=rec/play, hold 2s=tone.\n");

    TickType_t xLast    = xTaskGetTickCount();
    bool       prevBtn  = false;
    uint32_t   deadCnt  = 0;
    uint32_t   holdCnt  = 0;

    while (1)
    {
        vTaskDelayUntil(&xLast, pdMS_TO_TICKS(10));

        //----------------------------------------------------------------------
        // Button  (10 ms polling, 50 ms debounce, 2 s long-press threshold)
        //----------------------------------------------------------------------
        uint32_t btnVal;
        am_hal_gpio_state_read(BTN_PIN, AM_HAL_GPIO_INPUT_READ, &btnVal);
        bool down = (btnVal == 0);

        if (deadCnt > 0)
        {
            deadCnt--;
        }
        else if (down && !prevBtn)
        {
            holdCnt = 0;
        }
        else if (down && prevBtn)
        {
            holdCnt++;
            if (holdCnt == 200 && g_mode == MODE_IDLE)  // 200 × 10 ms = 2 s
            {
                g_tonePos = 0;
                g_mode    = MODE_TESTTONE;
                am_util_stdio_printf("[mic] Test tone: 1 kHz square, 2 s...\n");
                deadCnt = 5;
            }
        }
        else if (!down && prevBtn)
        {
            if (holdCnt < 200 && g_mode == MODE_IDLE)
            {
                deadCnt = 5;
                if (g_recLen > 0)
                {
                    g_playInt  = 0;
                    g_playFrac = 0;
                    g_mode     = MODE_PLAYING;
                    am_util_stdio_printf("[mic] Playing %lu samples...\n",
                                         (unsigned long)g_recLen);
                }
                else
                {
                    g_recPos = 0;
                    g_mode   = MODE_RECORDING;
                    am_util_stdio_printf("[mic] Recording %d s...\n", REC_SECONDS);
                }
            }
            holdCnt = 0;
        }
        prevBtn = down;

        //----------------------------------------------------------------------
        // ISR → task events
        //----------------------------------------------------------------------
        if (g_recDone)
        {
            g_recDone = false;
            post_record_analysis();
        }

        if (g_toneDone)
        {
            g_toneDone = false;
            am_util_stdio_printf("[mic] Test tone done.\n");
        }

        if (g_playDone)
        {
            g_playDone = false;
            am_util_stdio_printf("[mic] Playback done.\n");
        }

        //----------------------------------------------------------------------
        // One-shot diagnostic dump 3 s after boot.
        // Reads raw AUDADC registers directly so we can see exactly what the
        // hardware is doing regardless of whether the ISR is firing.
        //   CFG.ADCEN  : 1 = AUDADC is enabled
        //   INTTRIGTIMER.TIMEREN : 1 = IRTT is running
        //   DMASTAT.DMATIP : 1 = DMA transfer currently in progress
        //   DMATOTCOUNT : samples remaining in current DMA transfer (0..480)
        //   INTSTAT : pending interrupt bits (non-zero = ISR not clearing them)
        //   INTEN   : which interrupt bits are enabled in hardware
        //----------------------------------------------------------------------
        {
            static uint32_t diagTick = 0;
            static bool     diagDone = false;
            if (!diagDone)
            {
                diagTick++;
                if (diagTick >= 300)   // 300 × 10 ms = 3 s
                {
                    diagDone = true;
                    am_util_stdio_printf("[DIAG] AUDADC register snapshot:
");
                    am_util_stdio_printf("  CFG         = 0x%08X  (ADCEN bit1=%lu)
",
                        (unsigned)AUDADC->CFG,
                        (unsigned long)((AUDADC->CFG >> 1) & 1));
                    am_util_stdio_printf("  INTTRIGTIMER= 0x%08X  (TIMEREN bit0=%lu)
",
                        (unsigned)AUDADC->INTTRIGTIMER,
                        (unsigned long)(AUDADC->INTTRIGTIMER & 1));
                    am_util_stdio_printf("  DMACFG      = 0x%08X  (DMAEN=%lu)
",
                        (unsigned)AUDADC->DMACFG,
                        (unsigned long)(AUDADC->DMACFG & 1));
                    am_util_stdio_printf("  DMASTAT     = 0x%08X  (TIP=%lu CPL=%lu ERR=%lu)
",
                        (unsigned)AUDADC->DMASTAT,
                        (unsigned long)((AUDADC->DMASTAT >> 2) & 1),
                        (unsigned long)((AUDADC->DMASTAT >> 1) & 1),
                        (unsigned long)((AUDADC->DMASTAT >> 0) & 1));
                    am_util_stdio_printf("  DMATOTCOUNT = %lu  (0=complete, 480=never started)
",
                        (unsigned long)AUDADC->DMATOTCOUNT);
                    am_util_stdio_printf("  DMATARGADDR = 0x%08X
",
                        (unsigned)AUDADC->DMATARGADDR);
                    am_util_stdio_printf("  INTSTAT     = 0x%08X  (non-0 = ISR not firing)
",
                        (unsigned)AUDADC->INTSTAT);
                    am_util_stdio_printf("  INTEN       = 0x%08X
",
                        (unsigned)AUDADC->INTEN);
                    am_util_stdio_printf("  FIFOSTAT    = 0x%08X
",
                        (unsigned)AUDADC->FIFOSTAT);
                }
            }
        }

        //----------------------------------------------------------------------
        // Per-second stats — AUDADC input (always active)
        //
        //   avg  : mean |sample| over 1 s  (12-bit, 0–2047)
        //   peak : max  |sample| over 1 s
        //   min  : most negative raw sample
        //   max  : most positive raw sample
        //   span : dynamic range used = max − min
        //
        //   Silence:  avg < 10,  span < 40   (ADC noise floor)
        //   Speech :  avg > 50,  span > 400
        //----------------------------------------------------------------------
        if (g_audPrint)
        {
            g_audPrint = false;
            const char *m = (g_mode == MODE_RECORDING) ? "REC " :
                            (g_mode == MODE_PLAYING)   ? "PLAY" :
                            (g_mode == MODE_TESTTONE)  ? "TONE" : "IDLE";
            am_util_stdio_printf(
                "[IN ] %s  avg=%4ld  peak=%4ld  min=%6d  max=%6d  span=%5ld\n",
                m,
                (long)g_audAvg,
                (long)g_audPkLast,
                (int)g_audMinLast,
                (int)g_audMaxLast,
                (long)((int32_t)g_audMaxLast - (int32_t)g_audMinLast));
        }

        //----------------------------------------------------------------------
        // Per-second stats — I2S TX output (PLAY / TONE only)
        //----------------------------------------------------------------------
        if (g_i2sPrint)
        {
            g_i2sPrint = false;
            if (g_mode == MODE_PLAYING || g_mode == MODE_TESTTONE)
            {
                const char *m = (g_mode == MODE_PLAYING) ? "PLAY" : "TONE";
                am_util_stdio_printf(
                    "[OUT] %s  avg=%4ld  peak=%4ld\n",
                    m, (long)g_i2sAvg, (long)g_i2sPkLast);
            }
        }
    }
}