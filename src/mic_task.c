//*****************************************************************************
//
//! @file mic_task.c
//!
//! @brief Mic record/playback: AUDADC analog mic input, I2S0 TX speaker output.
//!
//! Press BUTTON1 (SW2)   -> record REC_SECONDS of audio from the MEMS mic.
//! Press again           -> play back through MAX98357A speaker via I2S0 TX.
//! Long press (2 s)      -> play 1 kHz test tone.
//!
//! Capture chain, in order:
//!   AUDADC (12-bit, 4 slots) -> DC blocker -> 4 kHz LPF -> decimate to 16 kHz
//!   -> noise gate -> g_pcmBuf -> Opus encode -> g_opusBuf (for BLE transfer)
//!
//! The model feed taps off BEFORE the noise gate, so quiet sounds reach the
//! wake-word model unattenuated.
//!
//! Exactly one test mode may be enabled; each selects what the listening path
//! feeds, and the others are not initialised so their arenas cost nothing:
//!   CHEW_TEST_MODE -> chewallow, 78x24 MFCC, 1024-sample window / 512 hop
//!   WW_TEST_MODE   -> 4-class wake word, 75x24 MFCC, 480 window / 320 hop
//!   neither        -> the original MLPerf KWS path, 49x10 MFCC
//!
//! Wiring (Apollo4p Blue KXR EVB):
//!   MEMS mic OUT   -> 1 uF series cap -> J17 "D0N" (= AUDADC SE0)
//!   MEMS mic VDD   <- J17 "AUDA" (VDDAUDA, measured 1.79 V)
//!                     with 0.1 uF + 10 uF decoupling at the mic (not implemented right now)
//!   MEMS mic GND   -> J17 GND
//!   MAX98357 BCLK  -> GPIO 11 (I2S0_CLK),   on header J11
//!   MAX98357 LRC   -> GPIO 13 (I2S0_WS),    on header J9
//!   MAX98357 DIN   -> GPIO 12 (I2S0_SDOUT), on header J9
//!   MAX98357 SD    -> GPIO 8  (firmware-controlled shutdown, see AMP_SD_PIN)
//!   MAX98357 VIN   -> 3.3 V (separate rail from the mic supply)
//!   Speaker +/-    -> MAX98357 output ONLY. The "-" terminal is a driven
//!                     output on this bridge-tied-load amp, NOT ground.
//!                     Tying anything else to it floats the mic's reference.
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
#include <math.h>           // sqrtf, for the wake-word window normalization

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "FreeRTOS.h"
#include "task.h"

#include "mic_task.h"
#include "ae_api.h"         // Ambiq Opus encoder (audio_enc_init / audio_enc_encode_frame)
#include "opus.h"
#include "opus_private.h"   // OPUS_SET_FORCE_MODE / MODE_CELT_ONLY
#ifndef KWS_DISABLE
#include "kws_inference.h"  // TFLite KWS wrapper
#include "ns_audio_mfcc.h"  // MFCC feature extraction
#include "chewallow_inference.h"
#include "chewallow_dummy_input.h"
#include "ww_inference.h"
#include "ww_dummy_input.h"
#endif

//*****************************************************************************
// Configuration
//*****************************************************************************

#define MIC_I2S_MODULE      0

// I2S0 TX GPIO pins (speaker only -- no SDIN: capture is on AUDADC)
#define MIC_CLK_PIN         11
#define MIC_CLK_FUNC        AM_HAL_PIN_11_I2S0_CLK
#define MIC_WS_PIN          13
#define MIC_WS_FUNC         AM_HAL_PIN_13_I2S0_WS
#define MIC_SDOUT_PIN       12
#define MIC_SDOUT_FUNC      AM_HAL_PIN_12_I2S0_SDOUT

// MAX98357A shutdown control. Driven LOW during listen/record so the amp's
// Class-D switching cannot couple into the mic; HIGH during playback/tone.
// Measured: 0 V when low, 1.79 V when high -- the Adafruit breakout's SD/GAIN
// resistor network forms a divider, so it does not reach a full 3.3 V. 1.79 V
// is comfortably above the amp's enable threshold, so this is fine.
#define AMP_SD_PIN          8

// I2S TX DMA: 256 words = 128 stereo frames per ISR completion (~5.46 ms).
#define MIC_DMA_SAMPLES     256

// Place scratch buffers in .sram_bss (NOLOAD) instead of .shared (AT>MCU_MRAM).
// Same SHARED_SRAM region, so DMA reachability is unchanged -- but no copy is
// stored in flash. g_pcmBuf alone was costing 960 KB of MRAM as literal zeros.
#define MIC_NOLOAD __attribute__((section(".sram_bss"))) __attribute__((aligned(16)))

// AUDADC DMA words per completion. SAMPMODE_MED + 4 slots emits 2 DMA words
// per scan (A-pair + B-pair). 256 words = 128 scans = 128 channel-A samples
// per completion (~5.46 ms -- one ISR call every ~5.46 ms, so ~183/sec).
#define AUDADC_DMA_WORDS    256
#define AUDADC_MONO_SAMPLES 128

// AUDADC slot bit-19 selects A-pair (0) or B-pair (1) within each DMA word.
#define MIC_SRC_A0          0
#define MIC_SRC_A1          1
#define MIC_SRC_B0          2
#define MIC_SRC_B1          3
#define MIC_RECORD_SOURCE   MIC_SRC_A0   // SE0 = analog mic

//*****************************************************************************
// AUDADC gain -- READ THIS BEFORE CHANGING
//
// There are two gain APIs and only ONE of them takes effect:
//
//   am_hal_audadc_gain_set()            <- writes the GAIN register
//   am_hal_audadc_internal_pga_config() <- OVERWRITES the whole GAIN register
//
// mic_audadc_init() calls gain_set() first and internal_pga_config() second,
// and the HAL's internal_pga_config() does a blind _VAL2FLD write with no
// read-modify-write. So gain_set() -- and therefore PREAMP_GAIN_DB -- has no
// effect whatsoever. Verified on hardware: changing PREAMP_GAIN_DB from 12 to
// 18 produced zero change in measured signal level.
//
// CHANNEL_GAIN_DB is the control that actually works. Verified sweep at 1 kHz
// (fixed tone source, quiet room; values are the ISR's per-second "avg"):
//
//   CHANNEL_GAIN_DB   signal   noise   SNR
//         6             450      52    18.7 dB
//        12             753      54    22.9 dB
//        18            1677      58    29.2 dB
//        24            3154      69    33.2 dB   <- current setting
//        30            6243     102    35.7 dB
//
// 24 was chosen over 30: the extra 2.5 dB of SNR was not worth 6 dB of
// clipping headroom, and quiet-room peaks became erratic at 30.
//
// PREAMP_GAIN_DB remains only because the gain_set() calls have not been
// removed. It is dead. Do not tune it.
//*****************************************************************************
#define PREAMP_GAIN_DB      18   // INERT -- see note above
#define CHANNEL_GAIN_DB     24   // the operative gain control

// MICBIAS trim. The mic is externally powered from J17 "AUDA" (1.79 V), not
// from MICBIAS -- but the AUDADC still needs its bias enabled or it produces
// no output at all.
#define MIC_BIAS_TRIM       63

// BUTTON1 (SW2) = GPIO 19. BUTTON0 (SW1, GPIO 17) is polled by ButtonTask in
// rtos.c, where it sends a BLE message and runs the inference timing harness.
#define BTN_PIN             19

// Playback: int16 PCM -> 24-bit I2S TX. Left-shift maps int16 full scale into
// the int24 range; soft limiter knees in well below clipping so loud peaks
// compress instead of hard-clipping the 24-bit packer.
#define PLAY_VOL_SHIFT      9                  // int16 << 9 -> 25-bit, soft-limited
#define PLAY_LIMIT_START    0x500000
#define PLAY_LIMIT_MAX      0x7FFFFF

// AUDADC capture rate. 48 MHz / 8 / 375 = 32,000 exactly.
#define MIC_NATIVE_RATE     32000

// I2S TX rate. HFRC 1.5 MHz / 64 bits per stereo frame = 23,437.5 Hz.
// Unchanged by the capture-rate switch -- these are separate clocks.
#define I2S_NATIVE_RATE     23438

// PCM storage rate -- Opus encoder requires 16 kHz mono int16 input.
#define REC_PCM_RATE        16000
#define REC_SECONDS         30
#define REC_PCM_SAMPLES     (REC_PCM_RATE * REC_SECONDS)

// Opus encoder framing (20 ms @ 16 kHz, CBR 24 kbit/s -> 60 bytes per frame)
#define USE_OPUS14          1     // 1 = neuralSPOT opus1.4 (fixed-point), 0 = Ambiq ae_api blob
#define OPUS_FRAME_SAMPLES  320
#define OPUS_FRAME_BYTES    60
#define OPUS_NUM_FRAMES     (REC_SECONDS * 50)
#define OPUS_BUF_BYTES      (OPUS_NUM_FRAMES * OPUS_FRAME_BYTES)
#define OPUS_ENC_MEM_WORDS  6400          // 25600 bytes; encoder needs 24544
static uint32_t g_encMem[OPUS_ENC_MEM_WORDS];

// Resampler steps in 16.16 Q-format.
//   Capture:  out=16000 from in=23438 -> step = 23438/16000 ~ 1.4649
//   Playback: out=23438 from in=16000 -> step = 16000/23438 ~ 0.6826
#define CAP_STEP_Q16  (uint32_t)(((uint64_t)MIC_NATIVE_RATE << 16) / REC_PCM_RATE)
#define PLAY_STEP_Q16 (uint32_t)(((uint64_t)REC_PCM_RATE << 16) / I2S_NATIVE_RATE)

// Per-second stats: 23438 Hz / 128 mono samples per ISR ~ 183 ISR calls/sec.
#define SEC_CHUNKS          250

#ifndef KWS_DISABLE

//-----------------------------------------------------------------------------
// WW_TEST_MODE
//   1 = the listening path feeds the wake word model. KWS and chewallow are
//       not initialised at all, so their tensor arenas cost nothing.
//   0 = the original KWS path, with chewallow available on the button harness.
//-----------------------------------------------------------------------------
#define WW_TEST_MODE        0
#define CHEW_TEST_MODE      1

#if WW_TEST_MODE && CHEW_TEST_MODE
#error "WW_TEST_MODE and CHEW_TEST_MODE are mutually exclusive"
#endif

// --- audio framing shared by both paths --------------------------------------
#if CHEW_TEST_MODE
// Chewallow hops 512 samples (winstep 0.032 @ 16 kHz), so one ISR chunk is
// exactly one hop. 320 does not divide 512 and the buffer would drift.
#define KWS_FRAME_SAMPLES   512
#else
#define KWS_FRAME_SAMPLES   320     // 20 ms @ 16 kHz
#endif

// --- original MLPerf KWS front end -------------------------------------------
#define KWS_MFCC_COEFFS     10
#define KWS_MFCC_FBANK_BINS 40
#define KWS_MFCC_FRAME_POW2 512

// neuralSPOT ns_mfcc_map_arena / ns_fbanks_map_arena multiply pointer-arithmetic
// offsets by sizeof(T) on a T* pointer, inflating the layout by 4x. The
// buffers don't overlap as long as the arena is large enough; 64 KB is plenty.
#define MFCC_ARENA_BYTES    (64 * 1024)

//-----------------------------------------------------------------------------
// Wake word front end.
//
// These MUST match the training config exactly:
//   n_mfcc 24, nfilt 40, nfft 512, winlen 0.030, winstep 0.020,
//   lowfreq 20, highfreq 4000, experiment 2 (coefficient 0 retained)
//
// The important one is WW_MFCC_WINLEN_SAMP. Training used a 30 ms window with
// a 20 ms hop, so consecutive analysis windows OVERLAP by 160 samples. The
// ISR hands over 320-sample (20 ms) chunks, so a 480-sample window has to be
// assembled from the previous chunk's tail plus the new chunk -- see
// g_wwWindow. Passing the raw 320-sample buffer would read past the end of it
// and compute features from a different window length than training used,
// which shows up as poor accuracy rather than an error.
//
// Note the original KWS config sets frame_len 320 while frame_len_ms says 30;
// those disagree, so do not copy it as a template.
//-----------------------------------------------------------------------------
#define WW_MFCC_COEFFS        24
#define WW_MFCC_FBANK_BINS    40
#define WW_MFCC_FRAME_POW2    512
#define WW_MFCC_LOW_FREQ      20
#define WW_MFCC_HIGH_FREQ     4000
#define WW_MFCC_WINLEN_SAMP   480   // 30 ms analysis window
#define WW_MFCC_HOP_SAMP      320   // 20 ms hop -- equals KWS_FRAME_SAMPLES
#define WW_MFCC_ARENA_BYTES   (64 * 1024)

// Frames between inferences. 38 x 20 ms = 760 ms, so a 1.5 s window advances
// about half its length each time and a phrase that straddles one window
// boundary still lands inside the next. Raise to cut CPU, lower for more
// chances to catch a phrase.
#define WW_HOP_FRAMES         38

// Peak of the audio since the last inference must exceed this to bother
// running the model. Without a gate the model runs on room noise all day.
#define WW_PEAK_GATE          1000

// Report a detection when a non-nothing class exceeds this. Deliberately low
// for testing; measured operating points for deployment are 0.99+.
#define WW_REPORT_THRESHOLD   0.99f

// Print a line on every inference, not just detections. Useful at first, then
// set to 0 once the log gets noisy.
#define WW_VERBOSE            1

// Run the embedded-wav test once at boot. Set to 0 when the front end is
// proven -- the embedded wav costs 48 KB of flash.
#define WW_SELFTEST           1

// Two 320-sample chunks. The analysis window is the FIRST 480 of these.
//
// Python frame k spans samples [k*320, k*320+480). Keeping only the most
// recent 480 samples would place every frame half a hop (10 ms) later than
// training did.
#define WW_MFCC_BUF_SAMP      (2 * WW_MFCC_HOP_SAMP)   /* 640 */

// Preemphasis: y[n] = x[n] - 0.97*x[n-1], over the CONTINUOUS stream.
//
// python_speech_features defaults to preemph=0.97 and applies it to the whole
// signal BEFORE framing, so every frame's first sample depends on the
// previous frame's last one. ns_mfcc_compute only ever sees an isolated
// buffer and cannot reproduce that by itself.
//
// Measured: MFCCs computed on isolated 480-sample slices differ from psf by
// up to 30 units per coefficient, against typical magnitudes of 20-30.
// Preemphasizing the continuous stream first and then slicing matches psf
// exactly (diff 0.000000). Not a rounding-level concern.
#define WW_PREEMPH_Q15        31785   /* 0.97 * 32768 */

// ns_mfcc rounds its output to a whole number -- ns_mfcc.c line ~188 does
//     sum *= (0x1 << cfg->num_dec_bits);
//     sum = round(sum);
// unconditionally, even though mfcc_out is a float*. num_dec_bits is
// therefore the only thing preserving fractional precision.
//
// At 0 the coefficients came back as integers. With typical MFCC magnitudes
// of 1-60 that flattened coefficients 5-23 to 0 or +/-1, and the model saw a
// near-constant vector. 10 bits gives ~0.001 resolution; the scaling is
// divided back out after every ns_mfcc_compute call.
#define WW_MFCC_DEC_BITS      10
#define WW_MFCC_DEC_SCALE     (1.0f / (float)(1 << WW_MFCC_DEC_BITS))

//-----------------------------------------------------------------------------
// Chewallow front end. Must match the DS5 mfcc_creator_instructions exactly:
//   n_mfcc 24, nfilt 32, nfft 1024, winlen 0.064, winstep 0.032,
//   lowfreq 20, highfreq 8000, experiment 2
//
// Simpler framing than the wake word: winlen is exactly 2 x winstep, so a
// 1024-sample buffer holding the last two chunks IS the analysis window. No
// partial read as the wake word's 480/320 required.
//-----------------------------------------------------------------------------
#if CHEW_TEST_MODE
#define CHEW_MFCC_COEFFS        24
#define CHEW_MFCC_FBANK_BINS    32
#define CHEW_MFCC_FRAME_POW2    1024
#define CHEW_MFCC_LOW_FREQ      20
#define CHEW_MFCC_HIGH_FREQ     8000
#define CHEW_MFCC_WINLEN_SAMP   1024   // 64 ms window
#define CHEW_MFCC_HOP_SAMP      512    // 32 ms hop = the feed chunk

// nfft 1024 needs ~4x the wake word's arena, and ns_mfcc_map_arena inflates
// its layout 4x on top of that.
#define CHEW_MFCC_ARENA_BYTES   (128 * 1024)

// Frames between inferences. 78 = no overlap (production cadence, one per
// 2.5 s). 39 halves it for testing so a chewing burst gets two chances.
#define CHEW_HOP_FRAMES         78

#define CHEW_PEAK_GATE          300      // chewing is quiet
#define CHEW_REPORT_THRESHOLD   0.35f
#define CHEW_VERBOSE            1

// ns_mfcc round()s its output unconditionally -- see the WW note above.
#define CHEW_MFCC_DEC_BITS      10
#define CHEW_MFCC_DEC_SCALE     (1.0f / (float)(1 << CHEW_MFCC_DEC_BITS))
#define CHEW_PREEMPH_Q15        31785   // 0.97 * 32768

#if CHEW_MFCC_COEFFS != CHEWALLOW_NUM_COEFFS
#error "CHEW_MFCC_COEFFS must match CHEWALLOW_NUM_COEFFS"
#endif
#if CHEW_MFCC_HOP_SAMP != KWS_FRAME_SAMPLES
#error "The feed chunk must equal the MFCC hop"
#endif
#endif  // CHEW_TEST_MODE



#if WW_MFCC_COEFFS != WW_NUM_COEFFS
#error "WW_MFCC_COEFFS must match WW_NUM_COEFFS in ww_inference.h"
#endif

// Must come AFTER the defines above: the guard depends on WW_SELFTEST, and
// the buffer further down is sized from WW_TEST_AUDIO_SAMPLES.
#if WW_TEST_MODE && WW_SELFTEST
#include "ww_test_audio.h"   // g_ww_test_audio[], from tools/wav_to_header.py
#endif

#endif  // KWS_DISABLE

//*****************************************************************************
// DMA buffers -- shared SRAM, 16-byte aligned
//*****************************************************************************

#ifndef KWS_DISABLE
// Model feed ping-pong (ISR fills one while task reads the other). Shared by
// both the KWS and wake-word paths -- both consume 20 ms chunks.
MIC_NOLOAD static int16_t    g_kwsBuf[2][KWS_FRAME_SAMPLES];
static volatile uint32_t     g_kwsWriteBuf = 0;
static volatile uint32_t     g_kwsWritePos = 0;
static volatile uint32_t     g_kwsReadyBuf = 0xFF;

// Software gain on the model feed only -- brings the AUDADC PCM into the range
// Google Speech Commands was trained on, without touching the recording path.
// Saturates to int16 full scale.
//
// For the wake word this matters less than it looks: the 75x24 window is
// z-score normalized before inference, so absolute level is removed. Gain
// only affects clipping and the peak gate. If loud speech looks clipped,
// try 1 and halve WW_PEAK_GATE.
#define KWS_SW_GAIN          1

// Energy gate for the KWS path: peak |sample| over the 1-second window must
// exceed this to invoke the model. Below it, force-classify as silence --
// eliminates the "go"/"yes" hallucinations the DS-CNN emits on AUDADC noise.
#define KWS_PEAK_GATE        1000
static volatile int32_t      g_kwsWindowPeak = 0;

#if !WW_TEST_MODE
MIC_NOLOAD static uint8_t    g_mfccArena[MFCC_ARENA_BYTES];
static float                 g_mfccFeatures[KWS_NUM_FRAMES * KWS_NUM_COEFFS];
static uint32_t              g_mfccFrameCount = 0;
static ns_mfcc_cfg_t         g_mfccCfg;
#endif

#if WW_TEST_MODE
MIC_NOLOAD static uint8_t g_wwMfccArena[WW_MFCC_ARENA_BYTES];
static ns_mfcc_cfg_t      g_wwMfccCfg;

// 75 frames x 24 coefficients, float32
static float    g_wwFeatures[WW_NUM_FRAMES * WW_MFCC_COEFFS];
static uint32_t g_wwFrameCount = 0;

// Rolling two-chunk buffer of PREEMPHASIZED audio. ns_mfcc_compute reads the
// first WW_MFCC_WINLEN_SAMP (480) of it, which is exactly Python's frame k.
// The 160-sample overlap between consecutive frames falls out of the 320
// sample hop across a 480 sample window.
static int16_t g_wwWindow[WW_MFCC_BUF_SAMP];      // 640, preemphasized
static bool    g_wwPrimed = false;   // needs two chunks before the first MFCC
static int16_t g_wwPreemphPrev = 0;  // last RAW sample of the previous chunk

#if WW_SELFTEST
// Preemphasized copy of the embedded test clip. In .sram_bss rather than
// plain .bss so it does not compete with the tensor arenas for TCM.
MIC_NOLOAD static int16_t g_wwPreBuf[WW_TEST_AUDIO_SAMPLES];
#endif

// Peak since the last inference, for the energy gate. Tracked separately from
// g_kwsWindowPeak because the wake-word window is a different length.
static volatile int32_t g_wwWindowPeak = 0;

// MFCC milliseconds accumulated across the frames of one inference window.
static uint32_t g_wwMfccMsAccum = 0;
static uint32_t g_wwInferCount  = 0;

#elif CHEW_TEST_MODE
MIC_NOLOAD static uint8_t g_chewMfccArena[CHEW_MFCC_ARENA_BYTES];
static ns_mfcc_cfg_t      g_chewMfccCfg;

static float    g_chewFeatures[CHEWALLOW_NUM_FRAMES * CHEW_MFCC_COEFFS];
static uint32_t g_chewFrameCount = 0;

// The analysis window: last two 512-sample chunks, preemphasized.
static int16_t  g_chewWindow[CHEW_MFCC_WINLEN_SAMP];
static bool     g_chewPrimed = false;
static int16_t  g_chewPreemphPrev = 0;

static volatile int32_t g_chewWindowPeak = 0;
static uint32_t g_chewMfccMsAccum = 0;
static uint32_t g_chewInferCount  = 0;

#endif
#endif

// AUDADC ping+pong contiguous buffer (HAL alternates halves automatically).
MIC_NOLOAD static uint32_t g_audadcBufRaw[2 * AUDADC_DMA_WORDS + 3];

// I2S TX ping/pong
MIC_NOLOAD static uint32_t g_txBufRawA[2 * MIC_DMA_SAMPLES + 3];
MIC_NOLOAD static uint32_t g_txBufRawB[2 * MIC_DMA_SAMPLES + 3];

//*****************************************************************************
// Recording buffers (in shared SRAM)
//   g_pcmBuf  -- 16 kHz int16 mono PCM, used for on-device playback
//   g_opusBuf -- Opus 24 kbit/s CBR, ready for BLE transmission
//*****************************************************************************

MIC_NOLOAD static int16_t g_pcmBuf[REC_PCM_SAMPLES];
MIC_NOLOAD static uint8_t g_opusBuf[OPUS_BUF_BYTES];

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

// Test tone: 1 kHz square wave, 2 seconds at native rate. Generated directly
// in the I2S ISR, so it bypasses g_pcmBuf and the playback resampler entirely
// -- useful for checking the amp/speaker path in isolation.
#define TONE_FREQ           1000
#define TONE_HALF_PERIOD    (MIC_NATIVE_RATE / (TONE_FREQ * 2))
#define TONE_AMPLITUDE      0x200000
#define TONE_DURATION       (MIC_NATIVE_RATE * 2)
static volatile uint32_t   g_tonePos  = 0;
static volatile bool       g_toneDone = false;

// 2:1 decimation state. Flips on every input sample; the sample is kept when
// it lands back on zero, so exactly half are stored.
static volatile uint32_t  g_capToggle     = 0;

// DC blocker (single-pole IIR HPF). The mic has a measurable DC bias at the
// AUDADC pin; leaving it in skews FFT bin 0 and any subsequent feature.
//   y[n] = x[n] - x[n-1] + R * y[n-1]
// R = 32440/32768 ~ 0.9900 -> corner ~ 37 Hz at 23.4 kHz (well below speech).
#define DC_BLOCK_R_Q15  32440
static volatile int32_t   g_dcPrevIn      = 0;
static volatile int32_t   g_dcPrevOut     = 0;

//*****************************************************************************
// Capture anti-alias filter: 4th-order Butterworth, fc 7 kHz @ 32 kHz.
// Two cascaded biquads, pole Qs 0.5412 and 1.3066.
//
// Replaces the 2nd-order 4 kHz filter, which was designed for the old
// 23,438 Hz capture rate. Biquad coefficients encode frequency as a fraction
// of the sample rate, so moving to 32 kHz already dragged that corner up to
// 5,469 Hz on its own.
//
// Why 7 kHz: measured on unblended chewing snippets, the 4-8 kHz band holds
// only 9.2% of chewing energy but separates from background by +12.91 dB --
// better than the +10.73 dB below 4 kHz -- and its per-snippet energy is
// uncorrelated with the low band (r = 0.219). Quiet, but not redundant.
// Restricting the MFCC to 4 kHz cost ~3 accuracy points (DS1 vs DS2).
//
// Response: -0.0 dB at 4k, -0.8 at 6k, -3.0 at 7k, -7.7 at 8k, -20.9 at 10k,
// -37.5 at 12k. The 8 kHz figure is weaker than the old filter's -19.6 dB,
// which is the unavoidable trade: 8 kHz is both where signal ends and where
// folding begins.
//
// NOT volatile -- only the ISR touches these, so the compiler can keep them
// in registers instead of reloading from memory every sample.
//*****************************************************************************
#define AA1_B0     0.211137f
#define AA1_B1     0.422275f
#define AA1_A1_N   0.204698f
#define AA1_A2_N  -0.049248f

#define AA2_B0     0.292624f
#define AA2_B1     0.585248f
#define AA2_A1_N   0.283700f
#define AA2_A2_N  -0.454196f

static float g_aa1X1 = 0.0f, g_aa1X2 = 0.0f, g_aa1Y1 = 0.0f, g_aa1Y2 = 0.0f;
static float g_aa2X1 = 0.0f, g_aa2X2 = 0.0f, g_aa2Y1 = 0.0f, g_aa2Y2 = 0.0f;


// Anti-alias / de-hiss biquad LPF (Butterworth 2nd order, fc~4 kHz @ 23.438 kHz,
// Q=0.707). Content above 8 kHz folds back into the audio band as hiss through
// the 23.438->16 kHz decimator. 4 kHz cutoff preserves speech (0.3-3.4 kHz).
//   y[n] = b0(x[n] + x[n-2]) + b1*x[n-1] + a1_n*y[n-1] + a2_n*y[n-2]
//
// NOTE for model work: this runs ahead of BOTH the recording path and the model
// feed, so anything above 4 kHz is gone before any model sees it. The wake-word
// MFCC therefore uses highfreq 4000 to match -- see WW_MFCC_HIGH_FREQ.
#define LPF_B0     0.1615f
#define LPF_B1     0.3231f
#define LPF_A1_N   0.5871f
#define LPF_A2_N  -0.2333f


//*****************************************************************************
// Downward expander (soft noise gate) -- applied ONLY to the recording path.
//
//   gain(env) = 1                   if env >= GATE_THRESH
//   gain(env) = (env/GATE_THRESH)^2 if env <  GATE_THRESH  (clamped to FLOOR)
//
// Tuning history, all judged on BLE-transferred audio played on a phone (which
// bypasses the local playback path):
//
//   GATE_ATTACK_A  0.20 -> 0.005    0.20 tracked individual pitch cycles and
//                                   modulated the waveform; 0.005 (~9 ms)
//                                   tracks syllables instead.
//   GATE_RELEASE_A 0.0015 -> 0.0004 Fixed audible pops. With release only 3x
//                                   slower than attack the gain snapped across
//                                   the threshold; 0.0004 glides through it.
//                                   DO NOT speed this back up.
//   GATE_THRESH    600 -> 3500      Raised until hiss in pauses was gone. No
//                                   word-onset clipping observed even at 3500.
//
// IMPORTANT: at GATE_THRESH 3500 the threshold sits ABOVE typical speech RMS
// (~1000 at CHANNEL_GAIN_DB 24) -- it works because the envelope peaks much
// higher than RMS. This is aggressive processing tuned for a human listener.
// Anything quieter than speech is heavily attenuated, so a model that needs to
// detect quiet events should tap the signal BEFORE this gate, as the model
// feed does.
//*****************************************************************************
#define GATE_THRESH        3500.0f
#define GATE_FLOOR         0.03f
#define GATE_ATTACK_A      0.005f     // ~9 ms attack
#define GATE_RELEASE_A     0.0004f    // slow release -- pop suppression
static volatile float     g_gateEnv       = 0.0f;

// Gate the recording path, or not.
//
// The gate makes audio pleasant for a human listener, but the model feed
// taps UNGATED. Audio recorded with the gate and inferred on without it are
// different signals, and a gate is nonlinear and envelope-dependent -- no
// linear correction can undo it afterwards.
//
// Chewing is the worst case: GATE_THRESH 3500 sits above typical chewing
// level, so the quiet passages that carry the discriminative content get
// squared down toward GATE_FLOOR.
//
// Default false, so recordings are training-safe. Set true only when the
// recording is meant purely for listening.
static volatile bool      g_gateRecording = false;

// Playback-side resampler state (16000 Hz int16 -> 23438 Hz int24)
static volatile uint32_t  g_playPhaseQ16  = 0;
static volatile uint32_t  g_playSrcIdx    = 0;

// Playback-side reconstruction LPF, with separate state from the capture
// filter so the two ISRs cannot corrupt each other. Added while chasing a
// buzzy playback artifact that turned out to be a failed speaker, so its
// benefit is unproven -- a candidate for removal if the float work in the I2S
// ISR ever matters. The audio is already low-passed at 4 kHz during capture,
// so this filters a second time and makes playback slightly duller.
static volatile float     g_plpfX1        = 0.0f;
static volatile float     g_plpfX2        = 0.0f;
static volatile float     g_plpfY1        = 0.0f;
static volatile float     g_plpfY2        = 0.0f;

// Per-second debug stats. g_secAvg is computed on the post-LPF, PRE-GATE
// signal, so it is a true noise-floor measure and is unaffected by gate
// tuning. Use it, not the gated RMS, when comparing hardware changes.
static volatile uint32_t g_dmaBufCount  = 0;
static volatile int32_t  g_secSumAbs    = 0;
static volatile int32_t  g_secAvg       = 0;
static volatile int32_t  g_secPeak      = 0;
static volatile int32_t  g_secPeakLast  = 0;
static volatile uint32_t g_printReady   = 0;
static volatile uint32_t g_audadcIsrCount = 0;
// ISR cost tracking. The ISR preempts every task including the model, so its
// CPU share comes straight off the 2500 ms window chewallow has to fit in.
// Sample rate drives this directly -- 32 kHz means ~37% more samples through
// the DC blocker, biquad and envelope follower than 23,438 Hz does.
static volatile uint32_t g_isrCycles    = 0;   // accumulated over one second
static volatile uint32_t g_isrCallCount = 0;
static volatile uint32_t g_isrCyclesRpt = 0;   // latched for the printf
static volatile uint32_t g_isrCallsRpt  = 0;
static volatile uint32_t g_isrSamplesRpt = 0;
static volatile uint32_t g_isrSamples   = 0;
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
// int16-scaled (~+/-32K) so no further shift is needed before resampling.
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

#if !defined(KWS_DISABLE) && (WW_TEST_MODE || CHEW_TEST_MODE)
//*****************************************************************************
// Per-window z-score, matching the numpy building script:
//
//     temp_mean = np.mean(feat);  feat -= temp_mean
//     temp_std  = np.std(feat);   feat /= temp_std
//
// np.std defaults to the population form (ddof=0), so divide by n, not n-1.
// A zero standard deviation means a completely flat window; the Python side
// discards those, so zeroing is the closest equivalent.
//
// This is not optional. The model was trained entirely on normalized windows
// and produces nonsense on raw MFCCs.
//*****************************************************************************
static void model_normalize_window(float *feat, uint32_t n)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i < n; i++) sum += feat[i];
    float mean = sum / (float)n;

    float var = 0.0f;
    for (uint32_t i = 0; i < n; i++)
    {
        float d = feat[i] - mean;
        var += d * d;
    }
    float sd = sqrtf(var / (float)n);

    if (sd > 0.0f)
    {
        for (uint32_t i = 0; i < n; i++) feat[i] = (feat[i] - mean) / sd;
    }
    else
    {
        for (uint32_t i = 0; i < n; i++) feat[i] = 0.0f;
    }
}
#endif

#if !defined(KWS_DISABLE) && WW_TEST_MODE && WW_SELFTEST
//*****************************************************************************
// One-shot front-end check against an embedded wav.
//
// Runs the identical audio through the firmware MFCC path that Python runs
// through python_speech_features, so the two can be compared number by
// number. No microphone, no gain, no timing -- if these disagree the front
// end is at fault; if they agree and live audio still misbehaves, the problem
// is in the capture chain.
//
// Frame k analyses samples [k*320, k*320+480), matching psf with
// winlen=0.030 winstep=0.020 on a 24000-sample clip: exactly 75 frames.
//*****************************************************************************
static void ww_selftest(void)
{
    am_util_stdio_printf("\n===== WW MFCC SELF-TEST =====\n");
    am_util_stdio_printf("audio: %d samples\n", WW_TEST_AUDIO_SAMPLES);

    // Peak and RMS, so the embedded array can be checked against the wav.
    int32_t peak = 0;
    int64_t sumsq = 0;
    for (int i = 0; i < WW_TEST_AUDIO_SAMPLES; i++)
    {
        int32_t a = g_ww_test_audio[i] < 0 ? -g_ww_test_audio[i]
                                           : g_ww_test_audio[i];
        if (a > peak) peak = a;
        sumsq += (int64_t)g_ww_test_audio[i] * g_ww_test_audio[i];
    }
    am_util_stdio_printf("peak=%ld rms=%ld\n",
                         (long)peak,
                         (long)sqrtf((float)sumsq / WW_TEST_AUDIO_SAMPLES));

    // Preemphasize the WHOLE clip first, exactly as psf does, then slice.
    // Doing it per frame leaves each frame's first sample un-differenced,
    // which acts like an impulse and smears energy across every coefficient.
    int16_t prev = 0;
    for (int i = 0; i < WW_TEST_AUDIO_SAMPLES; i++)
    {
        int32_t y = (int32_t)g_ww_test_audio[i]
                  - (((int32_t)prev * WW_PREEMPH_Q15) >> 15);
        if (y >  32767) y =  32767;
        if (y < -32768) y = -32768;
        g_wwPreBuf[i] = (int16_t)y;
        prev = g_ww_test_audio[i];
    }

    for (uint32_t k = 0; k < WW_NUM_FRAMES; k++)
    {
        uint32_t start = k * WW_MFCC_HOP_SAMP;
if (start + WW_MFCC_WINLEN_SAMP > WW_TEST_AUDIO_SAMPLES)
        {
            // psf zero-pads the final partial frame.
            int16_t padded[WW_MFCC_WINLEN_SAMP];
            memset(padded, 0, sizeof(padded));
            uint32_t avail = (start < WW_TEST_AUDIO_SAMPLES)
                             ? (WW_TEST_AUDIO_SAMPLES - start) : 0;
            if (avail > WW_MFCC_WINLEN_SAMP) avail = WW_MFCC_WINLEN_SAMP;
            memcpy(padded, &g_wwPreBuf[start], avail * sizeof(int16_t));
            ns_mfcc_compute(&g_wwMfccCfg, padded,
                            &g_wwFeatures[k * WW_MFCC_COEFFS]);
        }
        else
        {
            ns_mfcc_compute(&g_wwMfccCfg, &g_wwPreBuf[start],
                            &g_wwFeatures[k * WW_MFCC_COEFFS]);
        }

        // Undo the num_dec_bits scaling applied before ns_mfcc's round().
        for (uint32_t c = 0; c < WW_MFCC_COEFFS; c++)
            g_wwFeatures[k * WW_MFCC_COEFFS + c] *= WW_MFCC_DEC_SCALE;
    }

    // RAW coefficients, before normalization -- these compare directly
    // against python_speech_features' mfcc() output. Scaled by 1000 because
    // am_util_stdio_printf has no float support.
    const uint32_t dump_frames[] = {0, 1, 2, 37, 74};
    for (uint32_t d = 0; d < sizeof(dump_frames) / sizeof(dump_frames[0]); d++)
    {
        uint32_t k = dump_frames[d];
        am_util_stdio_printf("RAW f%02lu:", (unsigned long)k);
        for (uint32_t c = 0; c < WW_MFCC_COEFFS; c++)
            am_util_stdio_printf(" %d",
                (int)(g_wwFeatures[k * WW_MFCC_COEFFS + c] * 1000.0f));
        am_util_stdio_printf("\n");
    }

    // Whole-array statistics: a fast way to spot a scale or offset error
    // without reading 1800 numbers.
    float mn = g_wwFeatures[0], mx = g_wwFeatures[0], sum = 0.0f;
    for (uint32_t i = 0; i < WW_NUM_FRAMES * WW_MFCC_COEFFS; i++)
    {
        float v = g_wwFeatures[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
    }
    float mean = sum / (WW_NUM_FRAMES * WW_MFCC_COEFFS);
    float var = 0.0f;
    for (uint32_t i = 0; i < WW_NUM_FRAMES * WW_MFCC_COEFFS; i++)
    {
        float d2 = g_wwFeatures[i] - mean;
        var += d2 * d2;
    }
    am_util_stdio_printf("RAW stats: min=%d max=%d mean=%d std=%d (x1000)\n",
                         (int)(mn * 1000.0f), (int)(mx * 1000.0f),
                         (int)(mean * 1000.0f),
                         (int)(sqrtf(var / (WW_NUM_FRAMES * WW_MFCC_COEFFS))
                               * 1000.0f));

    // Normalize and run, as the live path does.
    model_normalize_window(g_wwFeatures, WW_NUM_FRAMES * WW_MFCC_COEFFS);

    am_util_stdio_printf("NORM f00:");
    for (uint32_t c = 0; c < WW_MFCC_COEFFS; c++)
        am_util_stdio_printf(" %d", (int)(g_wwFeatures[c] * 1000.0f));
    am_util_stdio_printf("\n");

    float probs[WW_NUM_CLASSES] = {0};
    TickType_t t0 = xTaskGetTickCount();
    int best = ww_run(g_wwFeatures, probs);
    uint32_t ms = (uint32_t)(xTaskGetTickCount() - t0);

    am_util_stdio_printf("RESULT: %s  n=%d iam=%d note=%d life=%d  (%lu ms)\n",
                         (best >= 0) ? ww_label_names[best] : "error",
                         (int)(probs[WW_IDX_NOTHING] * 100.0f),
                         (int)(probs[WW_IDX_IAM] * 100.0f),
                         (int)(probs[WW_IDX_NOTE] * 100.0f),
                         (int)(probs[WW_IDX_LIFEIKO] * 100.0f),
                         (unsigned long)ms);
    am_util_stdio_printf("===== END SELF-TEST =====\n\n");

    // Leave no state behind for the live path.
    memset(g_wwFeatures, 0, sizeof(g_wwFeatures));
    memset(g_wwWindow, 0, sizeof(g_wwWindow));
    g_wwFrameCount  = 0;
    g_wwPrimed      = false;
    g_wwPreemphPrev = 0;
}
#endif


//*****************************************************************************
// ISR -- AUDADC0
// Capture chain: extract -> DC block -> LPF -> envelope -> decimate -> gate
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

    uint32_t cyc0 = DWT->CYCCNT;

    int32_t sumAbs = 0;
    for (uint32_t i = 0; i < AUDADC_DMA_WORDS; i++)
    {
        // 4 slots are enabled (the Ambiq reference design; DMA does not
        // complete reliably with fewer). Only the SE0 words are ours.
        if (!audadc_word_matches_src(rxBuf[i], MIC_RECORD_SOURCE)) continue;

        int32_t xo_raw = extract_record_sample(rxBuf[i], MIC_RECORD_SOURCE);

        // --- DC blocker: y[n] = x[n] - x[n-1] + R * y[n-1] ---
        int32_t xo_dc = xo_raw - g_dcPrevIn +
                        (int32_t)(((int64_t)DC_BLOCK_R_Q15 * g_dcPrevOut) >> 15);
        g_dcPrevIn  = xo_raw;
        g_dcPrevOut = xo_dc;

        // --- 7 kHz 4th-order anti-alias LPF, two cascaded biquads ---
        float xf = (float)xo_dc;

        float y1 = AA1_B0 * (xf + g_aa1X2)
                 + AA1_B1 * g_aa1X1
                 + AA1_A1_N * g_aa1Y1
                 + AA1_A2_N * g_aa1Y2;
        g_aa1X2 = g_aa1X1;  g_aa1X1 = xf;
        g_aa1Y2 = g_aa1Y1;  g_aa1Y1 = y1;

        float yf = AA2_B0 * (y1 + g_aa2X2)
                 + AA2_B1 * g_aa2X1
                 + AA2_A1_N * g_aa2Y1
                 + AA2_A2_N * g_aa2Y2;
        g_aa2X2 = g_aa2X1;  g_aa2X1 = y1;
        g_aa2Y2 = g_aa2Y1;  g_aa2Y1 = yf;

        int32_t xo = (int32_t)yf;

        // --- Gate envelope follower (asymmetric: fast attack, slow release) ---
        float abs_y = (yf >= 0.0f) ? yf : -yf;
        if (abs_y > g_gateEnv)
            g_gateEnv += GATE_ATTACK_A  * (abs_y - g_gateEnv);
        else
            g_gateEnv += GATE_RELEASE_A * (abs_y - g_gateEnv);

        // Per-second stats are taken here, PRE-gate.
        int32_t ao = (xo < 0 ? -xo : xo);
        sumAbs += ao;
        if (ao > g_secPeak) g_secPeak = ao;

        // --- Decimate 32000 -> 16000, exactly 2:1 ---
        // Keep every other sample. No phase accumulator, no interpolation,
        // no jitter. The old 23,438 -> 16,000 ratio was 1.4649, which forced
        // linear interpolation between neighbours -- a poor reconstruction
        // filter whose passband droop and imaging the 4 kHz LPF was partly
        // compensating for.
        if (g_mode == MODE_LISTENING || g_mode == MODE_RECORDING)
        {
            g_capToggle ^= 1u;
            if (g_capToggle == 0)
            {
                int32_t s16 = xo;
                if (s16 >  32767) s16 =  32767;
                if (s16 < -32768) s16 = -32768;


                if (g_mode == MODE_RECORDING && g_recPos < REC_PCM_SAMPLES)
                {
                    // --- Noise gate, recording path only, and only when
                    //     g_gateRecording is set. See the note at its
                    //     declaration: gated recordings are not usable as
                    //     training data for a model that infers ungated.
                    float gate_gain = 1.0f;
                    if (g_gateRecording)
                    {
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
                    }
                    int32_t s16g = (int32_t)((float)s16 * gate_gain);
                    if (s16g >  32767) s16g =  32767;
                    if (s16g < -32768) s16g = -32768;
                    g_pcmBuf[g_recPos++] = (int16_t)s16g;
                }
#ifndef KWS_DISABLE
                // Model feed taps in UNGATED, so quiet speech still reaches
                // the model unattenuated.
                else if (g_mode == MODE_LISTENING && g_kwsReadyBuf == 0xFF)
                {
                    int32_t kg = (int32_t)s16 * KWS_SW_GAIN;
                    if (kg >  32767) kg =  32767;
                    if (kg < -32768) kg = -32768;
                    int32_t ag = (kg < 0) ? -kg : kg;
                    if (ag > g_kwsWindowPeak) g_kwsWindowPeak = ag;
#if WW_TEST_MODE
                    if (ag > g_wwWindowPeak) g_wwWindowPeak = ag;
#elif CHEW_TEST_MODE
                    if (ag > g_chewWindowPeak) g_chewWindowPeak = ag;
#endif

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
    }
	
    g_isrCycles += (DWT->CYCCNT - cyc0);
    g_isrCallCount++;
    g_isrSamples += AUDADC_MONO_SAMPLES;

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
        g_isrCyclesRpt  = g_isrCycles;
        g_isrCallsRpt   = g_isrCallCount;
        g_isrSamplesRpt = g_isrSamples;
        g_isrCycles     = 0;
        g_isrCallCount  = 0;
        g_isrSamples    = 0;
        g_secPeakLast = g_secPeak;
        g_secSumAbs   = 0;
        g_secPeak     = 0;
        g_dmaBufCount = 0;
        g_printReady  = 1;
    }
}

//*****************************************************************************
// ISR -- I2S0 (TX-only: playback / test tone / silence)
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

            // Linear interpolation. The int64 cast is REQUIRED: (s1-s0) can
            // reach +/-65534 and phase reaches 65535, so the product overflows
            // int32 at high signal levels and corrupts samples.
            int32_t out = (int32_t)s0 +
                          (int32_t)(((int64_t)(s1 - s0) * (int64_t)g_playPhaseQ16) >> 16);

            // Reconstruction LPF (see note at the g_plpfX1 declaration).
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

    // The SD control pin needs its OWN pin config with the plain GPIO
    // function. Reusing outCfg here silently gave it an I2S alt-function, so
    // the pin never drove a real logic level (measured 0.27 V in both states).
    am_hal_gpio_pincfg_t ampSdCfg = { 0 };
    ampSdCfg.GP.cfg_b.eGPOutCfg = 1;                  // push-pull output
    ampSdCfg.GP.cfg_b.ePullup   = 0;                  // no pullup
    ampSdCfg.GP.cfg_b.uFuncSel  = AM_HAL_PIN_8_GPIO;  // = 3, plain GPIO
    am_hal_gpio_pinconfig(AMP_SD_PIN, ampSdCfg);
    am_hal_gpio_state_write(AMP_SD_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);  // amp off

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
    // Power up AUDADC analog front-end. micbias must be enabled even though the
    // mic is externally powered; it also powers internal converter biasing and
    // without it the AUDADC produces zero output.
    am_hal_audadc_refgen_powerup();
    am_hal_audadc_pga_powerup(0);
    am_hal_audadc_pga_powerup(1);
    am_hal_audadc_pga_powerup(2);
    am_hal_audadc_pga_powerup(3);

    // NOTE: these four calls are INERT. am_hal_audadc_internal_pga_config()
    // below overwrites the GAIN register they write. Kept only to match the
    // Ambiq reference sequence; see the PREAMP_GAIN_DB note at the top.
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

    // 48 MHz / 8 / 375 = 32,000.0 Hz exactly. Was DIV32 with count 63,
    // giving 23,437.5 Hz and a non-integer 1.4649 ratio to 16 kHz -- which
    // forced the phase-accumulator resampler. 32 kHz is exactly 2:1, so
    // decimation becomes "keep every other sample".
    am_hal_audadc_irtt_config_t irttCfg =
    {
        .bIrttEnable      = true,
        .eClkDiv          = AM_HAL_AUDADC_RPTT_CLK_DIV4,
        .ui32IrttCountMax = 374,
    };

    am_hal_audadc_initialize(0, &g_pAUDADCHandle);
    am_hal_audadc_power_control(g_pAUDADCHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_audadc_configure(g_pAUDADCHandle, &audadcCfg);
    am_hal_audadc_configure_irtt(g_pAUDADCHandle, &irttCfg);
    am_hal_audadc_enable(g_pAUDADCHandle);
    am_hal_audadc_irtt_enable(g_pAUDADCHandle);

    // All 4 slots enabled, 12-bit. Audio is on SE0 (channel A LG); the ISR
    // filters the other slots out. Do NOT disable SE1-SE3: DMA then stops
    // completing entirely and the AUDADC ISR never fires (verified).
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

    // THIS is the gain that takes effect. The ui32LGA field is 7 bits (0-127)
    // and Ambiq does not document its units, so the dB relationship is
    // empirical -- see the measured sweep at the top of this file.
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

// Enable the MAX98357A only during playback/tone; keep it shut down during
// listening/recording/idle so its Class-D switching cannot couple into the
// mic. Verified: with the amp shut down, recordings match the noise floor
// measured with the amp physically removed. Called once per task loop pass
// rather than at each mode transition, so it cannot get out of sync.
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
    // Defer model init briefly so BLE can finish reset/advertising bring-up.
    am_util_stdio_printf("[mic] waiting 3s before model init\r\n");
    vTaskDelay(pdMS_TO_TICKS(3000));

#if WW_TEST_MODE
    //-------------------------------------------------------------------------
    // Wake word test mode. KWS and chewallow are skipped entirely, which
    // leaves their arenas unallocated -- chewallow's 68 KB in TCM and KWS's
    // 48 KB plus a 64 KB MFCC arena in .sram_bss.
    //-------------------------------------------------------------------------
    am_util_stdio_printf("[mic] >ww_init\r\n");
    ww_init();
    am_util_stdio_printf("[mic] <ww_init\r\n");

    g_wwMfccCfg.api              = &ns_mfcc_V1_0_0;
    g_wwMfccCfg.arena            = g_wwMfccArena;
    g_wwMfccCfg.sample_frequency = REC_PCM_RATE;          // 16000
    g_wwMfccCfg.num_fbank_bins   = WW_MFCC_FBANK_BINS;    // 40
    g_wwMfccCfg.low_freq         = WW_MFCC_LOW_FREQ;      // 20
    g_wwMfccCfg.high_freq        = WW_MFCC_HIGH_FREQ;     // 4000
    g_wwMfccCfg.num_frames       = WW_NUM_FRAMES;         // 75
    g_wwMfccCfg.num_coeffs       = WW_MFCC_COEFFS;        // 24
    g_wwMfccCfg.num_dec_bits     = WW_MFCC_DEC_BITS;      // see the note above
    g_wwMfccCfg.frame_shift_ms   = 20;
    g_wwMfccCfg.frame_len_ms     = 30;
    g_wwMfccCfg.frame_len        = WW_MFCC_WINLEN_SAMP;   // 480, NOT 320
    g_wwMfccCfg.frame_len_pow2   = WW_MFCC_FRAME_POW2;    // 512

    am_util_stdio_printf("[mic] >ns_mfcc_init (wake word)\r\n");
    ns_mfcc_init(&g_wwMfccCfg);
    am_util_stdio_printf("[mic] <ns_mfcc_init  %d frames x %d coeffs, "
                         "win %d hop %d, %d-%d Hz, inference every %d frames\r\n",
                         WW_NUM_FRAMES, WW_MFCC_COEFFS,
                         WW_MFCC_WINLEN_SAMP, WW_MFCC_HOP_SAMP,
                         WW_MFCC_LOW_FREQ, WW_MFCC_HIGH_FREQ, WW_HOP_FRAMES);

    memset(g_wwWindow, 0, sizeof(g_wwWindow));
    memset(g_wwFeatures, 0, sizeof(g_wwFeatures));
    g_wwFrameCount = 0;

#if WW_SELFTEST
    ww_selftest();
#endif

#elif CHEW_TEST_MODE
    //-------------------------------------------------------------------------
    // Chewallow test mode. KWS and the wake word are skipped, so their arenas
    // cost nothing.
    //-------------------------------------------------------------------------
    am_util_stdio_printf("[mic] >chewallow_init\r\n");
    chewallow_init();
    am_util_stdio_printf("[mic] <chewallow_init\r\n");

    g_chewMfccCfg.api              = &ns_mfcc_V1_0_0;
    g_chewMfccCfg.arena            = g_chewMfccArena;
    g_chewMfccCfg.sample_frequency = REC_PCM_RATE;            // 16000
    g_chewMfccCfg.num_fbank_bins   = CHEW_MFCC_FBANK_BINS;    // 32
    g_chewMfccCfg.low_freq         = CHEW_MFCC_LOW_FREQ;      // 20
    g_chewMfccCfg.high_freq        = CHEW_MFCC_HIGH_FREQ;     // 8000
    g_chewMfccCfg.num_frames       = CHEWALLOW_NUM_FRAMES;    // 78
    g_chewMfccCfg.num_coeffs       = CHEW_MFCC_COEFFS;        // 24
    g_chewMfccCfg.num_dec_bits     = CHEW_MFCC_DEC_BITS;      // 10, see note
    g_chewMfccCfg.frame_shift_ms   = 32;
    g_chewMfccCfg.frame_len_ms     = 64;
    g_chewMfccCfg.frame_len        = CHEW_MFCC_WINLEN_SAMP;   // 1024
    g_chewMfccCfg.frame_len_pow2   = CHEW_MFCC_FRAME_POW2;    // 1024

    am_util_stdio_printf("[mic] >ns_mfcc_init (chewallow)\r\n");
    ns_mfcc_init(&g_chewMfccCfg);
    am_util_stdio_printf("[mic] <ns_mfcc_init  %d frames x %d coeffs, "
                         "win %d hop %d, %d-%d Hz, inference every %d frames\r\n",
                         CHEWALLOW_NUM_FRAMES, CHEW_MFCC_COEFFS,
                         CHEW_MFCC_WINLEN_SAMP, CHEW_MFCC_HOP_SAMP,
                         CHEW_MFCC_LOW_FREQ, CHEW_MFCC_HIGH_FREQ,
                         CHEW_HOP_FRAMES);

    memset(g_chewWindow, 0, sizeof(g_chewWindow));
    memset(g_chewFeatures, 0, sizeof(g_chewFeatures));
    g_chewFrameCount  = 0;
    g_chewPrimed      = false;
    g_chewPreemphPrev = 0;

#else
    am_util_stdio_printf("[mic] >kws_init\r\n");
    kws_init();
    am_util_stdio_printf("[mic] <kws_init\r\n");

    am_util_stdio_printf("[mic] >chewallow_init\r\n");
    chewallow_init();
    am_util_stdio_printf("[mic] <chewallow_init\r\n");

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
    am_util_stdio_printf("[mic] >ns_mfcc_init (kws)\r\n");
    ns_mfcc_init(&g_mfccCfg);
    am_util_stdio_printf("[mic] <ns_mfcc_init\r\n");
#endif

    am_util_stdio_printf("[mic] heap: free=%u  min-ever-free=%u  of %u\r\n",
                         (unsigned)xPortGetFreeHeapSize(),
                         (unsigned)xPortGetMinimumEverFreeHeapSize(),
                         (unsigned)configTOTAL_HEAP_SIZE);
#endif

    am_util_stdio_printf("[mic] >mic_audadc_init\r\n");
    mic_audadc_init(dmaPtrA, dmaPtrB);
    am_util_stdio_printf("[mic] <mic_audadc_init\r\n");

    am_util_stdio_printf("[mic] >mic_i2s_tx_init\r\n");
    mic_i2s_tx_init(txPtrA, txPtrB);
    am_util_stdio_printf("[mic] <mic_i2s_tx_init\r\n");

    // DWT cycle counter, for measuring ISR cost. Free-running at the CPU
    // clock; reading it is a single load.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    am_util_stdio_printf("[mic] DWT cycle counter enabled\r\n");

    am_hal_interrupt_master_enable();

    // Kick off AUDADC repeating scan; IRTT takes over after first trigger.
    am_hal_audadc_sw_trigger(g_pAUDADCHandle);
    am_util_stdio_printf("[mic] AUDADC capture started\r\n");

    // I2S TX runs continuously; outputs silence until playback or tone.
    am_hal_i2s_dma_transfer_start(g_pI2SHandle, &g_sI2SConfig);
    am_util_stdio_printf("[mic] I2S TX DMA started\r\n");

#ifndef KWS_DISABLE
#if CHEW_TEST_MODE
    am_util_stdio_printf("[mic] Chewallow test mode. %d frames x %d coeffs. "
                         "Short press=record, long press(2s)=test tone.\n",
                         CHEWALLOW_NUM_FRAMES, CHEW_MFCC_COEFFS);
#elif WW_TEST_MODE
    am_util_stdio_printf("[mic] Wake word test mode. Listening for %s/%s/%s. "
                         "Short press=record, long press(2s)=test tone.\n",
                         ww_label_names[WW_IDX_IAM],
                         ww_label_names[WW_IDX_NOTE],
                         ww_label_names[WW_IDX_LIFEIKO]);
#else
    am_util_stdio_printf("[mic] Listening for \"%s\". Short press=record, long press(2s)=test tone.\n",
                         kws_label_names[KWS_TRIGGER_IDX]);
#endif
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
                    g_capToggle = 0;
                    g_opusLen   = 0;
                    g_mode      = MODE_RECORDING;
                    am_util_stdio_printf("[mic] Recording %d sec...\n", REC_SECONDS);
                }
            }
            btnHoldCnt = 0;
        }
        btnPrev = btnDown;

#ifndef KWS_DISABLE
#if WW_TEST_MODE
        //---------------------------------------------------------------------
        // Wake word: one 20 ms hop per ISR handoff, inference every
        // WW_HOP_FRAMES frames.
        //---------------------------------------------------------------------
        uint32_t readyBuf = g_kwsReadyBuf;
        if (readyBuf != 0xFF && g_mode == MODE_LISTENING)
        {
            // Shift out the older chunk, keeping the newer one at the front.
            memmove(g_wwWindow, &g_wwWindow[WW_MFCC_HOP_SAMP],
                    WW_MFCC_HOP_SAMP * sizeof(int16_t));

            // Preemphasize the new chunk as it is copied in, carrying the
            // previous chunk's last RAW sample so the difference is
            // continuous across the boundary. See WW_PREEMPH_Q15.
            for (uint32_t i = 0; i < WW_MFCC_HOP_SAMP; i++)
            {
                int16_t raw = g_kwsBuf[readyBuf][i];
                int32_t y = (int32_t)raw
                          - (((int32_t)g_wwPreemphPrev * WW_PREEMPH_Q15) >> 15);
                if (y >  32767) y =  32767;
                if (y < -32768) y = -32768;
                g_wwWindow[WW_MFCC_HOP_SAMP + i] = (int16_t)y;
                g_wwPreemphPrev = raw;
            }
            g_kwsReadyBuf = 0xFF;

            if (!g_wwPrimed)
            {
                // Only one chunk so far, so there is no complete 480-sample
                // window yet.
                g_wwPrimed = true;
                continue;
            }

            // ns_mfcc_compute reads the FIRST 480 samples of the 640-sample
            // buffer, which is Python's frame k. Passing the most recent 480
            // instead would sit half a hop later than training.
            TickType_t m0 = xTaskGetTickCount();
            ns_mfcc_compute(&g_wwMfccCfg, g_wwWindow,
                            &g_wwFeatures[g_wwFrameCount * WW_MFCC_COEFFS]);
            // Undo the num_dec_bits scaling applied before ns_mfcc's round().
            for (uint32_t c = 0; c < WW_MFCC_COEFFS; c++)
                g_wwFeatures[g_wwFrameCount * WW_MFCC_COEFFS + c]
                    *= WW_MFCC_DEC_SCALE;
            g_wwMfccMsAccum += (uint32_t)(xTaskGetTickCount() - m0);
            g_wwFrameCount++;

            if (g_wwFrameCount >= WW_NUM_FRAMES)
            {
                int32_t winPeak = g_wwWindowPeak;
                g_wwWindowPeak = 0;

                if (winPeak >= WW_PEAK_GATE)
                {
                    // Normalize the whole 75x24 window, exactly as the numpy
                    // building script does before training.
                    model_normalize_window(g_wwFeatures,
                                        WW_NUM_FRAMES * WW_MFCC_COEFFS);

                    float probs[WW_NUM_CLASSES] = {0};
                    TickType_t t0 = xTaskGetTickCount();
                    int best = ww_run(g_wwFeatures, probs);
                    uint32_t inferMs = (uint32_t)(xTaskGetTickCount() - t0);
                    g_wwInferCount++;

                    bool detected = (best >= 0 && best != WW_IDX_NOTHING
                                     && probs[best] >= WW_REPORT_THRESHOLD);

                    if (detected)
                    {
                        am_util_stdio_printf(
                            "[ww] *** %s %d%%  (n=%d iam=%d note=%d life=%d) "
                            "peak=%ld mfcc=%lu ms infer=%lu ms\n",
                            ww_label_names[best], (int)(probs[best] * 100.0f),
                            (int)(probs[WW_IDX_NOTHING] * 100.0f),
                            (int)(probs[WW_IDX_IAM] * 100.0f),
                            (int)(probs[WW_IDX_NOTE] * 100.0f),
                            (int)(probs[WW_IDX_LIFEIKO] * 100.0f),
                            (long)winPeak,
                            (unsigned long)g_wwMfccMsAccum,
                            (unsigned long)inferMs);
                    }
#if WW_VERBOSE
                    else
                    {
                        int b = (best < 0) ? WW_IDX_NOTHING : best;
                        am_util_stdio_printf(
                            "[ww] %s %d%%  peak=%ld mfcc=%lu ms infer=%lu ms\n",
                            ww_label_names[b], (int)(probs[b] * 100.0f),
                            (long)winPeak,
                            (unsigned long)g_wwMfccMsAccum,
                            (unsigned long)inferMs);
                    }
#endif
                }

                g_wwMfccMsAccum = 0;

                // Slide the feature buffer instead of resetting, so windows
                // overlap. A phrase that straddles one window boundary then
                // still lands inside the next one.
                memmove(g_wwFeatures,
                        &g_wwFeatures[WW_HOP_FRAMES * WW_MFCC_COEFFS],
                        (WW_NUM_FRAMES - WW_HOP_FRAMES) * WW_MFCC_COEFFS
                            * sizeof(float));
                g_wwFrameCount = WW_NUM_FRAMES - WW_HOP_FRAMES;
            }
        }
#elif CHEW_TEST_MODE
        //---------------------------------------------------------------------
        // Chewallow: one 32 ms hop per ISR handoff, inference every
        // CHEW_HOP_FRAMES frames.
        //
        // winlen is exactly 2 x winstep, so the 1024-sample buffer IS the
        // analysis window -- shift out the old 512-sample chunk, append the
        // new one, compute. No partial read as the wake word needed.
        //---------------------------------------------------------------------
        uint32_t readyBuf = g_kwsReadyBuf;
        if (readyBuf != 0xFF && g_mode == MODE_LISTENING)
        {
            memmove(g_chewWindow, &g_chewWindow[CHEW_MFCC_HOP_SAMP],
                    CHEW_MFCC_HOP_SAMP * sizeof(int16_t));

            // Preemphasize as we copy, carrying the previous chunk's last RAW
            // sample so the difference is continuous across the boundary.
            // python_speech_features preemphasizes before framing, so each
            // frame's first sample depends on the previous frame's last.
            for (uint32_t i = 0; i < CHEW_MFCC_HOP_SAMP; i++)
            {
                int16_t raw = g_kwsBuf[readyBuf][i];
                int32_t y = (int32_t)raw
                          - (((int32_t)g_chewPreemphPrev * CHEW_PREEMPH_Q15) >> 15);
                if (y >  32767) y =  32767;
                if (y < -32768) y = -32768;
                g_chewWindow[CHEW_MFCC_HOP_SAMP + i] = (int16_t)y;
                g_chewPreemphPrev = raw;
            }
            g_kwsReadyBuf = 0xFF;

            if (!g_chewPrimed)
            {
                // One chunk so far -- no complete 1024-sample window yet.
                g_chewPrimed = true;
                continue;
            }

            TickType_t m0 = xTaskGetTickCount();
            ns_mfcc_compute(&g_chewMfccCfg, g_chewWindow,
                            &g_chewFeatures[g_chewFrameCount * CHEW_MFCC_COEFFS]);
            // Undo the num_dec_bits scaling applied before ns_mfcc's round().
            for (uint32_t c = 0; c < CHEW_MFCC_COEFFS; c++)
                g_chewFeatures[g_chewFrameCount * CHEW_MFCC_COEFFS + c]
                    *= CHEW_MFCC_DEC_SCALE;
            g_chewMfccMsAccum += (uint32_t)(xTaskGetTickCount() - m0);
            g_chewFrameCount++;

            if (g_chewFrameCount >= CHEWALLOW_NUM_FRAMES)
            {
                int32_t winPeak = g_chewWindowPeak;
                g_chewWindowPeak = 0;

                if (winPeak >= CHEW_PEAK_GATE)
                {
                    // Normalize the whole 78x24 window, exactly as the numpy
                    // building script does before training.
                    model_normalize_window(g_chewFeatures,
                        CHEWALLOW_NUM_FRAMES * CHEW_MFCC_COEFFS);

                    float chewProb = 0.0f;
                    TickType_t t0 = xTaskGetTickCount();
                    int best = chewallow_run(g_chewFeatures, &chewProb);
                    uint32_t inferMs = (uint32_t)(xTaskGetTickCount() - t0);
                    g_chewInferCount++;

                    // Threshold on the probability directly. Requiring
                    // best == CHEWALLOW_IDX_CHEWALLOW would pin the effective
                    // threshold at 0.5, since with two classes argmax and
                    // p >= 0.5 are the same test -- which makes any
                    // CHEW_REPORT_THRESHOLD below 0.5 unreachable.
                    if (chewProb >= CHEW_REPORT_THRESHOLD)
                    {
                        am_util_stdio_printf(
                            "[chew] *** CHEWALLOW %d%%  peak=%ld "
                            "mfcc=%lu ms infer=%lu ms\n",
                            (int)(chewProb * 100.0f), (long)winPeak,
                            (unsigned long)g_chewMfccMsAccum,
                            (unsigned long)inferMs);
                    }
#if CHEW_VERBOSE
                    else
                    {
                        am_util_stdio_printf(
                            "[chew] nothing (p=%d%%)  peak=%ld "
                            "mfcc=%lu ms infer=%lu ms\n",
                            (int)(chewProb * 100.0f), (long)winPeak,
                            (unsigned long)g_chewMfccMsAccum,
                            (unsigned long)inferMs);
                    }
#endif
                }

                g_chewMfccMsAccum = 0;

                // Slide the feature buffer rather than resetting, so windows
                // overlap and a chewing burst near a boundary still lands
                // inside the next one.
                memmove(g_chewFeatures,
                        &g_chewFeatures[CHEW_HOP_FRAMES * CHEW_MFCC_COEFFS],
                        (CHEWALLOW_NUM_FRAMES - CHEW_HOP_FRAMES)
                            * CHEW_MFCC_COEFFS * sizeof(float));
                g_chewFrameCount = CHEWALLOW_NUM_FRAMES - CHEW_HOP_FRAMES;
            }
        }
#else
        // ---- KWS: process one 20 ms frame when the ISR hands one over ----
        uint32_t readyBuf = g_kwsReadyBuf;
        if (readyBuf != 0xFF && g_mode == MODE_LISTENING)
        {
            float confidence = 0.0f;
            static uint32_t mfccMs = 0;
            if (g_mfccFrameCount == 0) { mfccMs = 0; }

            TickType_t m0 = xTaskGetTickCount();
            ns_mfcc_compute(&g_mfccCfg,
                            (const int16_t *)g_kwsBuf[readyBuf],
                            &g_mfccFeatures[g_mfccFrameCount * KWS_MFCC_COEFFS]);
            mfccMs += (uint32_t)(xTaskGetTickCount() - m0);
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
                    TickType_t k0 = xTaskGetTickCount();
                    keyword = kws_run_top2(g_mfccFeatures,
                                           &keyword, &confidence,
                                           &top2_idx, &top2_conf);
                    TickType_t k1 = xTaskGetTickCount();
                    am_util_stdio_printf("[kws] mfcc(%d frames)=%lu ms  inference=%lu ms\n",
                                         KWS_NUM_FRAMES,
                                         (unsigned long)mfccMs,
                                         (unsigned long)(k1 - k0));
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
                    g_capToggle = 0;
                    g_opusLen  = 0;
                    g_mode     = MODE_RECORDING;
                }
            }
        }
#endif
#endif

        // ---- ISR notifications ----
        if (g_recDone)
        {
            g_recDone = false;

            // Peak of the recorded (gated) PCM.
            int32_t peakAbs = 0;
            for (uint32_t k = 0; k < g_recLen; k++)
            {
                int32_t a = (g_pcmBuf[k] < 0) ? -g_pcmBuf[k] : g_pcmBuf[k];
                if (a > peakAbs) peakAbs = a;
            }
            am_util_stdio_printf("[mic] PCM @16kHz: %lu samples, peak=%ld\n",
                                 (unsigned long)g_recLen, (long)peakAbs);

            // RMS over the whole recording, including start/stop transients.
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

            // RMS with the first and last second trimmed -- the transients at
            // each end skew the figure badly. This is the number to compare
            // across runs. NOTE: measured AFTER the noise gate, so it reflects
            // gate tuning, not the raw noise floor. For hardware comparisons
            // use the pre-gate "avg" from the per-second stats instead.
            {
                uint32_t trim = REC_PCM_RATE;          // 1 second of samples
                if (g_recLen > (2 * trim))
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

            // Encode to Opus (24 kbit/s CBR, 20 ms frames, 16 kHz mono)
            uint32_t totalFrames = g_recLen / OPUS_FRAME_SAMPLES;
            if (totalFrames > OPUS_NUM_FRAMES) totalFrames = OPUS_NUM_FRAMES;

            am_util_stdio_printf("[mic] Encoding %lu Opus frames...\n",
                                 (unsigned long)totalFrames);
            TickType_t encStartTick = xTaskGetTickCount();

#if USE_OPUS14
            static OpusEncoder *g_enc = NULL;
            if (g_enc == NULL)
            {
                int encSize = opus_encoder_get_size(1);
                am_util_stdio_printf("[mic] opus encoder size: %d bytes (have %d)\n",
                                     encSize, (int)sizeof(g_encMem));
                if (encSize > (int)sizeof(g_encMem)) {
                    am_util_stdio_printf("[mic] ERROR: encoder buffer too small\n");
                    return;
                }
                g_enc = (OpusEncoder *)g_encMem;
                int err = opus_encoder_init(g_enc, 16000, 1, OPUS_APPLICATION_VOIP);
                if (err != OPUS_OK) {
                    am_util_stdio_printf("[mic] opus_encoder_init failed: %d\n", err);
                    return;
                }
                opus_encoder_ctl(g_enc, OPUS_SET_BITRATE(24000));
                opus_encoder_ctl(g_enc, OPUS_SET_VBR(0));
                opus_encoder_ctl(g_enc, OPUS_SET_VBR_CONSTRAINT(0));
                opus_encoder_ctl(g_enc, OPUS_SET_COMPLEXITY(0));
                opus_encoder_ctl(g_enc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
                opus_encoder_ctl(g_enc, OPUS_SET_MAX_BANDWIDTH(OPUS_BANDWIDTH_WIDEBAND));
                opus_encoder_ctl(g_enc, OPUS_SET_FORCE_MODE(MODE_CELT_ONLY));
            }
#else
            audio_enc_init(0);
#endif

            TickType_t initDoneTick = xTaskGetTickCount();

            uint32_t opusBytes = 0;
            for (uint32_t f = 0; f < totalFrames; f++)
            {
#if USE_OPUS14
                int n = opus_encode(g_enc,
                            (const opus_int16 *)&g_pcmBuf[f * OPUS_FRAME_SAMPLES],
                            OPUS_FRAME_SAMPLES,
                            &g_opusBuf[opusBytes],
                            OPUS_BUF_BYTES - opusBytes);
#else
                int n = audio_enc_encode_frame(
                            (short *)&g_pcmBuf[f * OPUS_FRAME_SAMPLES],
                            OPUS_FRAME_SAMPLES,
                            &g_opusBuf[opusBytes]);
#endif
                if (f < 10 || n != OPUS_FRAME_BYTES)
                    am_util_stdio_printf("[mic] frame %lu: n=%d\n",
                                         (unsigned long)f, n);
                if (n <= 0 || opusBytes + n > OPUS_BUF_BYTES) break;
                opusBytes += (uint32_t)n;
            }

            TickType_t encDoneTick = xTaskGetTickCount();
            g_opusLen = opusBytes;

            uint32_t initMs   = (uint32_t)(initDoneTick - encStartTick);
            uint32_t encodeMs = (uint32_t)(encDoneTick - initDoneTick);
            uint32_t totalMs  = (uint32_t)(encDoneTick - encStartTick);

            am_util_stdio_printf("[mic] Opus encode done: %lu bytes (~%lu kbit/s)\n",
                                 (unsigned long)opusBytes,
                                 (unsigned long)((opusBytes * 8) / REC_SECONDS / 1000));

            am_util_stdio_printf("[mic] Encode timing: init=%lu ms, encode=%lu ms, total=%lu ms (%lu frames, %lu us/frame)\n",
                                 (unsigned long)initMs,
                                 (unsigned long)encodeMs,
                                 (unsigned long)totalMs,
                                 (unsigned long)totalFrames,
                                 (unsigned long)(totalFrames > 0 ? (encodeMs * 1000UL) / totalFrames : 0));

#if !defined(KWS_DISABLE) && !WW_TEST_MODE && !CHEW_TEST_MODE
            {
                float chewProb = 0.0f;
                TickType_t cw0 = xTaskGetTickCount();
                int rc = chewallow_run_dummy_timing(g_chewallow_dummy_input, &chewProb);
                TickType_t cw1 = xTaskGetTickCount();
                am_util_stdio_printf("[chewallow] rc=%d prob=%d/1000 inference=%lu ms\n",
                                     rc, (int)(chewProb * 1000.0f),
                                     (unsigned long)(cw1 - cw0));
            }
#endif

            am_util_stdio_printf("[mic] Press short=play (PCM), hold 2s=test tone.\n");
#ifndef KWS_DISABLE
            g_kwsReadyBuf    = 0xFF;
            g_kwsWritePos    = 0;
#if WW_TEST_MODE
            // Discard the partial window collected before recording started;
            // it is not contiguous with what comes next.
            g_wwFrameCount  = 0;
            g_wwWindowPeak  = 0;
            g_wwPrimed      = false;
            g_wwPreemphPrev = 0;
            memset(g_wwWindow, 0, sizeof(g_wwWindow));
#elif CHEW_TEST_MODE
            g_chewFrameCount  = 0;
            g_chewWindowPeak  = 0;
            g_chewPrimed      = false;
            g_chewPreemphPrev = 0;
            memset(g_chewWindow, 0, sizeof(g_chewWindow));
#else
            g_mfccFrameCount = 0;
#endif
#endif
            g_mode           = MIC_MODE_INITIAL;
        }
        if (g_toneDone)
        {
            g_toneDone = false;
            am_util_stdio_printf("[mic] Test tone done.\n");
#ifndef KWS_DISABLE
            g_kwsReadyBuf    = 0xFF;
            g_kwsWritePos    = 0;
#if WW_TEST_MODE
            g_wwFrameCount  = 0;
            g_wwWindowPeak  = 0;
            g_wwPrimed      = false;
            g_wwPreemphPrev = 0;
            memset(g_wwWindow, 0, sizeof(g_wwWindow));
#elif CHEW_TEST_MODE
            g_chewFrameCount  = 0;
            g_chewWindowPeak  = 0;
            g_chewPrimed      = false;
            g_chewPreemphPrev = 0;
            memset(g_chewWindow, 0, sizeof(g_chewWindow));
#else
            g_mfccFrameCount = 0;
#endif
#endif
            g_mode           = MIC_MODE_INITIAL;
        }
        if (g_playDone)
        {
            g_playDone = false;
            am_util_stdio_printf("[mic] Playback done.\n");
#ifndef KWS_DISABLE
            g_kwsReadyBuf    = 0xFF;
            g_kwsWritePos    = 0;
#if WW_TEST_MODE
            g_wwFrameCount  = 0;
            g_wwWindowPeak  = 0;
            g_wwPrimed      = false;
            g_wwPreemphPrev = 0;
            memset(g_wwWindow, 0, sizeof(g_wwWindow));
#elif CHEW_TEST_MODE
            g_chewFrameCount  = 0;
            g_chewWindowPeak  = 0;
            g_chewPrimed      = false;
            g_chewPreemphPrev = 0;
            memset(g_chewWindow, 0, sizeof(g_chewWindow));
#else
            g_mfccFrameCount = 0;
#endif
#endif
            g_mode           = MIC_MODE_INITIAL;
        }

        // ---- AUDADC liveness diagnostic every 3 seconds ----
        // If audadc_isr_count stops climbing, the AUDADC DMA has stalled.
        diagTicks++;
        if (diagTicks >= 300)
        {
            diagTicks = 0;
            if (g_micDebug)
            {
                am_util_stdio_printf("[mic] audadc_isr_count=%lu recPos=%lu\n",
                                     (unsigned long)g_audadcIsrCount,
                                     (unsigned long)g_recPos);
                am_util_stdio_printf("[mic] heap: free=%u min-ever-free=%u of %u\r\n",
                                     (unsigned)xPortGetFreeHeapSize(),
                                     (unsigned)xPortGetMinimumEverFreeHeapSize(),
                                     (unsigned)configTOTAL_HEAP_SIZE);
#if !defined(KWS_DISABLE) && WW_TEST_MODE
                am_util_stdio_printf("[ww] inferences so far: %lu\n",
                                     (unsigned long)g_wwInferCount);
#elif !defined(KWS_DISABLE) && CHEW_TEST_MODE
                am_util_stdio_printf("[chew] inferences so far: %lu\n",
                                     (unsigned long)g_chewInferCount);
#endif
            }
        }

        // ---- Per-second stats (PRE-gate: use these for noise measurements) ----
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
                // 960000 cycles = 1% of one second at 96 MHz.
                am_util_stdio_printf("[mic] isr: %lu%% cpu  %lu calls  "
                                     "%lu samples  %lu cyc/sample\n",
                                     (unsigned long)(g_isrCyclesRpt / 960000),
                                     (unsigned long)g_isrCallsRpt,
                                     (unsigned long)g_isrSamplesRpt,
                                     (unsigned long)(g_isrSamplesRpt ?
                                         g_isrCyclesRpt / g_isrSamplesRpt : 0));
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