#ifndef CHEWALLOW_INFERENCE_H
#define CHEWALLOW_INFERENCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Model input: (1, 79, 24, 1) float32 — 79 frames of 24-coefficient MFCC,
// nfft=1024, winlen=0.064, winstep=0.032, nfilt=32, experiment=2
// (zeroth coefficient retained -> 24 cols), covering 2.5s of audio.
#define CHEWALLOW_NUM_FRAMES   79
#define CHEWALLOW_NUM_COEFFS   24
#define CHEWALLOW_INPUT_SIZE   (CHEWALLOW_NUM_FRAMES * CHEWALLOW_NUM_COEFFS)  // 1896

#define CHEWALLOW_NUM_CLASSES  2
// class_names = ['nothing', 'chewallow'] per notebook
#define CHEWALLOW_IDX_NOTHING    0
#define CHEWALLOW_IDX_CHEWALLOW  1

extern const char *chewallow_label_names[CHEWALLOW_NUM_CLASSES];

// Call once at startup before any chewallow_run() calls.
void chewallow_init(void);

// Run inference on a (CHEWALLOW_NUM_FRAMES x CHEWALLOW_NUM_COEFFS) float
// MFCC feature buffer that has ALREADY been globally z-score normalized
// (mean 0, std 1 across the whole 250x22 matrix), matching the notebook's
// preprocess_wav_to_model_input(). This function does NOT normalize for you.
// Writes class-1 ("chewallow") probability to *p_chewallow_out.
// Returns the index of the highest-scoring class, or -1 on error.
int chewallow_run(const float *mfcc_features, float *p_chewallow_out);

// Run inference on a raw, not-yet-converted dummy/test buffer of exactly
// CHEWALLOW_INPUT_SIZE floats, purely for timing Invoke() in isolation.
// Behaves identically to chewallow_run; provided as a clearly-named entry
// point for the dummy-input timing harness so call sites are self-documenting.
int chewallow_run_dummy_timing(const float *dummy_features, float *p_chewallow_out);

#ifdef __cplusplus
}
#endif

#endif // CHEWALLOW_INFERENCE_H
