#ifndef WW_INFERENCE_H
#define WW_INFERENCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Model input: (1, 79, 24, 1) float32 — 79 frames of 24-coefficient MFCC,
// nfft=1024, winlen=0.064, winstep=0.032, nfilt=32, experiment=2
// (zeroth coefficient retained -> 24 cols), covering 2.5s of audio.
#define WW_NUM_FRAMES   75
#define WW_NUM_COEFFS   24
#define WW_INPUT_SIZE   (WW_NUM_FRAMES * WW_NUM_COEFFS)  // 1896

#define WW_NUM_CLASSES  4
// class_names = ['nothing', 'iam', 'note', 'lifeiko'] per notebook
#define WW_IDX_NOTHING   0
#define WW_IDX_IAM       1
#define WW_IDX_NOTE      2
#define WW_IDX_LIFEIKO   3

extern const char *ww_label_names[WW_NUM_CLASSES];

// Call once at startup before any ww_run() calls.
void ww_init(void);

// Run inference on a (ww_NUM_FRAMES x ww_NUM_COEFFS) float
// MFCC feature buffer that has ALREADY been globally z-score normalized
// (mean 0, std 1 across the whole 250x22 matrix), matching the notebook's
// preprocess_wav_to_model_input(). This function does NOT normalize for you.
// Writes class-1 ("chewallow") probability to *p_ww_out.
// Returns the index of the highest-scoring class, or -1 on error.
int ww_run(const float *mfcc_features, float *p_ww_out);

// Run inference on a raw, not-yet-converted dummy/test buffer of exactly
// WW_INPUT_SIZE floats, purely for timing Invoke() in isolation.
// Behaves identically to ww_run; provided as a clearly-named entry
// point for the dummy-input timing harness so call sites are self-documenting.
int ww_run_dummy_timing(const float *dummy_features, float *p_ww_out);

#ifdef __cplusplus
}
#endif

#endif // WW_INFERENCE_H
