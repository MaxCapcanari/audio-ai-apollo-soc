#ifndef KWS_INFERENCE_H
#define KWS_INFERENCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KWS_NUM_FRAMES      250
#define KWS_NUM_COEFFS      22
#define KWS_INPUT_SIZE      (KWS_NUM_FRAMES * KWS_NUM_COEFFS)  // 5500
#define KWS_NUM_CLASSES     2

// Chewallow binary classifier label order (matches kCategoryLabels in
// kws/kws_model_settings.h): 0=nothing 1=chewallow.
#define KWS_TRIGGER_IDX     1   // "chewallow"
#define KWS_THRESHOLD       0.80f

extern const char *kws_label_names[KWS_NUM_CLASSES];

// Call once at startup before any kws_run() calls.
void kws_init(void);

// Run inference on a (KWS_NUM_FRAMES x KWS_NUM_COEFFS) float MFCC feature buffer.
// Returns the index of the highest-scoring class, writes its confidence to *conf_out.
// Returns -1 on error.
int kws_run(const float *mfcc_features, float *conf_out);

// Same as kws_run but also returns the runner-up class index/confidence for
// debugging classifier bias. Pass NULLs for any pointer you don't need.
int kws_run_top2(const float *mfcc_features,
                 int *top1_idx, float *top1_conf,
                 int *top2_idx, float *top2_conf);

#ifdef __cplusplus
}
#endif

#endif // KWS_INFERENCE_H
