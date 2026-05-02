#ifndef KWS_INFERENCE_H
#define KWS_INFERENCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KWS_NUM_FRAMES      49
#define KWS_NUM_COEFFS      10
#define KWS_INPUT_SIZE      (KWS_NUM_FRAMES * KWS_NUM_COEFFS)  // 490
#define KWS_NUM_CLASSES     12

// Standard Google Speech Commands / ML Commons label order:
// 0=silence 1=unknown 2=yes 3=no 4=up 5=down 6=left 7=right 8=on 9=off 10=stop 11=go
#define KWS_TRIGGER_IDX     2   // "yes"
#define KWS_THRESHOLD       0.75f

extern const char *kws_label_names[KWS_NUM_CLASSES];

// Call once at startup before any kws_run() calls.
void kws_init(void);

// Run inference on a (KWS_NUM_FRAMES x KWS_NUM_COEFFS) float MFCC feature buffer.
// Returns the index of the highest-scoring class, writes its confidence to *conf_out.
// Returns -1 on error.
int kws_run(const float *mfcc_features, float *conf_out);

#ifdef __cplusplus
}
#endif

#endif // KWS_INFERENCE_H
