#ifndef V0_1_KWS_MODEL_SETTINGS_H_
#define V0_1_KWS_MODEL_SETTINGS_H_

// Chewallow detector — input is the MFCC tensor (1, 250, 22, 1) float32.
// 250 frames @ 10 ms step = 2.5 s of audio; 22 MFCC coefficients per frame
// (the first of 23 raw coefficients is dropped, per the training pipeline).
constexpr int kNumCols     = 22;
constexpr int kNumRows     = 250;
constexpr int kNumChannels = 1;

constexpr int kKwsInputSize = kNumCols * kNumRows * kNumChannels;

constexpr size_t kCategoryCount = 2;
extern const char *kCategoryLabels[kCategoryCount];

const char *kCategoryLabels[kCategoryCount] = {
    "nothing", "chewallow"};

#endif