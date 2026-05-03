#include "kws_inference.h"

#include <cstddef>

// C++14 sized-delete stub. The MicroMutableOpResolver destructor is emitted
// even though we never destroy the static instance; libsupc++ is not linked,
// so we provide a no-op. Must not abort — it can also be reached via vtable.
void operator delete(void *, std::size_t) noexcept {}

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "kws_model_data.h"
#include "kws_model_settings.h"  // kKwsInputSize, kCategoryCount (C++ constexpr only)

#include "ns_core.h"
#include "am_util.h"
#include "FreeRTOS.h"
#include "task.h"

static_assert(kKwsInputSize  == KWS_INPUT_SIZE,  "Model input size mismatch");
static_assert(kCategoryCount == KWS_NUM_CLASSES,  "Model class count mismatch");

// Measured peak: ~23 KB on our DS-CNN KWS model. 48 KB gives headroom for
// future model variants without bloating SHARED_SRAM use.
#define KWS_TENSOR_ARENA_SIZE (48 * 1024)

// Tensor arena in SHARED_SRAM — NS_SRAM_BSS expands to the right section attribute
// for apollo4p + gcc, keeping it out of the 384 KB TCM.
NS_SRAM_BSS alignas(16) static uint8_t s_arena[KWS_TENSOR_ARENA_SIZE];

static tflite::MicroMutableOpResolver<6> s_resolver;
static const tflite::Model             *s_model      = nullptr;
static tflite::MicroInterpreter        *s_interpreter = nullptr;
static TfLiteTensor                    *s_input       = nullptr;
static TfLiteTensor                    *s_output      = nullptr;

// Placement-new buffer for MicroInterpreter — avoids static-local destructor
// registration (__dso_handle / __cxa_atexit) that bare-metal linkers lack.
alignas(alignof(tflite::MicroInterpreter))
static uint8_t s_interp_buf[sizeof(tflite::MicroInterpreter)];

// Label order for the neuralSPOT-bundled KWS DS-CNN model. Verified against
// neuralSPOT/apps/ai/kws/src/kws_model_settings.h — NOT canonical Google
// Speech Commands order. "yes" sits at idx 9, "silence" at 10, "unknown" at 11.
const char *kws_label_names[KWS_NUM_CLASSES] = {
    "down", "go", "left", "no", "off", "on",
    "right", "stop", "up", "yes", "silence", "unknown"
};

void kws_init(void) {
    am_util_stdio_printf("[kws] init: InitializeTarget\r\n");
    tflite::InitializeTarget();

    am_util_stdio_printf("[kws] init: GetModel addr=%p first8=%02x%02x%02x%02x%02x%02x%02x%02x\r\n",
                         g_kws_model_data,
                         g_kws_model_data[0], g_kws_model_data[1],
                         g_kws_model_data[2], g_kws_model_data[3],
                         g_kws_model_data[4], g_kws_model_data[5],
                         g_kws_model_data[6], g_kws_model_data[7]);
    s_model = tflite::GetModel(g_kws_model_data);
    am_util_stdio_printf("[kws] init: model version=%lu (expected %d)\r\n",
                         (unsigned long)s_model->version(), TFLITE_SCHEMA_VERSION);
    if (s_model->version() != TFLITE_SCHEMA_VERSION) {
        am_util_stdio_printf("[kws] FATAL: schema version mismatch, aborting init\r\n");
        return;
    }

    am_util_stdio_printf("[kws] init: AddOps\r\n");
    s_resolver.AddFullyConnected();
    s_resolver.AddConv2D();
    s_resolver.AddDepthwiseConv2D();
    s_resolver.AddReshape();
    s_resolver.AddSoftmax();
    s_resolver.AddAveragePool2D();

    am_util_stdio_printf("[kws] init: ctor MicroInterpreter\r\n");
    s_interpreter = new(s_interp_buf) tflite::MicroInterpreter(
        s_model, s_resolver, s_arena, KWS_TENSOR_ARENA_SIZE);

    am_util_stdio_printf("[kws] init: stack hwm before AllocateTensors = %u words\r\n",
                         (unsigned)uxTaskGetStackHighWaterMark(NULL));
    am_util_stdio_printf("[kws] init: >AllocateTensors (arena=%d KB)\r\n",
                         KWS_TENSOR_ARENA_SIZE / 1024);
    uint32_t t0 = xTaskGetTickCount();
    TfLiteStatus status = s_interpreter->AllocateTensors();
    uint32_t dt = xTaskGetTickCount() - t0;
    am_util_stdio_printf("[kws] init: <AllocateTensors -> %d (%lu ms, used=%d/%d)\r\n",
                         (int)status, (unsigned long)dt,
                         (int)s_interpreter->arena_used_bytes(),
                         KWS_TENSOR_ARENA_SIZE);
    am_util_stdio_printf("[kws] init: stack hwm after AllocateTensors = %u words\r\n",
                         (unsigned)uxTaskGetStackHighWaterMark(NULL));
    if (status != kTfLiteOk) {
        am_util_stdio_printf("[kws] AllocateTensors failed! arena_used=%d\n",
                             (int)s_interpreter->arena_used_bytes());
        return;
    }

    s_input  = s_interpreter->input(0);
    s_output = s_interpreter->output(0);

    am_util_stdio_printf("[kws] input:  type=%d bytes=%u dims=%d",
                         (int)s_input->type, (unsigned)s_input->bytes,
                         s_input->dims ? s_input->dims->size : 0);
    if (s_input->dims) {
        am_util_stdio_printf(" [");
        for (int i = 0; i < s_input->dims->size; i++) {
            am_util_stdio_printf("%d%s", s_input->dims->data[i],
                                 i + 1 < s_input->dims->size ? "," : "");
        }
        am_util_stdio_printf("]");
    }
    am_util_stdio_printf(" (KWS_INPUT_SIZE compile-time=%d)\r\n", KWS_INPUT_SIZE);

    am_util_stdio_printf("[kws] output: type=%d bytes=%u dims=%d",
                         (int)s_output->type, (unsigned)s_output->bytes,
                         s_output->dims ? s_output->dims->size : 0);
    if (s_output->dims) {
        am_util_stdio_printf(" [");
        for (int i = 0; i < s_output->dims->size; i++) {
            am_util_stdio_printf("%d%s", s_output->dims->data[i],
                                 i + 1 < s_output->dims->size ? "," : "");
        }
        am_util_stdio_printf("]");
    }
    am_util_stdio_printf(" (KWS_NUM_CLASSES compile-time=%d)\r\n", KWS_NUM_CLASSES);

    am_util_stdio_printf("[kws] Ready. Arena used: %d / %d bytes\n",
                         (int)s_interpreter->arena_used_bytes(), KWS_TENSOR_ARENA_SIZE);
    am_util_stdio_printf("[kws] Trigger word: \"%s\" (idx %d), threshold %.0f%%\n",
                         kws_label_names[KWS_TRIGGER_IDX],
                         KWS_TRIGGER_IDX, KWS_THRESHOLD * 100.0f);
}

int kws_run_top2(const float *mfcc_features,
                 int *top1_idx, float *top1_conf,
                 int *top2_idx, float *top2_conf) {
    if (!s_interpreter) return -1;

    const int in_count  = (int)s_input->bytes;
    const int out_count = (int)s_output->bytes;

    float in_scale = s_input->params.scale;
    int   in_zp    = s_input->params.zero_point;
    for (int i = 0; i < in_count; i++) {
        float q = mfcc_features[i] / in_scale + (float)in_zp;
        if (q >  127.0f) q =  127.0f;
        if (q < -128.0f) q = -128.0f;
        s_input->data.int8[i] = (int8_t)q;
    }

    if (s_interpreter->Invoke() != kTfLiteOk) {
        return -1;
    }

    float out_scale = s_output->params.scale;
    int   out_zp    = s_output->params.zero_point;

    int   best_idx = 0,  second_idx = 0;
    float best     = -1e9f, second  = -1e9f;
    for (int i = 0; i < out_count; i++) {
        float score = ((float)s_output->data.int8[i] - (float)out_zp) * out_scale;
        if (score > best) {
            second     = best;
            second_idx = best_idx;
            best       = score;
            best_idx   = i;
        } else if (score > second) {
            second     = score;
            second_idx = i;
        }
    }

    if (top1_idx)  *top1_idx  = best_idx;
    if (top1_conf) *top1_conf = best;
    if (top2_idx)  *top2_idx  = second_idx;
    if (top2_conf) *top2_conf = second;
    return best_idx;
}

int kws_run(const float *mfcc_features, float *conf_out) {
    if (!s_interpreter) return -1;

    // Clamp loops to the tensor's own byte counts. Writing past the input
    // would clobber adjacent tensor metadata in the arena and fault on the
    // next read of s_output->data.
    const int in_count  = (int)s_input->bytes;
    const int out_count = (int)s_output->bytes;

    float in_scale = s_input->params.scale;
    int   in_zp    = s_input->params.zero_point;
    for (int i = 0; i < in_count; i++) {
        float q = mfcc_features[i] / in_scale + (float)in_zp;
        if (q >  127.0f) q =  127.0f;
        if (q < -128.0f) q = -128.0f;
        s_input->data.int8[i] = (int8_t)q;
    }

    if (s_interpreter->Invoke() != kTfLiteOk) {
        am_util_stdio_printf("[kws] Invoke failed\n");
        return -1;
    }

    // Dequantize output and find best class
    float out_scale = s_output->params.scale;
    int   out_zp    = s_output->params.zero_point;

    int   best_idx   = 0;
    float best_score = -1.0f;
    for (int i = 0; i < out_count; i++) {
        float score = ((float)s_output->data.int8[i] - (float)out_zp) * out_scale;
        if (score > best_score) {
            best_score = score;
            best_idx   = i;
        }
    }

    if (conf_out) *conf_out = best_score;
    return best_idx;
}
