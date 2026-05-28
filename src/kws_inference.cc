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

// Chewallow int8 model needs 175 KB measured; 192 KB gives ~10% headroom
// for future variants without bloating TCM.
#define KWS_TENSOR_ARENA_SIZE (192 * 1024)

// Tensor arena in TCM (single-cycle CPU access) rather than SHARED_SRAM
// (multi-cycle AXI fabric). CMSIS-NN s8 conv is access-bound on its int8
// activation/weight loads, so TCM placement is the largest single-knob
// speedup we have without retraining. The linker_script.ld routes
// .tcm_bss to MCU_TCM; in exchange, the FreeRTOS heap was moved out of
// TCM into .sram_bss to make room.
__attribute__((section(".tcm_bss"))) alignas(16) static uint8_t s_arena[KWS_TENSOR_ARENA_SIZE];

static tflite::MicroMutableOpResolver<7> s_resolver;
static const tflite::Model             *s_model      = nullptr;
static tflite::MicroInterpreter        *s_interpreter = nullptr;
static TfLiteTensor                    *s_input       = nullptr;
static TfLiteTensor                    *s_output      = nullptr;

// Placement-new buffer for MicroInterpreter — avoids static-local destructor
// registration (__dso_handle / __cxa_atexit) that bare-metal linkers lack.
alignas(alignof(tflite::MicroInterpreter))
static uint8_t s_interp_buf[sizeof(tflite::MicroInterpreter)];

// Chewallow binary classifier. Mirrors kCategoryLabels in
// kws/kws_model_settings.h. Keep these two in sync.
const char *kws_label_names[KWS_NUM_CLASSES] = {
    "nothing", "chewallow"
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
    s_resolver.AddConv2D();
    s_resolver.AddMul();
    s_resolver.AddAdd();
    s_resolver.AddMaxPool2D();
    s_resolver.AddMean();
    s_resolver.AddFullyConnected();
    s_resolver.AddSoftmax();

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

// Element count from dims — bytes scales with the tensor's element size
// (4× for float32 vs int8), so looping on `bytes` would overrun a float
// tensor and only touch the low byte of each element.
static int tensor_element_count(const TfLiteTensor *t) {
    if (!t || !t->dims) return 0;
    int n = 1;
    for (int i = 0; i < t->dims->size; i++) n *= t->dims->data[i];
    return n;
}

// Returns 0 on success, -1 if input tensor type is unsupported.
static int write_input(const float *mfcc_features) {
    const int n = tensor_element_count(s_input);
    if (s_input->type == kTfLiteFloat32) {
        for (int i = 0; i < n; i++) s_input->data.f[i] = mfcc_features[i];
        return 0;
    }
    if (s_input->type == kTfLiteInt8) {
        float scale = s_input->params.scale;
        int   zp    = s_input->params.zero_point;
        for (int i = 0; i < n; i++) {
            float q = mfcc_features[i] / scale + (float)zp;
            if (q >  127.0f) q =  127.0f;
            if (q < -128.0f) q = -128.0f;
            s_input->data.int8[i] = (int8_t)q;
        }
        return 0;
    }
    return -1;
}

// Reads class i score in real units (post-softmax probability for our model).
// Returns NAN if output type is unsupported.
static float read_output_score(int i) {
    if (s_output->type == kTfLiteFloat32) {
        return s_output->data.f[i];
    }
    if (s_output->type == kTfLiteInt8) {
        return ((float)s_output->data.int8[i] - (float)s_output->params.zero_point)
               * s_output->params.scale;
    }
    return __builtin_nanf("");
}

int kws_run_top2(const float *mfcc_features,
                 int *top1_idx, float *top1_conf,
                 int *top2_idx, float *top2_conf) {
    if (!s_interpreter || !s_input || !s_output) return -1;

    if (write_input(mfcc_features) != 0) return -1;

    if (s_interpreter->Invoke() != kTfLiteOk) {
        return -1;
    }

    const int out_count = tensor_element_count(s_output);
    int   best_idx = 0,  second_idx = 0;
    float best     = -1e9f, second  = -1e9f;
    for (int i = 0; i < out_count; i++) {
        float score = read_output_score(i);
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
    if (!s_interpreter || !s_input || !s_output) return -1;

    if (write_input(mfcc_features) != 0) return -1;

    if (s_interpreter->Invoke() != kTfLiteOk) {
        am_util_stdio_printf("[kws] Invoke failed\n");
        return -1;
    }

    const int out_count = tensor_element_count(s_output);
    int   best_idx   = 0;
    float best_score = -1.0f;
    for (int i = 0; i < out_count; i++) {
        float score = read_output_score(i);
        if (score > best_score) {
            best_score = score;
            best_idx   = i;
        }
    }

    if (conf_out) *conf_out = best_score;
    return best_idx;
}
