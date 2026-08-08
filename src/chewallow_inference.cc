#include "chewallow_inference.h"

#include <cstddef>

// NOTE: the C++14 sized-delete stub (operator delete(void*, size_t)) is
// already defined once in kws_inference.cc and covers this whole binary --
// do not redefine it here, it would be a duplicate-symbol link error.

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "chewallow_model_data.h"
#include "chewallow_inference.h"

#include "ns_core.h"
#include "am_util.h"
#include "FreeRTOS.h"
#include "task.h"

// Ops actually present in model.tflite, confirmed via interpreter introspection
// on desktop TF (XNNPACK DELEGATE op ignored — that's a desktop-only wrapper,
// not part of the real graph TFLM executes):
//   CONV_2D x3, MUL x4, ADD x4, MAX_POOL_2D x3, MEAN x1,
//   FULLY_CONNECTED x2, SOFTMAX x1
// MUL/ADD pairs are folded batchnorm (scale + shift), not arithmetic on
// arbitrary tensors -- still need the generic Mul/Add ops registered.
#define CHEWALLOW_NUM_OPS 7

// Starting arena guess: float32 activations on a (250,22) input are far
// larger than KWS's int8 (49,10) ones. 256 KB is a deliberate overestimate;
// AllocateTensors() below reports the real arena_used_bytes() so we can
// right-size this once we have a measurement instead of guessing twice.
#define CHEWALLOW_TENSOR_ARENA_SIZE (64 * 1024)

//__attribute__((section(".sram_bss"))) alignas(16) static uint8_t s_arena[CHEWALLOW_TENSOR_ARENA_SIZE];
// back to:
AM_SHARED_RW alignas(16) static uint8_t s_arena[CHEWALLOW_TENSOR_ARENA_SIZE];

static tflite::MicroMutableOpResolver<CHEWALLOW_NUM_OPS> s_resolver;
static const tflite::Model             *s_model      = nullptr;
static tflite::MicroInterpreter        *s_interpreter = nullptr;
static TfLiteTensor                    *s_input       = nullptr;
static TfLiteTensor                    *s_output      = nullptr;

alignas(alignof(tflite::MicroInterpreter))
static uint8_t s_interp_buf[sizeof(tflite::MicroInterpreter)];

const char *chewallow_label_names[CHEWALLOW_NUM_CLASSES] = {
    "nothing", "chewallow"
};

void chewallow_init(void) {
    am_util_stdio_printf("[chewallow] init: InitializeTarget\r\n");
    tflite::InitializeTarget();

    am_util_stdio_printf("[chewallow] init: GetModel addr=%p first8=%02x%02x%02x%02x%02x%02x%02x%02x\r\n",
                         g_chewallow_model_data,
                         g_chewallow_model_data[0], g_chewallow_model_data[1],
                         g_chewallow_model_data[2], g_chewallow_model_data[3],
                         g_chewallow_model_data[4], g_chewallow_model_data[5],
                         g_chewallow_model_data[6], g_chewallow_model_data[7]);
    s_model = tflite::GetModel(g_chewallow_model_data);
    am_util_stdio_printf("[chewallow] init: model version=%lu (expected %d)\r\n",
                         (unsigned long)s_model->version(), TFLITE_SCHEMA_VERSION);
    if (s_model->version() != TFLITE_SCHEMA_VERSION) {
        am_util_stdio_printf("[chewallow] FATAL: schema version mismatch, aborting init\r\n");
        return;
    }

    am_util_stdio_printf("[chewallow] init: AddOps\r\n");
    s_resolver.AddConv2D();
    s_resolver.AddMul();
    s_resolver.AddAdd();
    s_resolver.AddMaxPool2D();
    s_resolver.AddMean();
    s_resolver.AddFullyConnected();
    s_resolver.AddSoftmax();

    am_util_stdio_printf("[chewallow] init: ctor MicroInterpreter\r\n");
    s_interpreter = new(s_interp_buf) tflite::MicroInterpreter(
        s_model, s_resolver, s_arena, CHEWALLOW_TENSOR_ARENA_SIZE);

    am_util_stdio_printf("[chewallow] init: stack hwm before AllocateTensors = %u words\r\n",
                         (unsigned)uxTaskGetStackHighWaterMark(NULL));
    am_util_stdio_printf("[chewallow] init: >AllocateTensors (arena=%d KB)\r\n",
                         CHEWALLOW_TENSOR_ARENA_SIZE / 1024);
    uint32_t t0 = xTaskGetTickCount();
    TfLiteStatus status = s_interpreter->AllocateTensors();
    uint32_t dt = xTaskGetTickCount() - t0;
    am_util_stdio_printf("[chewallow] init: <AllocateTensors -> %d (%lu ms, used=%d/%d)\r\n",
                         (int)status, (unsigned long)dt,
                         (int)s_interpreter->arena_used_bytes(),
                         CHEWALLOW_TENSOR_ARENA_SIZE);
    am_util_stdio_printf("[chewallow] init: stack hwm after AllocateTensors = %u words\r\n",
                         (unsigned)uxTaskGetStackHighWaterMark(NULL));
    if (status != kTfLiteOk) {
        am_util_stdio_printf("[chewallow] AllocateTensors failed! arena_used=%d\n",
                             (int)s_interpreter->arena_used_bytes());
        return;
    }

    s_input  = s_interpreter->input(0);
    s_output = s_interpreter->output(0);

    am_util_stdio_printf("[chewallow] input:  type=%d bytes=%u dims=%d",
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
    am_util_stdio_printf(" (CHEWALLOW_INPUT_SIZE compile-time=%d)\r\n", CHEWALLOW_INPUT_SIZE);

    am_util_stdio_printf("[chewallow] output: type=%d bytes=%u dims=%d",
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
    am_util_stdio_printf(" (CHEWALLOW_NUM_CLASSES compile-time=%d)\r\n", CHEWALLOW_NUM_CLASSES);

    am_util_stdio_printf("[chewallow] Ready. Arena used: %d / %d bytes\n",
                         (int)s_interpreter->arena_used_bytes(), CHEWALLOW_TENSOR_ARENA_SIZE);
}

// Shared implementation for chewallow_run / chewallow_run_dummy_timing.
// Float32 model end-to-end -- no quantize/dequantize step, unlike KWS.
static int chewallow_run_impl(const float *mfcc_features, float *p_chewallow_out) {
	if (!s_interpreter || !s_input || !s_output) {
			am_util_stdio_printf("[chewallow] run called before successful init\r\n");
			return -1;
		}

    const int in_floats  = (int)(s_input->bytes  / sizeof(float));
    const int out_floats = (int)(s_output->bytes / sizeof(float));

    // Straight copy -- caller is responsible for matching the notebook's
    // global z-score normalization before calling this.
    for (int i = 0; i < in_floats; i++) {
        s_input->data.f[i] = mfcc_features[i];
    }

    if (s_interpreter->Invoke() != kTfLiteOk) {
        return -1;
    }

    int   best_idx = 0;
    float best     = -1e9f;
    for (int i = 0; i < out_floats; i++) {
        float score = s_output->data.f[i];
        if (score > best) {
            best     = score;
            best_idx = i;
        }
    }

    if (p_chewallow_out && out_floats > CHEWALLOW_IDX_CHEWALLOW) {
        *p_chewallow_out = s_output->data.f[CHEWALLOW_IDX_CHEWALLOW];
    }
    return best_idx;
}

int chewallow_run(const float *mfcc_features, float *p_chewallow_out) {
    return chewallow_run_impl(mfcc_features, p_chewallow_out);
}

int chewallow_run_dummy_timing(const float *dummy_features, float *p_chewallow_out) {
    return chewallow_run_impl(dummy_features, p_chewallow_out);
}
