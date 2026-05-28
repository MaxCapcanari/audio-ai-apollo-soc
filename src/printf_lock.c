// Serializes am_util_stdio_printf across FreeRTOS tasks. The SDK's
// printf uses a single static g_prfbuf — concurrent callers (RadioTask
// formatting BLE events, MicTask printing KWS results, etc.) race on
// that buffer, both producing torn output AND scribbling past the
// 1 KB end into adjacent .bss when one task's vsprintf clobbers
// another's length tracking.
//
// Activated via -Wl,--wrap=am_util_stdio_printf in gcc/Makefile. The
// linker redirects every am_util_stdio_printf call site to the wrapper
// below, which holds a recursive mutex around the SDK's vprintf.

#include "printf_lock.h"

#include <stdarg.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "am_util.h"

extern uint32_t am_util_stdio_vprintf(const char *pcFmt, va_list pArgs);

static StaticSemaphore_t s_mutexBuf;
static SemaphoreHandle_t s_mutex = NULL;

// Idle + timer task storage. configSUPPORT_STATIC_ALLOCATION=1 (enabled
// so we can use xSemaphoreCreateRecursiveMutexStatic above) makes these
// callbacks mandatory — the kernel calls them at startup to discover where
// to put the idle and timer task TCBs and stacks.
static StaticTask_t s_idleTcb;
static StackType_t  s_idleStack[configMINIMAL_STACK_SIZE];
static StaticTask_t s_timerTcb;
static StackType_t  s_timerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxTcb,
                                   StackType_t  **ppxStack,
                                   uint32_t      *pulStackSize) {
    *ppxTcb       = &s_idleTcb;
    *ppxStack     = s_idleStack;
    *pulStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTcb,
                                    StackType_t  **ppxStack,
                                    uint32_t      *pulStackSize) {
    *ppxTcb       = &s_timerTcb;
    *ppxStack     = s_timerStack;
    *pulStackSize = configTIMER_TASK_STACK_DEPTH;
}

void printf_lock_init(void) {
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateRecursiveMutexStatic(&s_mutexBuf);
    }
}

uint32_t __wrap_am_util_stdio_printf(const char *pcFmt, ...) {
    int can_lock = (s_mutex != NULL)
                   && (xPortIsInsideInterrupt() == pdFALSE)
                   && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);

    if (can_lock) {
        xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY);
    }

    va_list args;
    va_start(args, pcFmt);
    uint32_t n = am_util_stdio_vprintf(pcFmt, args);
    va_end(args);

    if (can_lock) {
        xSemaphoreGiveRecursive(s_mutex);
    }

    return n;
}
