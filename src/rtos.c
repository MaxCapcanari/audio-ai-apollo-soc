//*****************************************************************************
//
//! @file rtos.c
//!
//! @brief Essential functions to make the RTOS run correctly.
//!
//! These functions are required by the RTOS for ticking, sleeping, and basic
//! error checking.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"
#include "portable.h"
#include "ble_freertos_fit.h"
#include "mic_task.h"

extern TaskHandle_t radio_task_handle;

extern void HelloJsonOnButton0Pressed(void);

TaskHandle_t xSetupTask;

static void
ButtonTask(void *pvParameters)
{
    (void)pvParameters;

    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON0, g_AM_BSP_GPIO_BUTTON0);

    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t lastWake = xTaskGetTickCount();

    bool armed = true;
    uint8_t lowCount = 0;

    while (1)
    {
        vTaskDelayUntil(&lastWake, period);

        bool cur = am_hal_gpio_input_read(AM_BSP_GPIO_BUTTON0);

        if (armed)
        {
            if (!cur)
            {
                if (lowCount < 4) lowCount++;
                if (lowCount >= 4)
                {
                    HelloJsonOnButton0Pressed();
                    armed = false;
                    lowCount = 0;
                }
            }
            else
            {
                lowCount = 0;
            }
        }
        else
        {
            if (cur)
            {
                armed = true;
                lowCount = 0;
            }
        }
    }
}

//*****************************************************************************
//
// Interrupt handler for the CTIMER module.
//
//*****************************************************************************
void
am_timer_isr(void)
{
    uint32_t ui32Status;

    am_hal_timer_interrupt_status_get(false, &ui32Status);
    am_hal_timer_interrupt_clear(ui32Status);

    // we don't have this function in new hal
    //  am_hal_ctimer_int_service(ui32Status);
}

//*****************************************************************************
//
// Sleep function called from FreeRTOS IDLE task.
// Do necessary application specific Power down operations here
// Return 0 if this function also incorporates the WFI, else return value same
// as idleTime
//
//*****************************************************************************
uint32_t am_freertos_sleep(uint32_t idleTime)
{
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL); // DEBUG: changed from SLEEP_DEEP so JLink can halt
    return 0;
}

//*****************************************************************************
//
// Recovery function called from FreeRTOS IDLE task, after waking up from Sleep
// Do necessary 'wakeup' operations here, e.g. to power up/enable peripherals etc.
//
//*****************************************************************************
void am_freertos_wakeup(uint32_t idleTime)
{
    return;
}

//*****************************************************************************
//
// FreeRTOS debugging functions.
//
//*****************************************************************************
void
vApplicationMallocFailedHook(void)
{
    am_util_stdio_printf("\r\n[FATAL] FreeRTOS malloc failed\r\n");
    while (1);
}

void
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pxTask;
    am_util_stdio_printf("\r\n[FATAL] Stack overflow in task: %s\r\n",
                         pcTaskName ? pcTaskName : "(null)");
    while (1);
}

//*****************************************************************************
//
// High priority task to run immediately after the scheduler starts.
//
//*****************************************************************************
void
setup_task(void *pvParameters)
{
    am_util_debug_printf("Running setup tasks...\r\n");

    RadioTaskSetup();

    xTaskCreate(RadioTask, "RadioTask", 1024, 0, 3, &radio_task_handle);

    xTaskCreate(ButtonTask, "ButtonTask", 256, 0, 2, NULL);

    // 16384 words (64 KB) — Opus encoder uses ~10-20 KB; TFLM AllocateTensors
    // can also be stack-heavy during graph planning.
    xTaskCreate(MicTask, "MicTask", 16384, 0, 2, NULL);

    vTaskSuspend(NULL);

    while (1);
}

//*****************************************************************************
//
// Initializes all tasks
//
//*****************************************************************************
void
run_tasks(void)
{
    xTaskCreate(setup_task, "Setup", 512, 0, 3, &xSetupTask);

    vTaskStartScheduler();
}
