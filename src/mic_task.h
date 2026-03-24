#ifndef MIC_TASK_H
#define MIC_TASK_H

//*****************************************************************************
//! @brief FreeRTOS task entry point for INMP441 I2S microphone.
//!
//! Call MicTaskSetup() once before starting the scheduler, then create the
//! task with xTaskCreate(MicTask, ...).
//*****************************************************************************

void MicTask(void *pvParameters);

#endif // MIC_TASK_H
