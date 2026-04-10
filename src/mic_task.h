#ifndef MIC_TASK_H
#define MIC_TASK_H

#include <stdint.h>

//*****************************************************************************
//! @brief FreeRTOS task entry point for INMP441 I2S microphone.
//!
//! Call MicTaskSetup() once before starting the scheduler, then create the
//! task with xTaskCreate(MicTask, ...).
//*****************************************************************************

void MicTask(void *pvParameters);

//*****************************************************************************
//! @brief Return a pointer to the latest Opus-encoded recording.
//!
//! @param pLen  Out: number of valid bytes in the returned buffer. 0 before
//!              any recording has completed.
//!
//! @return Pointer to the internal Opus byte buffer. Always non-NULL; the
//!         buffer is only valid for *pLen bytes. The buffer is overwritten
//!         when a new recording finishes encoding.
//*****************************************************************************
const uint8_t *MicTaskGetOpusData(uint32_t *pLen);

#endif // MIC_TASK_H
