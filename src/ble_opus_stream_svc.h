#ifndef BLE_OPUS_STREAM_SVC_H
#define BLE_OPUS_STREAM_SVC_H

#include "wsf_types.h"
#include "dm_api.h"

// Pump callback: delayMs > 0 starts the timer, 0 stops it.
typedef void (*opus_pump_fn_t)(uint16_t delayMs);

void   OpusStreamSvcAdd(void);
void   OpusStreamCccState(dmConnId_t connId, bool_t enabled);
void   OpusStreamConnClose(dmConnId_t connId);
void   OpusStreamSetMtu(uint16_t mtu);
void   OpusStreamSetPumpCallback(opus_pump_fn_t fn);
bool_t OpusStreamOnJsonCmd(const char *json, uint16_t len);
void   OpusStreamTick(void);

#endif
