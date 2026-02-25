#ifndef BLE_OPUS_STREAM_SVC_H
#define BLE_OPUS_STREAM_SVC_H

#include "wsf_types.h"
#include "dm_api.h"

void OpusStreamSvcAdd(void);
void OpusStreamCccState(dmConnId_t connId, bool_t enabled);
void OpusStreamConnClose(dmConnId_t connId);
bool_t OpusStreamSendNextChunk(void);
void OpusStreamSetMtu(uint16_t mtu);

#endif
