#include <string.h>

#include "wsf_types.h"
#include "att_api.h"
#include "dm_api.h"

#include "am_util.h"

#include "fake_opus_audio.h"

#define OPUS_SVC_START_HDL  0x9010
#define OPUS_VAL_HDL        (OPUS_SVC_START_HDL + 2)
#define OPUS_CCC_HDL        (OPUS_SVC_START_HDL + 3)
#define OPUS_SVC_END_HDL    (OPUS_SVC_START_HDL + 3)

#define OPUS_META_BYTES        8U
#define OPUS_MIN_ATT_MTU       23U
#define OPUS_MAX_AUDIO_PAYLOAD 200U
#define OPUS_ATT_HDR_BYTES     3U
#define OPUS_FLAG_START        0x01U
#define OPUS_FLAG_END          0x02U

static bool_t g_opusGroupAdded = FALSE;

static const uint8_t opusSvcUuid[ATT_128_UUID_LEN] =
{
  0xBB,0x90,0x78,0x56,0x34,0x12,0x34,0x12,
  0x34,0x12,0x78,0x56,0x34,0x12,0x34,0x12
};

static const uint8_t opusValUuid[ATT_128_UUID_LEN] =
{
  0xBC,0x90,0x78,0x56,0x34,0x12,0x34,0x12,
  0x34,0x12,0x78,0x56,0x34,0x12,0x34,0x12
};

static uint8_t opusChDecl[1 + 2 + ATT_128_UUID_LEN] =
{
  (ATT_PROP_READ | ATT_PROP_NOTIFY),
  (uint8_t)(OPUS_VAL_HDL & 0xFF),
  (uint8_t)((OPUS_VAL_HDL >> 8) & 0xFF),
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

static uint16_t opusSvcUuidLen = sizeof(opusSvcUuid);
static uint16_t opusChDeclLen  = sizeof(opusChDecl);

static uint8_t  opusValBuf[OPUS_META_BYTES + OPUS_MAX_AUDIO_PAYLOAD];
static uint16_t opusValLen = 0;

static uint8_t  opusCccVal[2] = { 0x00, 0x00 };
static uint16_t opusCccLen = sizeof(opusCccVal);

static attsAttr_t opusSvcList[] =
{
  {
    (uint8_t *)attPrimSvcUuid,
    (uint8_t *)opusSvcUuid,
    &opusSvcUuidLen,
    sizeof(opusSvcUuid),
    0,
    ATTS_PERMIT_READ
  },
  {
    (uint8_t *)attChUuid,
    opusChDecl,
    &opusChDeclLen,
    sizeof(opusChDecl),
    0,
    ATTS_PERMIT_READ
  },
  {
    (uint8_t *)opusValUuid,
    opusValBuf,
    &opusValLen,
    sizeof(opusValBuf),
    0,
    ATTS_PERMIT_READ
  },
  {
    (uint8_t *)attCliChCfgUuid,
    opusCccVal,
    &opusCccLen,
    sizeof(opusCccVal),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE)
  }
};

static attsGroup_t opusSvcGroup =
{
  NULL,
  opusSvcList,
  NULL,
  NULL,
  OPUS_SVC_START_HDL,
  OPUS_SVC_END_HDL
};

static dmConnId_t g_connId = DM_CONN_ID_NONE;
static bool_t     g_notifyEnabled = FALSE;
static uint32_t   g_opusOffset = 0;
static uint16_t   g_pktIndex = 0;
static uint16_t   g_totalPackets = 0;
static uint16_t   g_payloadBytes = FAKE_OPUS_FRAME_BYTES;
static uint16_t   g_attMtu = OPUS_MIN_ATT_MTU;

static uint16_t opusComputePayloadBytes(void)
{
  uint16_t maxByMtu;
  if (g_attMtu <= (OPUS_ATT_HDR_BYTES + OPUS_META_BYTES))
  {
    return 1U;
  }

  maxByMtu = (uint16_t)(g_attMtu - OPUS_ATT_HDR_BYTES - OPUS_META_BYTES);
  if (maxByMtu > OPUS_MAX_AUDIO_PAYLOAD)
  {
    maxByMtu = OPUS_MAX_AUDIO_PAYLOAD;
  }
  if (maxByMtu == 0U)
  {
    maxByMtu = 1U;
  }
  return maxByMtu;
}

static void opusResetStreamState(void)
{
  uint32_t totalLen;
  (void)FakeOpusAudioGetData(&totalLen);

  g_opusOffset = 0;
  g_pktIndex = 0;
  g_payloadBytes = opusComputePayloadBytes();
  g_totalPackets = (uint16_t)((totalLen + g_payloadBytes - 1U) / g_payloadBytes);
}

static void opusBuildCurrentPacket(void)
{
  const uint8_t *pData;
  uint32_t totalLen;
  uint32_t remaining;
  uint16_t chunkLen;
  uint8_t flags = 0;

  pData = FakeOpusAudioGetData(&totalLen);
  if (g_opusOffset >= totalLen)
  {
    opusValLen = 0;
    return;
  }

  remaining = totalLen - g_opusOffset;
  chunkLen = (remaining >= g_payloadBytes) ? g_payloadBytes : (uint16_t)remaining;

  if (g_pktIndex == 0U)
  {
    flags |= OPUS_FLAG_START;
  }
  if ((g_opusOffset + chunkLen) >= totalLen)
  {
    flags |= OPUS_FLAG_END;
  }

  opusValBuf[0] = flags;
  opusValBuf[1] = 1U; /* metadata version */
  opusValBuf[2] = (uint8_t)(g_pktIndex & 0xFFU);
  opusValBuf[3] = (uint8_t)((g_pktIndex >> 8) & 0xFFU);
  opusValBuf[4] = (uint8_t)(g_totalPackets & 0xFFU);
  opusValBuf[5] = (uint8_t)((g_totalPackets >> 8) & 0xFFU);
  opusValBuf[6] = (uint8_t)(totalLen & 0xFFU);
  opusValBuf[7] = (uint8_t)((totalLen >> 8) & 0xFFU);
  memcpy(&opusValBuf[OPUS_META_BYTES], &pData[g_opusOffset], chunkLen);

  opusValLen = (uint16_t)(OPUS_META_BYTES + chunkLen);
  AttsSetAttr(OPUS_VAL_HDL, opusValLen, opusValBuf);
}

void OpusStreamSvcAdd(void)
{
  if (g_opusGroupAdded)
  {
    return;
  }
  g_opusGroupAdded = TRUE;

  memcpy(&opusChDecl[3], opusValUuid, ATT_128_UUID_LEN);
  AttsAddGroup(&opusSvcGroup);

  opusResetStreamState();
  opusBuildCurrentPacket();
}

void OpusStreamCccState(dmConnId_t connId, bool_t enabled)
{
  if ((g_connId != DM_CONN_ID_NONE) && (connId != g_connId))
  {
    return;
  }

  g_connId = connId;
  g_notifyEnabled = enabled;
  opusResetStreamState();

  if (enabled)
  {
    opusBuildCurrentPacket();
    am_util_debug_printf("OpusStream: notify enabled (connId=%d)\r\n", connId);
    am_util_debug_printf("OpusStream: mtu=%u payload=%u total_packets=%u\r\n",
                         (unsigned)g_attMtu,
                         (unsigned)g_payloadBytes,
                         (unsigned)g_totalPackets);
  }
  else
  {
    am_util_debug_printf("OpusStream: notify disabled (connId=%d)\r\n", connId);
  }
}

void OpusStreamConnClose(dmConnId_t connId)
{
  if (g_connId == connId)
  {
    g_connId = DM_CONN_ID_NONE;
    g_notifyEnabled = FALSE;
    opusResetStreamState();
  }
}

bool_t OpusStreamSendNextChunk(void)
{
  uint32_t totalLen;
  const uint8_t *pData = FakeOpusAudioGetData(&totalLen);
  uint32_t beforeOffset;

  (void)pData;

  if (!g_notifyEnabled)
  {
    return FALSE;
  }

  if (g_connId == DM_CONN_ID_NONE)
  {
    return FALSE;
  }

  if (g_opusOffset >= totalLen)
  {
    return FALSE;
  }

  beforeOffset = g_opusOffset;
  opusBuildCurrentPacket();

  if (opusValLen == 0)
  {
    return FALSE;
  }

  AttsHandleValueNtf(g_connId, OPUS_VAL_HDL, opusValLen, opusValBuf);
  g_opusOffset += (uint32_t)(opusValLen - OPUS_META_BYTES);
  g_pktIndex++;

  if (g_opusOffset >= totalLen)
  {
    am_util_debug_printf("OpusStream: stream complete (%u bytes)\r\n",
                         (unsigned)totalLen);
    return FALSE;
  }

  if ((beforeOffset % ((uint32_t)g_payloadBytes * 100U)) == 0U)
  {
    am_util_debug_printf("OpusStream: sent %u/%u bytes\r\n",
                         (unsigned)g_opusOffset, (unsigned)totalLen);
  }

  return TRUE;
}

void OpusStreamSetMtu(uint16_t mtu)
{
  if (mtu < OPUS_MIN_ATT_MTU)
  {
    mtu = OPUS_MIN_ATT_MTU;
  }

  g_attMtu = mtu;

  if (g_notifyEnabled)
  {
    /* Restart with deterministic packetization for this MTU. */
    opusResetStreamState();
    opusBuildCurrentPacket();
    am_util_debug_printf("OpusStream: MTU updated to %u, stream reset\r\n",
                         (unsigned)g_attMtu);
  }
}

/*
 * Stubs for the windowed-flow-control pump API referenced by the patched
 * fit_main.c.  The full implementation lives on the main Opus branch
 * (commit 283f784).  On the `audio_analog` branch we only care about the
 * AUDADC capture path, so these stubs let the firmware link and boot.
 * Re-plumbing the pump path is tracked separately.
 */
void OpusStreamTick(void)
{
  /* no-op on this branch */
}

void OpusStreamSetPumpCallback(void (*fn)(uint16_t))
{
  (void)fn;
  /* no-op on this branch */
}
