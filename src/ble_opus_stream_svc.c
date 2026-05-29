#include <string.h>
#include <stdlib.h>

#include "wsf_types.h"
#include "att_api.h"
#include "dm_api.h"

#include "am_util.h"

#include "mic_task.h"
#include "ble_opus_stream_svc.h"

/* ------- GATT handle layout ------- */
#define OPUS_SVC_START_HDL  0x9010
#define OPUS_VAL_HDL        (OPUS_SVC_START_HDL + 2)
#define OPUS_CCC_HDL        (OPUS_SVC_START_HDL + 3)
#define OPUS_SVC_END_HDL    (OPUS_SVC_START_HDL + 3)

/* ------- Protocol constants ------- */
#define OPUS_META_BYTES         10U
#define OPUS_MIN_ATT_MTU        23U
#define OPUS_MAX_AUDIO_PAYLOAD  200U
#define OPUS_ATT_HDR_BYTES      3U
#define OPUS_FLAG_START         0x01U
#define OPUS_FLAG_END           0x02U
#define OPUS_FLAG_WINDOW_END    0x04U
#define OPUS_DEFAULT_WINDOW     20U
#define OPUS_INTER_PKT_DELAY_MS 20U
#define OPUS_ACK_TIMEOUT_MS     30000U
#define OPUS_MAX_MISSING        64U

/* Set to 1 to enable per-packet "[opus tx] idx=N/M flags=0x.. len=.." UART
 * logs from opusSendPacketAtIndex(). Useful for diagnosing whether the
 * firmware actually pushed every packet down to Cordio. Off by default
 * because each line takes a few ms to emit at 115200 baud, which is
 * significant relative to the 50 ms inter-packet pacing. */
#ifndef OPUS_VERBOSE_TX
#define OPUS_VERBOSE_TX         0
#endif

/* ------- GATT service data ------- */
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

/* ------- Connection & MTU state ------- */
static dmConnId_t g_connId        = DM_CONN_ID_NONE;
static bool_t     g_notifyEnabled = FALSE;
static uint16_t   g_attMtu        = OPUS_MIN_ATT_MTU;

/* ------- Windowed stream state ------- */
typedef enum
{
  STREAM_IDLE,
  STREAM_SENDING_WINDOW,
  STREAM_RESENDING,
  STREAM_WAITING_ACK
} stream_state_t;

static stream_state_t g_streamState  = STREAM_IDLE;
static uint16_t g_totalPackets       = 0;
static uint16_t g_payloadBytes       = OPUS_MAX_AUDIO_PAYLOAD;
static uint16_t g_windowSize         = OPUS_DEFAULT_WINDOW;
static uint16_t g_windowStart        = 0;
static uint16_t g_windowSent         = 0;
static uint16_t g_missingList[OPUS_MAX_MISSING];
static uint16_t g_missingCount       = 0;
static uint16_t g_missingIdx         = 0;

/* Auto-send: set TRUE by OpusStreamStartAuto() when a recording is ready
 * but no phone is connected (or notifications not yet enabled). The pending
 * stream is flushed as soon as CCC enable arrives. Cleared on transfer
 * complete or on explicit stream_start (manual replay). */
static bool_t g_pendingAutoStart = FALSE;

/* ------- Pump callback ------- */
static opus_pump_fn_t g_pumpFn = NULL;

/* ======================================================================
 *  Helpers
 * ====================================================================== */

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

static void opusRecomputeStream(void)
{
  uint32_t totalLen;
  (void)MicTaskGetOpusData(&totalLen);

  g_payloadBytes = opusComputePayloadBytes();
  g_totalPackets = (totalLen == 0U)
                   ? 0
                   : (uint16_t)((totalLen + g_payloadBytes - 1U) / g_payloadBytes);
}

static void opusSchedule(uint16_t delayMs)
{
  if (g_pumpFn)
  {
    g_pumpFn(delayMs);
  }
}

/* Build header + payload for a specific packet index and send as notification. */
static void opusSendPacketAtIndex(uint16_t pktIdx, uint8_t extraFlags)
{
  const uint8_t *pData;
  uint32_t totalLen;

  pData = MicTaskGetOpusData(&totalLen);
  if (totalLen == 0U || pktIdx >= g_totalPackets)
  {
    return;
  }

  uint32_t offset = (uint32_t)pktIdx * g_payloadBytes;
  if (offset >= totalLen)
  {
    return;
  }

  uint32_t remaining = totalLen - offset;
  uint16_t chunkLen  = (remaining >= g_payloadBytes)
                       ? g_payloadBytes
                       : (uint16_t)remaining;

  uint8_t flags = extraFlags;
  if (pktIdx == 0U)
  {
    flags |= OPUS_FLAG_START;
  }
  if ((offset + chunkLen) >= totalLen)
  {
    flags |= OPUS_FLAG_END;
  }

  opusValBuf[0] = flags;
  opusValBuf[1] = 1U; /* metadata version */
  opusValBuf[2] = (uint8_t)(pktIdx & 0xFFU);
  opusValBuf[3] = (uint8_t)((pktIdx >> 8) & 0xFFU);
  opusValBuf[4] = (uint8_t)(g_totalPackets & 0xFFU);
  opusValBuf[5] = (uint8_t)((g_totalPackets >> 8) & 0xFFU);
  opusValBuf[6] = (uint8_t)(totalLen & 0xFFU);
  opusValBuf[7] = (uint8_t)((totalLen >> 8) & 0xFFU);
  opusValBuf[8] = (uint8_t)((totalLen >> 16) & 0xFFU);
  opusValBuf[9] = (uint8_t)((totalLen >> 24) & 0xFFU);
  memcpy(&opusValBuf[OPUS_META_BYTES], &pData[offset], chunkLen);

  opusValLen = (uint16_t)(OPUS_META_BYTES + chunkLen);
  AttsSetAttr(OPUS_VAL_HDL, opusValLen, opusValBuf);

#if OPUS_VERBOSE_TX
  am_util_stdio_printf("[opus tx] idx=%u/%u flags=0x%02X len=%u\r\n",
                       (unsigned)pktIdx,
                       (unsigned)g_totalPackets,
                       (unsigned)flags,
                       (unsigned)opusValLen);
#endif

  AttsHandleValueNtf(g_connId, OPUS_VAL_HDL, opusValLen, opusValBuf);
}

/* ======================================================================
 *  Minimal JSON helpers (no library — safe for our known command format)
 * ====================================================================== */

/* Find the value start for a given key in a JSON string.
 * Returns pointer to first non-whitespace char after "key": */
static const char *jsonFindValue(const char *json, const char *key)
{
  const char *p = json;
  size_t klen   = strlen(key);

  while ((p = strchr(p, '"')) != NULL)
  {
    p++; /* skip opening quote */
    if (strncmp(p, key, klen) == 0 && p[klen] == '"')
    {
      p += klen + 1; /* skip key + closing quote */
      while (*p == ' ' || *p == '\t') p++;
      if (*p != ':') continue;
      p++;
      while (*p == ' ' || *p == '\t') p++;
      return p;
    }
  }
  return NULL;
}

static bool_t jsonMatchStr(const char *json, const char *key, const char *val)
{
  const char *p = jsonFindValue(json, key);
  if (!p || *p != '"') return FALSE;
  p++;
  size_t vlen = strlen(val);
  return (strncmp(p, val, vlen) == 0 && p[vlen] == '"');
}

static bool_t jsonGetInt(const char *json, const char *key, int *out)
{
  const char *p = jsonFindValue(json, key);
  if (!p) return FALSE;
  char *end;
  long v = strtol(p, &end, 10);
  if (end == p) return FALSE;
  *out = (int)v;
  return TRUE;
}

static uint16_t jsonGetIntArray(const char *json, const char *key,
                                uint16_t *outArr, uint16_t maxCount)
{
  const char *p = jsonFindValue(json, key);
  if (!p || *p != '[') return 0;
  p++;
  uint16_t count = 0;
  while (count < maxCount)
  {
    while (*p == ' ' || *p == '\t') p++;
    if (*p == ']' || *p == '\0') break;
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p) break;
    outArr[count++] = (uint16_t)v;
    p = end;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == ',') p++;
  }
  return count;
}

/* Begin (or restart) a window-mode transfer at packet 0. Returns TRUE if a
 * stream was actually kicked off, FALSE if there is no recording to send.
 * Caller must ensure notifications are enabled. */
static bool_t opusKickOffStream(uint16_t windowSize)
{
  uint32_t totalLen;
  (void)MicTaskGetOpusData(&totalLen);
  if (totalLen == 0U)
  {
    return FALSE;
  }

  opusRecomputeStream();
  if (g_totalPackets == 0U)
  {
    return FALSE;
  }

  g_windowSize  = (windowSize == 0U) ? OPUS_DEFAULT_WINDOW : windowSize;
  g_windowStart = 0;
  g_windowSent  = 0;
  g_streamState = STREAM_SENDING_WINDOW;

  am_util_debug_printf("OpusStream: start (window=%u, packets=%u, bytes=%u)\r\n",
                       (unsigned)g_windowSize, (unsigned)g_totalPackets,
                       (unsigned)totalLen);
  opusSchedule(1);
  return TRUE;
}

/* ======================================================================
 *  Public API
 * ====================================================================== */

void OpusStreamSetPumpCallback(opus_pump_fn_t fn)
{
  g_pumpFn = fn;
}

void OpusStreamStartAuto(void)
{
  uint32_t totalLen;
  (void)MicTaskGetOpusData(&totalLen);
  if (totalLen == 0U)
  {
    g_pendingAutoStart = FALSE;
    return;
  }

  /* A new recording is ready — any prior in-flight stream is now stale
   * (it referenced the previous g_opusBuf contents). */
  g_streamState = STREAM_IDLE;

  if (g_notifyEnabled && g_connId != DM_CONN_ID_NONE)
  {
    am_util_debug_printf("OpusStream: auto-send (recording ready, phone connected)\r\n");
    if (opusKickOffStream(OPUS_DEFAULT_WINDOW))
    {
      g_pendingAutoStart = FALSE;
      return;
    }
  }

  am_util_debug_printf("OpusStream: auto-send pending (waiting for phone)\r\n");
  g_pendingAutoStart = TRUE;
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
}

void OpusStreamCccState(dmConnId_t connId, bool_t enabled)
{
  if ((g_connId != DM_CONN_ID_NONE) && (connId != g_connId))
  {
    return;
  }

  g_connId        = connId;
  g_notifyEnabled = enabled;

  if (enabled)
  {
    opusRecomputeStream();
    am_util_debug_printf("OpusStream: notify enabled (connId=%d, mtu=%u, payload=%u)\r\n",
                         connId, (unsigned)g_attMtu, (unsigned)g_payloadBytes);

    /* Flush any recording that finished encoding before the phone was ready. */
    if (g_pendingAutoStart)
    {
      am_util_debug_printf("OpusStream: flushing pending recording\r\n");
      if (opusKickOffStream(OPUS_DEFAULT_WINDOW))
      {
        g_pendingAutoStart = FALSE;
      }
    }
  }
  else
  {
    g_streamState = STREAM_IDLE;
    opusSchedule(0);
    am_util_debug_printf("OpusStream: notify disabled (connId=%d)\r\n", connId);
  }
}

void OpusStreamConnClose(dmConnId_t connId)
{
  if (g_connId == connId)
  {
    g_connId        = DM_CONN_ID_NONE;
    g_notifyEnabled = FALSE;
    g_streamState   = STREAM_IDLE;
    opusSchedule(0);
  }
}

void OpusStreamSetMtu(uint16_t mtu)
{
  if (mtu < OPUS_MIN_ATT_MTU)
  {
    mtu = OPUS_MIN_ATT_MTU;
  }

  g_attMtu = mtu;

  /* Only recompute if we are not mid-stream; MTU should be negotiated
   * before the phone sends stream_start. */
  if (g_streamState == STREAM_IDLE)
  {
    opusRecomputeStream();
    am_util_debug_printf("OpusStream: MTU %u, payload %u\r\n",
                         (unsigned)g_attMtu, (unsigned)g_payloadBytes);
  }
}

/* -----------------------------------------------------------------------
 * OpusStreamOnJsonCmd — called from the JSON write callback.
 *
 * Recognized commands (written to JSON characteristic by the phone):
 *   {"cmd":"stream_start","window":20}
 *   {"cmd":"ack","next":20}
 *   {"cmd":"nack","window_start":20,"missing":[22,27]}
 *
 * Returns TRUE if the command was recognized and acted on.
 * ----------------------------------------------------------------------- */
bool_t OpusStreamOnJsonCmd(const char *json, uint16_t len)
{
  (void)len;

  if (!g_notifyEnabled || g_connId == DM_CONN_ID_NONE)
  {
    return FALSE;
  }

  /* ---- stream_start (manual replay; auto-send happens via
   *      OpusStreamStartAuto + OpusStreamCccState) ---- */
  if (jsonMatchStr(json, "cmd", "stream_start"))
  {
    int win = 0;
    if (!jsonGetInt(json, "window", &win) || win <= 0)
    {
      win = OPUS_DEFAULT_WINDOW;
    }

    g_pendingAutoStart = FALSE;
    if (!opusKickOffStream((uint16_t)win))
    {
      am_util_debug_printf("OpusStream: stream_start but no recording\r\n");
      return FALSE;
    }
    return TRUE;
  }

  /* ---- ack ---- */
  if (jsonMatchStr(json, "cmd", "ack"))
  {
    if (g_streamState == STREAM_IDLE)
    {
      return FALSE;
    }

    int next = 0;
    if (!jsonGetInt(json, "next", &next))
    {
      return FALSE;
    }

    if ((uint16_t)next >= g_totalPackets)
    {
      g_streamState      = STREAM_IDLE;
      g_pendingAutoStart = FALSE;
      opusSchedule(0);
      am_util_debug_printf("OpusStream: transfer complete\r\n");
      return TRUE;
    }

    g_windowStart = (uint16_t)next;
    g_windowSent  = 0;
    g_streamState = STREAM_SENDING_WINDOW;

    am_util_debug_printf("OpusStream: ack, next window at pkt %u\r\n",
                         (unsigned)next);
    opusSchedule(1);
    return TRUE;
  }

  /* ---- nack ---- */
  if (jsonMatchStr(json, "cmd", "nack"))
  {
    if (g_streamState == STREAM_IDLE)
    {
      return FALSE;
    }

    g_missingCount = jsonGetIntArray(json, "missing",
                                     g_missingList, OPUS_MAX_MISSING);
    g_missingIdx = 0;

    if (g_missingCount > 0)
    {
      g_streamState = STREAM_RESENDING;
      am_util_debug_printf("OpusStream: nack, resending %u packets\r\n",
                           (unsigned)g_missingCount);
      opusSchedule(1);
      return TRUE;
    }
    return FALSE;
  }

  return FALSE; /* unrecognized command */
}

/* -----------------------------------------------------------------------
 * OpusStreamTick — called from the WSF timer handler (fit_main.c).
 *
 * Sends one packet per call.  Schedules the next tick via the pump
 * callback: 8 ms between packets within a window, 5 s timeout while
 * waiting for an ACK, or 0 to stop.
 * ----------------------------------------------------------------------- */
void OpusStreamTick(void)
{
  if (!g_notifyEnabled || g_connId == DM_CONN_ID_NONE)
  {
    g_streamState = STREAM_IDLE;
    return;
  }

  /* ACK timeout — resend the entire window */
  if (g_streamState == STREAM_WAITING_ACK)
  {
    am_util_debug_printf("OpusStream: ACK timeout, resending window at pkt %u\r\n",
                         (unsigned)g_windowStart);
    g_windowSent  = 0;
    g_streamState = STREAM_SENDING_WINDOW;
    /* fall through to send first packet */
  }

  /* ---- Normal window send ---- */
  if (g_streamState == STREAM_SENDING_WINDOW)
  {
    uint16_t pktIdx = g_windowStart + g_windowSent;
    if (pktIdx >= g_totalPackets)
    {
      g_streamState = STREAM_IDLE;
      return;
    }

    bool_t lastInWindow = (g_windowSent == (uint16_t)(g_windowSize - 1U))
                       || (pktIdx == (uint16_t)(g_totalPackets - 1U));

    uint8_t extraFlags = lastInWindow ? OPUS_FLAG_WINDOW_END : 0;
    opusSendPacketAtIndex(pktIdx, extraFlags);
    g_windowSent++;

    if (lastInWindow)
    {
      g_streamState = STREAM_WAITING_ACK;
      opusSchedule(OPUS_ACK_TIMEOUT_MS);
      am_util_debug_printf("OpusStream: window sent (pkts %u-%u / %u)\r\n",
                           (unsigned)g_windowStart, (unsigned)pktIdx,
                           (unsigned)g_totalPackets);
    }
    else
    {
      opusSchedule(OPUS_INTER_PKT_DELAY_MS);
    }
    return;
  }

  /* ---- Selective resend (NACK) ---- */
  if (g_streamState == STREAM_RESENDING)
  {
    if (g_missingIdx >= g_missingCount)
    {
      g_streamState = STREAM_WAITING_ACK;
      opusSchedule(OPUS_ACK_TIMEOUT_MS);
      return;
    }

    uint16_t pktIdx    = g_missingList[g_missingIdx];
    bool_t   lastResend = (g_missingIdx == (uint16_t)(g_missingCount - 1U));
    uint8_t  extraFlags = lastResend ? OPUS_FLAG_WINDOW_END : 0;

    opusSendPacketAtIndex(pktIdx, extraFlags);
    g_missingIdx++;

    if (lastResend)
    {
      g_streamState = STREAM_WAITING_ACK;
      opusSchedule(OPUS_ACK_TIMEOUT_MS);
    }
    else
    {
      opusSchedule(OPUS_INTER_PKT_DELAY_MS);
    }
    return;
  }

  /* STREAM_IDLE — nothing to do */
}
