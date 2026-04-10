#include <string.h>
#include <stdio.h>

#include "wsf_types.h"
#include "att_api.h"
#include "dm_api.h"

#include "FreeRTOS.h"
#include "task.h"

#include "am_util.h"
#include "ble_opus_stream_svc.h"

#define HELLO_SVC_START_HDL  0x9000
#define HELLO_VAL_HDL        (HELLO_SVC_START_HDL + 2)
#define HELLO_CCC_HDL        (HELLO_SVC_START_HDL + 3)
#define HELLO_SVC_END_HDL    (HELLO_SVC_START_HDL + 3)

static bool_t g_helloGroupAdded = FALSE;

static const uint8_t helloSvcUuid[ATT_128_UUID_LEN] =
{
  0xAB,0x90,0x78,0x56,0x34,0x12,0x34,0x12,
  0x34,0x12,0x78,0x56,0x34,0x12,0x34,0x12
};

static const uint8_t helloValUuid[ATT_128_UUID_LEN] =
{
  0xAC,0x90,0x78,0x56,0x34,0x12,0x34,0x12,
  0x34,0x12,0x78,0x56,0x34,0x12,0x34,0x12
};

static uint8_t helloChDecl[1 + 2 + ATT_128_UUID_LEN] =
{
  (ATT_PROP_READ | ATT_PROP_WRITE | ATT_PROP_NOTIFY),
  (uint8_t)(HELLO_VAL_HDL & 0xFF),
  (uint8_t)((HELLO_VAL_HDL >> 8) & 0xFF),
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

static uint16_t helloSvcUuidLen = sizeof(helloSvcUuid);
static uint16_t helloChDeclLen  = sizeof(helloChDecl);

static uint8_t  helloValBuf[128];
static uint16_t helloValLen = 0;

static uint8_t  helloCccVal[2] = { 0x00, 0x00 };
static uint16_t helloCccLen = sizeof(helloCccVal);

static uint8_t helloWriteCback(dmConnId_t connId, uint16_t handle,
                               uint8_t operation, uint16_t offset,
                               uint16_t len, uint8_t *pValue,
                               attsAttr_t *pAttr);

static attsAttr_t helloSvcList[] =
{
  {
    (uint8_t *)attPrimSvcUuid,
    (uint8_t *)helloSvcUuid,
    &helloSvcUuidLen,
    sizeof(helloSvcUuid),
    0,
    ATTS_PERMIT_READ
  },
  {
    (uint8_t *)attChUuid,
    helloChDecl,
    &helloChDeclLen,
    sizeof(helloChDecl),
    0,
    ATTS_PERMIT_READ
  },
  {
    (uint8_t *)helloValUuid,
    helloValBuf,
    &helloValLen,
    sizeof(helloValBuf),
    (ATTS_SET_VARIABLE_LEN | ATTS_SET_WRITE_CBACK),
    (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE)
  },
  {
    (uint8_t *)attCliChCfgUuid,
    helloCccVal,
    &helloCccLen,
    sizeof(helloCccVal),
    ATTS_SET_CCC,
    (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE)
  }
};

static attsGroup_t helloSvcGroup =
{
  NULL,
  helloSvcList,
  NULL,
  helloWriteCback,
  HELLO_SVC_START_HDL,
  HELLO_SVC_END_HDL
};

static dmConnId_t g_connId = DM_CONN_ID_NONE;
static bool_t     g_notifyEnabled = FALSE;

static bool_t helloLooksLikeJsonObject(const char *pStr, uint16_t len)
{
  uint16_t start = 0;
  int end = (int)len - 1;

  while ((start < len) &&
         ((pStr[start] == ' ') || (pStr[start] == '\r') ||
          (pStr[start] == '\n') || (pStr[start] == '\t')))
  {
    start++;
  }

  while ((end >= 0) &&
         ((pStr[end] == ' ') || (pStr[end] == '\r') ||
          (pStr[end] == '\n') || (pStr[end] == '\t')))
  {
    end--;
  }

  if (start >= len)
  {
    return FALSE;
  }

  if (end < 0)
  {
    return FALSE;
  }

  return (pStr[start] == '{') && (pStr[end] == '}');
}

static void helloSetJsonAckPong(void)
{
  int n = snprintf((char *)helloValBuf,
                   sizeof(helloValBuf),
                   "{\"device\":\"Apollo4\",\"status\":\"ok\",\"reply\":\"pong\"}");

  if ((n < 0) || ((size_t)n >= sizeof(helloValBuf)))
  {
    return;
  }

  helloValLen = (uint16_t)n;
  AttsSetAttr(HELLO_VAL_HDL, helloValLen, helloValBuf);
}

static void helloSetJsonAckLen(uint16_t len)
{
  int n = snprintf((char *)helloValBuf,
                   sizeof(helloValBuf),
                   "{\"device\":\"Apollo4\",\"status\":\"ok\",\"received_len\":%u}",
                   (unsigned)len);

  if ((n < 0) || ((size_t)n >= sizeof(helloValBuf)))
  {
    return;
  }

  helloValLen = (uint16_t)n;
  AttsSetAttr(HELLO_VAL_HDL, helloValLen, helloValBuf);
}

static void helloSetJsonError(const char *reason)
{
  int n = snprintf((char *)helloValBuf,
                   sizeof(helloValBuf),
                   "{\"device\":\"Apollo4\",\"status\":\"error\",\"reason\":\"%s\"}",
                   reason);

  if ((n < 0) || ((size_t)n >= sizeof(helloValBuf)))
  {
    return;
  }

  helloValLen = (uint16_t)n;
  AttsSetAttr(HELLO_VAL_HDL, helloValLen, helloValBuf);
}

static void helloSetJsonHello(const char *deviceName)
{
  int n = snprintf((char *)helloValBuf,
                   sizeof(helloValBuf),
                   "{\"device\":\"%s\",\"message\":\"hello\"}",
                   deviceName);

  if (n < 0) return;
  if ((size_t)n >= sizeof(helloValBuf)) return;

  helloValLen = (uint16_t)n;
  AttsSetAttr(HELLO_VAL_HDL, helloValLen, helloValBuf);
}

static void helloNotify(void)
{
  if (!g_notifyEnabled) return;
  if (g_connId == DM_CONN_ID_NONE) return;
  if (helloValLen == 0) return;

  AttsHandleValueNtf(g_connId, HELLO_VAL_HDL, helloValLen, helloValBuf);
}

static uint8_t helloWriteCback(dmConnId_t connId, uint16_t handle,
                               uint8_t operation, uint16_t offset,
                               uint16_t len, uint8_t *pValue,
                               attsAttr_t *pAttr)
{
  char rxBuf[sizeof(helloValBuf)];

  (void)operation;
  (void)pAttr;

  if (handle != HELLO_VAL_HDL)
  {
    return ATT_SUCCESS;
  }

  if (offset != 0)
  {
    return ATT_ERR_OFFSET;
  }

  if ((len == 0) || (len >= sizeof(rxBuf)))
  {
    return ATT_ERR_LENGTH;
  }

  memcpy(rxBuf, pValue, len);
  rxBuf[len] = '\0';

  g_connId = connId;

  am_util_debug_printf("HelloJson RX (%u bytes): %s\r\n", (unsigned)len, rxBuf);

  if (!helloLooksLikeJsonObject(rxBuf, len))
  {
    helloSetJsonError("invalid_json");
    helloNotify();
    return ATT_SUCCESS;
  }

  /* Forward stream-control commands to the Opus service */
  if (strstr(rxBuf, "\"stream_start\"") != NULL ||
      strstr(rxBuf, "\"nack\"") != NULL ||
      (strstr(rxBuf, "\"ack\"") != NULL && strstr(rxBuf, "\"next\"") != NULL))
  {
    bool_t ok = OpusStreamOnJsonCmd(rxBuf, len);
    if (ok)
    {
      helloSetJsonAckLen(len);
    }
    else
    {
      helloSetJsonError("stream_not_ready");
    }
    helloNotify();
    return ATT_SUCCESS;
  }

  if ((strstr(rxBuf, "\"cmd\"") != NULL) && (strstr(rxBuf, "\"ping\"") != NULL))
  {
    helloSetJsonAckPong();
  }
  else
  {
    helloSetJsonAckLen(len);
  }

  helloNotify();
  return ATT_SUCCESS;
}

void HelloJsonSvcAdd(void)
{
  if (g_helloGroupAdded)
  {
    return;
  }
  g_helloGroupAdded = TRUE;

  memcpy(&helloChDecl[3], helloValUuid, ATT_128_UUID_LEN);

  AttsAddGroup(&helloSvcGroup);

  helloSetJsonHello("Apollo4");
}



void HelloJsonCccState(dmConnId_t connId, bool_t enabled)
{
  /* Ignore CCC changes for other connections */
  if ((g_connId != DM_CONN_ID_NONE) && (connId != g_connId))
  {
    return;
  }

  g_connId = connId;
  g_notifyEnabled = enabled;

  if (enabled)
  {
    am_util_debug_printf("HelloJson: notify enabled (connId=%d)\r\n", connId);

    /* Prepare value, but DO NOT notify here */
    helloSetJsonHello("Apollo4");
  }
  else
  {
    am_util_debug_printf("HelloJson: notify disabled (connId=%d)\r\n", connId);
  }
}


void HelloJsonConnClose(dmConnId_t connId)
{
  if (g_connId == connId)
  {
    g_connId = DM_CONN_ID_NONE;
    g_notifyEnabled = FALSE;
  }
}

void HelloJsonOnButton0Pressed(void)
{
  helloSetJsonHello("Apollo4");
  am_util_debug_printf("Sending message: %s\r\n", (char *)helloValBuf);
  helloNotify();
}
