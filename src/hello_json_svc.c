#include <string.h>
#include <stdio.h>

#include "wsf_types.h"
#include "att_api.h"
#include "dm_api.h"

#include "FreeRTOS.h"
#include "task.h"

#include "am_util.h"

#define HELLO_SVC_START_HDL  0x0800
#define HELLO_VAL_HDL        (HELLO_SVC_START_HDL + 2)
#define HELLO_CCC_HDL        (HELLO_SVC_START_HDL + 3)
#define HELLO_SVC_END_HDL    (HELLO_SVC_START_HDL + 3)

#define HELLO_CCC_OFFSET   0
#define HELLO_CCC_TBL_LEN  (HELLO_CCC_OFFSET + 1)

static attsCccSet_t helloCccSet[] =
{
  { HELLO_CCC_HDL, ATT_CLIENT_CFG_NOTIFY, HELLO_CCC_OFFSET }
};

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
  (ATT_PROP_READ | ATT_PROP_NOTIFY),
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
    0,
    ATTS_PERMIT_READ
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


static uint16_t helloCccTbl[HELLO_CCC_TBL_LEN] = { 0 };


static attsGroup_t helloSvcGroup =
{
  NULL,
  helloSvcList,
  NULL,
  NULL,
  HELLO_SVC_START_HDL,
  HELLO_SVC_END_HDL
};

static dmConnId_t g_connId = DM_CONN_ID_NONE;
static bool_t     g_notifyEnabled = FALSE;

static void helloSetJsonWithUptime(const char *deviceName)
{
  uint32_t ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  uint32_t totalSeconds = ms / 1000U;
  uint32_t hours   = totalSeconds / 3600U;
  uint32_t minutes = (totalSeconds % 3600U) / 60U;
  uint32_t seconds = totalSeconds % 60U;

  int n = snprintf((char *)helloValBuf,
                   sizeof(helloValBuf),
                   "{\"device\":\"%s\",\"message\":\"hello\",\"uptime\":\"%02lu:%02lu:%02lu\"}",
                   deviceName,
                   (unsigned long)hours,
                   (unsigned long)minutes,
                   (unsigned long)seconds);

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

static void helloCccCback(attsCccEvt_t *pEvt)
{
  if (pEvt->handle != HELLO_CCC_HDL) return;

  g_connId = (dmConnId_t)pEvt->hdr.param;
  g_notifyEnabled = (pEvt->value == ATT_CLIENT_CFG_NOTIFY) ? TRUE : FALSE;

  if (g_notifyEnabled)
  {
    helloSetJsonWithUptime("Apollo4");
    am_util_debug_printf("HelloJson: notifications enabled, sending initial: %s\r\n", (char *)helloValBuf);
    helloNotify();
  }
  else
  {
    am_util_debug_printf("HelloJson: notifications disabled\r\n");
  }
}

static void helloDmCback(dmEvt_t *pEvt)
{
  switch (pEvt->hdr.event)
  {
    case DM_CONN_OPEN_IND:
      g_connId = (dmConnId_t)pEvt->hdr.param;
      AttsCccInitTable(g_connId, helloCccTbl);
      break;

    case DM_CONN_CLOSE_IND:
      g_connId = DM_CONN_ID_NONE;
      g_notifyEnabled = FALSE;
      break;

    default:
      break;
  }
}

void HelloJsonSvcAdd(void)
{
  memcpy(&helloChDecl[3], helloValUuid, ATT_128_UUID_LEN);

  helloSetJsonWithUptime("Apollo4");

  AttsAddGroup(&helloSvcGroup);

  AttsCccRegister((uint8_t)(sizeof(helloCccSet) / sizeof(helloCccSet[0])),
                  helloCccSet,
                  helloCccCback);
}


void HelloJsonOnButton0Pressed(void)
{
  helloSetJsonWithUptime("Apollo4");
  am_util_debug_printf("Sending message: %s\r\n", (char *)helloValBuf);
  helloNotify();
}

void HelloJsonRegisterConnCallback(uint8_t clientId)
{
  DmConnRegister(clientId, helloDmCback);
}


