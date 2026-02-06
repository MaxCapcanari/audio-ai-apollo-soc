#include <stdint.h>
#include <string.h>

#include "wsf_types.h"
#include "wsf_msg.h"

#include "am_util.h"
#include "app_api.h"
#include "app_main.h"
#include "att_api.h"
#include "dm_api.h"
#include "svc_core.h"
#include "hello_app.h"


#define HELLO_ADV_HANDLE 0
#define HELLO_DEVICE_NAME "EatingAnalytics_A4B"

static wsfHandlerId_t g_helloHandlerId;

static const uint8_t g_advData[] = {2,
                                    DM_ADV_TYPE_FLAGS,
                                    DM_FLAG_LE_GENERAL_DISC |
                                        DM_FLAG_LE_BREDR_NOT_SUP,

                                    17,
                                    DM_ADV_TYPE_128_UUID_PART,
                                    0xAB,
                                    0x90,
                                    0x78,
                                    0x56,
                                    0x34,
                                    0x12,
                                    0x34,
                                    0x12,
                                    0x34,
                                    0x12,
                                    0x78,
                                    0x56,
                                    0x34,
                                    0x12,
                                    0x34,
                                    0x12};

static uint8_t g_scanData[32];

static void printBdAddr(const uint8_t *pAddr) {
  am_util_debug_printf("%02X:%02X:%02X:%02X:%02X:%02X", pAddr[5], pAddr[4],
                       pAddr[3], pAddr[2], pAddr[1], pAddr[0]);
}

static uint8_t buildScanData(uint8_t *pBuf, uint8_t maxLen) {
  const uint8_t nameLen = (uint8_t)strlen(HELLO_DEVICE_NAME);
  const uint8_t needed = (uint8_t)(2 + nameLen);

  if (needed > maxLen) {
    return 0;
  }

  pBuf[0] = (uint8_t)(1 + nameLen);
  pBuf[1] = DM_ADV_TYPE_LOCAL_NAME;
  memcpy(&pBuf[2], HELLO_DEVICE_NAME, nameLen);

  return needed;
}

static void helloDmCback(dmEvt_t *pDmEvt) {
  dmEvt_t *pMsg = WsfMsgAlloc(sizeof(dmEvt_t));
  if (pMsg) {
    memcpy(pMsg, pDmEvt, sizeof(dmEvt_t));
    WsfMsgSend(g_helloHandlerId, pMsg);
  }
}

static void helloAttCback(attEvt_t *pAttEvt) {
  attEvt_t *pMsg = WsfMsgAlloc(sizeof(attEvt_t));
  if (pMsg) {
    memcpy(pMsg, pAttEvt, sizeof(attEvt_t));
    WsfMsgSend(g_helloHandlerId, pMsg);
  }
}

static void helloSetupAdv(void) {
  uint8_t scanLen = buildScanData(g_scanData, sizeof(g_scanData));

  DmAdvSetData(HELLO_ADV_HANDLE, 0, DM_DATA_LOC_ADV, (uint8_t)sizeof(g_advData),
               (uint8_t *)g_advData);

  am_util_debug_printf("HELLO: set ADV data (%d bytes)\r\n",
                       (int)sizeof(g_advData));
  am_util_debug_printf("HELLO: set SCAN data (%d bytes)\r\n", (int)scanLen);

  if (scanLen > 0) {
    DmAdvSetData(HELLO_ADV_HANDLE, 0, DM_DATA_LOC_SCAN, scanLen, g_scanData);
  }
}

void HelloAppHandlerInit(wsfHandlerId_t handlerId) {
  am_util_debug_printf("HELLO: HandlerInit called\r\n");

  g_helloHandlerId = handlerId;

  DmRegister(helloDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, helloDmCback);

  AttRegister(helloAttCback);
  AttConnRegister(AppServerConnCback);

  AppSlaveInit();

  SvcCoreAddGroup();

  AttsSetAttr(GAP_DN_HDL, (uint8_t)strlen(HELLO_DEVICE_NAME),
              (uint8_t *)HELLO_DEVICE_NAME);

  am_util_debug_printf("HELLO: init\r\n");
  am_util_debug_printf("HELLO: device name = %s\r\n", HELLO_DEVICE_NAME);
  am_util_debug_printf(
      "HELLO: adv payload has 128-bit UUID (partial list)\r\n");
}

void HelloAppStart(void) {
  am_util_debug_printf("HELLO: DmDevReset()...\r\n");
  DmDevReset();
}

void HelloAppHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  (void)event;

  dmEvt_t *pDmEvt = (dmEvt_t *)pMsg;

  switch (pDmEvt->hdr.event)
  {
    case DM_RESET_CMPL_IND:
      am_util_debug_printf("HELLO: DM reset complete\r\n");
      helloSetupAdv();
      am_util_debug_printf("HELLO: advertising data set (name in scan response)\r\n");
      AppAdvStart(APP_MODE_DISCOVERABLE);
      am_util_debug_printf("HELLO: advertising start requested\r\n");
      break;

    case DM_ADV_START_IND:
      am_util_debug_printf("HELLO: DM_ADV_START_IND\r\n");
      break;

    case DM_ADV_STOP_IND:
      am_util_debug_printf("HELLO: DM_ADV_STOP_IND (restarting advertising)\r\n");
      AppAdvStart(APP_MODE_DISCOVERABLE);
      break;

    case DM_CONN_OPEN_IND:
      am_util_debug_printf("HELLO: connected to ");
      printBdAddr(pDmEvt->connOpen.peerAddr);
      am_util_debug_printf("\r\n");
      break;

    case DM_CONN_CLOSE_IND:
      am_util_debug_printf("HELLO: disconnected, reason=0x%02X\r\n",
                           pDmEvt->connClose.reason);
      break;

    default:
      break;
  }

  AppSlaveProcDmMsg(pDmEvt);
  AppSlaveSecProcDmMsg(pDmEvt);
}

