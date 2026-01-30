#include <string.h>
#include "wsf_types.h"
#include "att_api.h"

#define HELLO_SVC_START_HDL  0x0100
#define HELLO_SVC_END_HDL    0x0103

#define HELLO_VAL_HDL        (HELLO_SVC_START_HDL + 2)

static const uint8_t helloSvcUuid[ATT_128_UUID_LEN] =
{
  0xAB,0x90,0x78,0x56,0x34,0x12,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x34,0x12
};

static const uint8_t helloValUuid[ATT_128_UUID_LEN] =
{
  0xAC,0x90,0x78,0x56,0x34,0x12,0x34,0x12, 0x34,0x12,0x78,0x56,0x34,0x12,0x34,0x12
};

static uint8_t helloChDecl[1 + 2 + ATT_128_UUID_LEN] =
{
  ATT_PROP_READ,
  (uint8_t)(HELLO_VAL_HDL & 0xFF),
  (uint8_t)((HELLO_VAL_HDL >> 8) & 0xFF),
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

static uint16_t helloSvcUuidLen = sizeof(helloSvcUuid);
static uint16_t helloChDeclLen  = sizeof(helloChDecl);
static uint8_t  helloValBuf[128];
static uint16_t helloValLen     = 0;

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
  }
};

static attsGroup_t helloSvcGroup =
{
  NULL,
  helloSvcList,
  NULL,
  NULL,
  HELLO_SVC_START_HDL,
  HELLO_SVC_END_HDL
};

static void helloSetJson(const char *deviceName)
{
  const char *pfx = "{\"device\":\"";
  const char *mid = "\",\"message\":\"hello\"}";

  uint16_t n1 = (uint16_t)strlen(pfx);
  uint16_t n2 = (uint16_t)strlen(deviceName);
  uint16_t n3 = (uint16_t)strlen(mid);

  if ((uint16_t)(n1 + n2 + n3) > sizeof(helloValBuf))
  {
    return;
  }

  memcpy(helloValBuf, pfx, n1);
  memcpy(helloValBuf + n1, deviceName, n2);
  memcpy(helloValBuf + n1 + n2, mid, n3);

  helloValLen = (uint16_t)(n1 + n2 + n3);
  AttsSetAttr(HELLO_VAL_HDL, helloValLen, helloValBuf);
}

void HelloJsonSvcAdd(void)
{
  memcpy(&helloChDecl[3], helloValUuid, ATT_128_UUID_LEN);

  helloSetJson("Apollo4");

  AttsAddGroup(&helloSvcGroup);
}
