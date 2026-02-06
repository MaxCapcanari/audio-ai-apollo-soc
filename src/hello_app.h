#ifndef HELLO_APP_H
#define HELLO_APP_H
#pragma once
#include "wsf_types.h"
#include "wsf_msg.h"


void HelloAppHandlerInit(wsfHandlerId_t handlerId);
void HelloAppHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);
void HelloAppStart(void);

#endif
