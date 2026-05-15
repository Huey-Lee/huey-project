#ifndef __PLAT_RF_STORE_H
#define __PLAT_RF_STORE_H

#include "main.h"

void RFStore_Init(void);
_Bool RFStore_IsBound(void);
uint16_t RFStore_GetAddr(void);
/* 返回 0 成功，非 0 失败（可重试对码） */
uint8_t RFStore_WritePairedAddr(uint16_t addr);

#endif
