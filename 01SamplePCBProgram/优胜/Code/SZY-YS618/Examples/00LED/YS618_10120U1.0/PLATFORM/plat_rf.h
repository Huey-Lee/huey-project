#ifndef __PLAT_RF_H
#define __PLAT_RF_H

#include "main.h"
#include <stdbool.h>

typedef enum {
    RF_TYPE_NONE = 0,
    RF_TYPE_24BIT,      // 协议1 (16位ID + 8位键值)
    RF_TYPE_32BIT       // 协议2 (16位ID + 8位键值 + 8位校验)
} RF_Type_t;

typedef struct {
    uint32_t addr;
    uint8_t  key;
    RF_Type_t type;
    volatile bool vld;  // 原子性标志位
} RF_Msg_t;

extern RF_Msg_t g_rf_msg;

void RF433_Handler_60us(void);

#endif

