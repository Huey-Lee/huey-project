#ifndef __PLAT_RF_H
#define __PLAT_RF_H

#include "main.h"
#include <stdbool.h>

typedef struct {
    uint32_t addr;
    uint8_t  key;
    volatile bool vld;  /* 收到一帧 24 位码后置位，由 Keypad_Handler_10ms 消费后清除 */
} RF_Msg_t;

extern RF_Msg_t g_rf_msg;

void RF433_Handler_60us(void);

#endif
