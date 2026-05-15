#ifndef __PLAT_KEYFUN_H
#define __PLAT_KEYFUN_H

#include "main.h"

/* 1: 启用 plat_touchkey.c（触控 ADC）；0: 关闭 */
#ifndef PLAT_USE_TOUCHKEY
#define PLAT_USE_TOUCHKEY   1
#endif
/* 1: 433 解码；触控与射频同时为 1 时触控优先 */
#ifndef PLAT_USE_RF433
#define PLAT_USE_RF433   0
#endif

typedef enum {
    K_ID_NONE = 0,
    K_ID_UP,
    K_ID_DN,
    K_ID_ONOFF,
    K_ID_MODE,
    K_ID_3,
    K_ID_6,
    K_ID_9,
    K_ID_12,
    K_ID_P     /* 程式键 P01-P08，见 plat_touchkey 映射 */
} KeyID_t;

typedef enum {
    K_EVT_PRESS = 0,
    K_EVT_HOLD,
    K_EVT_RELEASE
} KeyEvt_t;

void Keypad_Handler_10ms(void);
void Keypad_On_Event(KeyID_t id, KeyEvt_t evt);

#endif
