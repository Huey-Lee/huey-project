#ifndef __PLAT_KEYFUN_H
#define __PLAT_KEYFUN_H

#include "main.h"

/* 1= PA05/ADC 触摸(XW09B)；0= 关闭触摸采样 */
#ifndef PLAT_USE_TOUCHKEY
#define PLAT_USE_TOUCHKEY   1
#endif
/* 1= 433 遥控参与；与触摸同时为 1 时：本周期有触摸档则优先触摸，否则再取射频 */
#ifndef PLAT_USE_RF433
#define PLAT_USE_RF433      0
#endif

typedef enum { 
	K_ID_NONE = 0, 
	K_ID_UP, 
	K_ID_DN, 
	K_ID_ONOFF, 
	K_ID_MODE 
} KeyID_t;

typedef enum { 
	K_EVT_PRESS = 0, 
	K_EVT_HOLD, 
	K_EVT_RELEASE 
} KeyEvt_t;

void Keypad_Handler_10ms(void); // 挂载在 10ms 调度任务
void Keypad_On_Event(KeyID_t id, KeyEvt_t evt); // 业务回调

/**
 * @brief 按键分发中心：由机制层在产生事件时自动回调
 * @param id  按键逻辑ID (UP, DN, ONOFF, MODE)
 * @param evt 事件类型 (PRESS, HOLD, RELEASE)
 */
void Keypad_On_Event(KeyID_t id, KeyEvt_t evt);

#endif

