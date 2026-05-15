#ifndef __PLAT_TOUCHKEY_H
#define __PLAT_TOUCHKEY_H

#include "main.h"

/* 1: 启用 plat_touchkey.c（触控 ADC）；0: 关闭 */
#ifndef PLAT_USE_TOUCHKEY
#define PLAT_USE_TOUCHKEY   1
#endif
/* 1：PB00/PB01/PA03/PA04 弹簧键，与 PA02+PA05 双路触摸可同时使用 */
#ifndef PLAT_USE_GPIO_KEYS
#define PLAT_USE_GPIO_KEYS  1
#endif
/* 1: 5 字节按键/心跳协议上发显示板（plat_keylink）；0: 关闭 */
#ifndef PLAT_USE_KEYLINK
#define PLAT_USE_KEYLINK   1
#endif

typedef enum {
    K_ID_NONE = 0,
    K_ID_UP,        /* B 线：速度 + */
    K_ID_DN,        /* B 线：速度 - */
    K_ID_MODE,      /* A 线：M */
    K_ID_ON,        /* B 线：ON → 协议 START */
    K_ID_OFF,       /* A 线：OFF → 协议 STOP */
    K_ID_INC_UP,    /* A 线：坡度 + */
    K_ID_INC_DOWN,  /* A 线：坡度 - */
    K_ID_P,
    /* 里程快捷键；K_ID_HK_8 在 A/B 两线共用同一协议 0x0B */
    K_ID_HK_2,
    K_ID_HK_4,
    K_ID_HK_5,
    K_ID_HK_6,
    K_ID_HK_8,
    K_ID_HK_10,
    K_ID_HK_12
} KeyID_t;

typedef enum {
    K_EVT_PRESS = 0,
    K_EVT_HOLD,
    K_EVT_RELEASE
} KeyEvt_t;

typedef struct {
    uint8_t  debounce_ticks;
    uint8_t  sustain_ticks;
    uint16_t hold_start_ticks;
    uint8_t  repeat_ticks;
} Touchkey_Params_t;

void Touchkey_Init(void);
void Touchkey_SetParams(const Touchkey_Params_t *p);
void Touchkey_GetParams(Touchkey_Params_t *p);

/** 10ms：双路采样、分档、消抖；同时按下时优先 LINE_A（PA05） */
KeyID_t Touchkey_Handler_10ms(void);

uint16_t Touchkey_GetHoldStartTicks(void);
uint8_t  Touchkey_GetRepeatTicks(void);
uint8_t  Touchkey_GetSustainTicks(void);

/** 10ms 任务：触摸扫描 → 按下/长按/弹起 → Keylink + 本地状态 */
void Keypad_Handler_10ms(void);

#endif
