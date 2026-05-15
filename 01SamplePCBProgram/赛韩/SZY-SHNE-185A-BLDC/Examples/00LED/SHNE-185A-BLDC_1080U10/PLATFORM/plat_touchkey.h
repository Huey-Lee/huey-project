#ifndef __PLAT_TOUCHKEY_H
#define __PLAT_TOUCHKEY_H

#include "main.h"
#include "plat_keyfun.h"

/** 触摸参数（10ms 为单位）。与 433 同时开启时仲裁见 plat_keyfun.h（PLAT_USE_RF433）。 */

typedef struct {
    uint8_t  debounce_ticks;   /* 连续同档次数才确认，建议 2~5 */
    uint8_t  sustain_ticks;    /* 按下维持计数，与 plat_keyfun 中 g_energy 刷新一致 */
    uint16_t hold_start_ticks; /* 长按开始前的累计 10ms 数 */
    uint8_t  repeat_ticks;     /* 长按连发间隔 10ms 数，越小越快 */
} Touchkey_Params_t;

void Touchkey_Init(void);
void Touchkey_SetParams(const Touchkey_Params_t *p);
void Touchkey_GetParams(Touchkey_Params_t *p);

/**
 * @brief 10ms 调用一次：内部 ADC 均值 + 分档消抖，输出与射频一致的 KeyID
 * @return 当前稳定识别的键；无键为 K_ID_NONE
 */
KeyID_t Touchkey_Handler_10ms(void);

uint16_t Touchkey_GetHoldStartTicks(void);
uint8_t  Touchkey_GetRepeatTicks(void);
uint8_t  Touchkey_GetSustainTicks(void);

#endif
