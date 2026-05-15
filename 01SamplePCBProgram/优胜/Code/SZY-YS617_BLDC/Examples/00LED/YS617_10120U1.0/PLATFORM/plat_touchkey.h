#ifndef __PLAT_TOUCHKEY_H
#define __PLAT_TOUCHKEY_H

#include "main.h"
#include "plat_keyfun.h"

/**
 * 双 ADC 触摸（同一文件）：
 * - PA05 -> ADC_InputCH5（芯片手册：通道 5）
 * - PA02 -> ADC_InputCH2（芯片手册：通道 2 = PA02；若你原理图画的是 PA04 请改 .c 里 TK_ADC_CH_LINE_B）
 * 每路 6 个物理键：分档对应 prv_classify_pad 的 PAD0~PAD5（共 12 键）；KeyID 在 .c 里 s_keys_line_a/b[6] 自填。
 */

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

#endif
