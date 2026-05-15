#ifndef __PLAT_HR_TM998_H__
#define __PLAT_HR_TM998_H__

#include "main.h"

/**
 * TM998 OUT：与 P417A 参考机一致采用「下降沿」捕获 R-R（TIM1 CH1 下降沿）。
 * 若实测模块为空闲低、正脉冲，则改为 1 使用上升沿 + 高有效确认。
 */
#ifndef PLAT_HR_PULSE_ACTIVE_HIGH
#define PLAT_HR_PULSE_ACTIVE_HIGH 0
#endif

/** 离手无有效脉冲（ms）→ 与 P417 TIM1 2s ARR 行为对齐，清有效、心率送 0 */
#ifndef PLAT_HR_OFFHAND_MS
#define PLAT_HR_OFFHAND_MS 2000u
#endif

#ifndef PLAT_USE_HR_TM998
#define PLAT_USE_HR_TM998 1
#endif

#ifndef PLAT_HR_REQUIRE_PANEL_KEY
#define PLAT_HR_REQUIRE_PANEL_KEY 0
#endif

void HrTm998_Init(void);
void HrTm998_MsTick_1ms(void);
void HrTm998_GpioIrqHandler(void);
void HrTm998_OnKeypadScan_10ms(void);

#endif
