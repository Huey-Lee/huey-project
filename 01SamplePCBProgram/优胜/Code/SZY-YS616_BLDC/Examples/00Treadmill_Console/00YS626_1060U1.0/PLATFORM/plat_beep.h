#ifndef __PLAT_BEEP_H
#define __PLAT_BEEP_H

#include "main.h"

// 音色预设
typedef enum {
    BEEP_CLICK = 0,    // 短促：2.7KHz, 50ms (按键)
    BEEP_LIMIT,        // 异音：4.5KHz, 150ms (达到极值)
    BEEP_ALARM,        // 警报：3.3KHz, 100ms响/100ms停, 10次 (报错)
    BEEP_POWER_ON,     // 开机：1.5KHz, 500ms (上电)
} BeepMode_t;

void Beep_Init(void);
void Beep_Handler_1ms(void); 
void Beep_Play(BeepMode_t mode); // 极简调用
void Beep_Play_Custom(uint16_t freq, uint8_t vol, uint16_t on, uint16_t off, uint8_t count);

#endif
