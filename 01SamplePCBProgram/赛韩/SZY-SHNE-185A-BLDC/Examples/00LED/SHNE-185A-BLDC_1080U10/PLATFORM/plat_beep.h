/*
 * plat_beep.h  -  Buzzer (HA380A 风格：1 ms 节拍、CLICK 排队、报警次数 treadmills.h)
 */
#ifndef __PLAT_BEEP_H
#define __PLAT_BEEP_H

#include "main.h"

typedef enum {
    BEEP_CLICK = 0,
    BEEP_ALARM,
    BEEP_POWER_ON,
    BEEP_LONG,
    BEEP_PAIR_OK
} BeepMode_t;

void Beep_Init(void);
void Beep_Handler_1ms(void);
void Beep_Stop(void);
uint8_t Beep_IsBusy(void);
void Beep_Play(BeepMode_t mode);
void Beep_Play_Custom(uint16_t freq, uint8_t vol, uint16_t on, uint16_t off, uint8_t count);

#endif /* __PLAT_BEEP_H */
