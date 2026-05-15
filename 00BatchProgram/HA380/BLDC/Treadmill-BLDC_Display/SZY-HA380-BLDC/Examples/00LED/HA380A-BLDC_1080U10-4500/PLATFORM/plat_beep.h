/*
 * plat_beep.h  -  Buzzer sound presets (HNR-1407A, 4000 Hz resonant)
 *
 * For configurable beep count call Beep_Play_Custom() directly.
 * Example — 10-beep error alarm:
 *   Beep_Play_Custom(2000, 50, 100, 100, TM_ERROR_BEEP_COUNT);
 * Example — safety-key continuous alarm (255 repeats ≈ 51 s):
 *   Beep_Play_Custom(2000, 50, 100, 100, 255);
 */
#ifndef __PLAT_BEEP_H
#define __PLAT_BEEP_H

#include "main.h"

typedef enum {
    BEEP_CLICK = 0,   /* key click    : short tone */
    BEEP_ALARM,       /* error alarm  : repeated beeps  */
    BEEP_POWER_ON,    /* power-on     : long tone  */
    BEEP_PAIR_OK,     /* RF 对码成功 : 2kHz 双长声    */
} BeepMode_t;

void Beep_Init(void);
void Beep_Handler_1ms(void);
void Beep_Play(BeepMode_t mode);
void Beep_Play_Custom(uint16_t freq, uint8_t vol, uint16_t on, uint16_t off, uint8_t count);

#endif /* __PLAT_BEEP_H */
