#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "main.h"

void BSP_PWM_Init(void);
void BSP_PWM_Set(uint16_t freq, uint8_t volume); // freq:Hz, volume:0-100
void BSP_PWM_Stop(void);


#endif
