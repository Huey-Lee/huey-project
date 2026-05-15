#ifndef __PLAT_GPIO_KEYS_H__
#define __PLAT_GPIO_KEYS_H__

#include "plat_touchkey.h"

/** 弹簧四键（低有效、上拉）：PB00=− ，PB01=+ ，PA03=ON ，PA04=OFF — 不占 PA05，可与两路触摸并存 */
void GpioKeys_Init(void);
KeyID_t GpioKeys_Handler_10ms(void);

#endif
