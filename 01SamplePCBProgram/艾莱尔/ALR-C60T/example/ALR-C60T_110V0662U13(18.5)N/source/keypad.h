#ifndef __KEYPAD_H__
#define __KEYPAD_H__
#include "common.h"



#define Reed_Val  Gpio_GetInputIO(GpioPort2, GpioPin5)


extern u8 key_cnt;
extern u8 key;
extern u8 key_mask;
extern u16 no_key_cnt;

extern u8 get_key_status(u8 btn);
extern void KeyPad_Scan(void);
void no_key_filter_check(void);

#endif /* SOFTWARE_USER_KEYPAD_H_ */
