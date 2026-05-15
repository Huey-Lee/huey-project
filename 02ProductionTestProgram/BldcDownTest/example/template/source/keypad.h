#ifndef __KEYPAD_H__
#define __KEYPAD_H__
#include "common.h"


extern u8 key_cnt;
extern u8 key;
extern u8 key_mask;

extern void KeyPad_Scan(void);

#endif /* SOFTWARE_USER_KEYPAD_H_ */
