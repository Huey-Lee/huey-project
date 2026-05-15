/*
 * keypad_fun.h
 *
 *  Created on: 2018年12月14日
 *      Author: Administrator
 */

#ifndef __keypad_fun__HH
#define __keypad_fun__HH
#include "common.h"
#include "keypad.h"

#define KEYPAD_BUTTON_0  (1<<0)
#define KEYPAD_BUTTON_1  (1<<1)
#define KEYPAD_BUTTON_2  (1<<2)
#define KEYPAD_BUTTON_3  (1<<3)
#define KEYPAD_BUTTON_4  (1<<4)
#define KEYPAD_BUTTON_5  (1<<5)
#define KEYPAD_BUTTON_6  (1<<6)
#define KEYPAD_BUTTON_7  (1<<7)

// 定义速度模式
#define SPEED_MODE_2KM  200
#define SPEED_MODE_4KM  400
#define SPEED_MODE_6KM  600


/**
 * 硬件物理映射
 */

//#define  BTN_ADD     (KEYPAD_BUTTON_0)
//#define  BTN_ONOFF   (KEYPAD_BUTTON_1)
//#define  BTN_SUB     (KEYPAD_BUTTON_2)

#define  BTN_ADD    (KEYPAD_BUTTON_0)
#define  BTN_ONOFF  (KEYPAD_BUTTON_1)
#define  BTN_MODE   (KEYPAD_BUTTON_2)
#define  BTN_SUB    (KEYPAD_BUTTON_3)

#define BTN_ALL_ON       (0XFF)
#define BTN_ALL_OFF      (0X00)

//extern bit PAUSE_FLAG;
extern  void keypad_short_proc(void);
extern  void keypad_long_proc(void);
extern  void keypad_continue_proc(void);
extern  void keypad_all_realse_proc(void);
//extern u16 old_bl;

#endif /* BSP_KEYPAD_FUN_H_ */
