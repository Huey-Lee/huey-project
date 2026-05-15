/*
 * Author: HUEY
 * Date: 2024-08-04
 */

#ifndef __KEYPAD_FUN__H__
#define __KEYPAD_FUN__H__
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
/**
 * Ó²¼þÎïÀíÓ³Éä
 */
#define  BTN_MODE     (KEYPAD_BUTTON_0)
#define  BTN_START    (KEYPAD_BUTTON_1)
#define  BTN_ADD      (KEYPAD_BUTTON_2)
#define  BTN_SUB      (KEYPAD_BUTTON_3)
#define  BTN_NC       (KEYPAD_BUTTON_5)
#define  BTN_STOP     (KEYPAD_BUTTON_6)
#define  BTN_PROGRAM  (KEYPAD_BUTTON_7)
//#define  BTN_ADD    (KEYPAD_BUTTON_3)
//#define  BTN_ONOFF  (KEYPAD_BUTTON_2)
//#define  BTN_MODE   (KEYPAD_BUTTON_1)
//#define  BTN_SUB    (KEYPAD_BUTTON_0)

#define BTN_ALL_ON       (0XFF)
#define BTN_ALL_OFF      (0X00)

#define BLINK_HOLD_TIME 5    // 500ms

extern _Bool PAUSE_FLAG;
extern _Bool write_flag;
extern void keypad_m_key_release_poll(u8 raw_keys);

extern  void keypad_short_proc(void);
extern  void keypad_long_proc(void);
extern  void keypad_continue_proc(void);
extern  void keypad_all_realse_proc(void);
void Protected_mode(void);
void Protected_mode2(void);

extern  void beep_foha(void);
extern void update_display_mode(void);
extern u16 old_bl;
#endif /* BSP_KEYPAD_FUN_H_ */
