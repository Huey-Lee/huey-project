#ifndef __KEYPAD_FUN_H__
#define __KEYPAD_FUN_H__
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
#define  BTN_ADD    (KEYPAD_BUTTON_0)
#define  BTN_ONOFF  (KEYPAD_BUTTON_1)
#define  BTN_MODE   (KEYPAD_BUTTON_2)
#define  BTN_SUB    (KEYPAD_BUTTON_3)

//#define  BTN_ADD    (KEYPAD_BUTTON_0)
//#define  BTN_MODE   (KEYPAD_BUTTON_1)
//#define  BTN_SUB    (KEYPAD_BUTTON_2)
//#define  BTN_ONOFF  (KEYPAD_BUTTON_3)
//#define  BTN_SLEEP  (KEYPAD_BUTTON_4)

#define BTN_ALL_ON       (0XFF)
#define BTN_ALL_OFF      (0X00)

extern _Bool PAUSE_FLAG;


#define PARAME_SET_EN   (0)

extern _Bool write_flag;
extern  void keypad_short_proc(void);
extern  void keypad_long_proc(void);
extern  void keypad_continue_proc(void);
extern  void keypad_all_realse_proc(void);
void Protected_mode(void);
void Protected_mode2(void);

extern  void beep_foha(void);
//extern void bilingbuling(void);
extern u16 old_bl;
#endif /* BSP_KEYPAD_FUN_H_ */
