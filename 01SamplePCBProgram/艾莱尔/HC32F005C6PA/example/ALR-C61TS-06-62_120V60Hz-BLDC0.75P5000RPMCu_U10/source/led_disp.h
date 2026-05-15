/*
 * led_disp.h
 *
 *  Created on: 2019Äê11ÔÂ27ÈÕ
 *      Author: Administrator
 */

#ifndef __LED_DISP_H__
#define __LED_DISP_H__
#include "common.h"

#define LED_ON (1)
#define LED_OFF (0)

typedef enum _SEG_MODE_E
{
	SEG_MODE_NORMAL=1,
	SEG_MODE_FLASH_SELF,
	SEG_MODE_FLASH_TOGGLE,
}SEG_MODE_E;

#define SEG_FLASH_FREQ  (3)//unit: 100ms

extern void led_display_init(void);
extern void seg_disp_proc(void);
extern void set_seg_mode(u8 mode);
extern void set_seg_val(u16 val);
extern void set_bit_seg(u8 index, u8 dat);

extern void set_seg_two(u8 hi2,u8 lo2);
extern void set_seg_two_spd(u8 hi2,u8 lo2);
extern void set_seg_two_sc(u8 hi2,u8 lo2);

//extern void set_seg_toggle_val(u16 val0,u16 val1);
//extern void set_seg_flash_cnt(u8 cnt);
//extern void set_bit_led(u8 index,u8 status);
#endif /* SOFTWARE_BSP_LED_DISP_H_ */
