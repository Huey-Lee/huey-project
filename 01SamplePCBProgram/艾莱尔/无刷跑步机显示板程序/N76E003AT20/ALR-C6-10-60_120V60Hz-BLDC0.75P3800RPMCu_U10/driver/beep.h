/*
 * beep.h
 *
 *  Created on: 2019Äê7ÔÂ3ÈÕ
 *      Author: Administrator
 */

#ifndef DRIVER_BEEP_H_
#define DRIVER_BEEP_H_
#include "common.h"


#define DEFAULT_ON_TICKS  (10)//unit:10ms
#define DEFAULT_OFF_TICKS (1)

#define STOP_BEEP_CNT        (5)
#define STOP_BEEP_ON_TICKS   (50)
#define STOP_BEEP_OFF_TICKS  (150)


#define ERROR_ON_TICKS  (300)//unit:10ms
#define ERROR_OFF_TICKS (50)

#define BEEP_SWITCH_ON      0   
#define BEEP_SWITCH_OFF     1 

#define BEEP_SWITCH_ON_1_CNT()   {         \
                                   if(treadmills.param.beep_status == BEEP_SWITCH_ON)  \
                                    {\
                                          beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);    \
                                    }\
                                 }

extern void beep_sound_proc(void);
extern void beep_set(u8 count,u16 on_ticks,u16 off_ticks);
extern void beep_sound_proc1(void);
extern void beep_set1(u8 count,u16 on_ticks,u16 off_ticks);
extern void beep_force_stop(void);
extern u8 is_disable_beep;
#endif /* DRIVER_BEEP_H_ */
