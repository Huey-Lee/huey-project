/* user_timer.h - tim_t, PWM, timer 0/6/2 control. */
#ifndef USER_TIMER_H
#define USER_TIMER_H
#include "bt.h"
#include "hc32f005.h"
#include "base_types.h"
#include "adt.h"
/******************************************************
*
*
*ȫ�ֱ���
*
*
******************************************************/
#define tim2_cnt 	0XFFAA

#define tim6_10ms 110
#define tim6_100ms 1100
#define tim6_200ms 2200
#define tim6_change_speed_time 2000
/* MT_START_PWM: use motor.h (must match) */
typedef struct _t_tim
{
	uint16_t tim2_10ms;
  uint16_t tim6_cur;
}tim_t;

/******************************************************
*
*
*��������
*
*
******************************************************/
void user_timer_init(void);
void tim0run(void);
void tim0stop(void);
void tim2run(void);
void tim6run(void);
void tim6stop(void);
#endif
