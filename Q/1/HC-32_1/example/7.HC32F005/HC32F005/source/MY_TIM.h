#ifndef _MY_TIM_H
#define _MY_TIM_H
#include "bt.h"
#include "hc32f005.h"
#include "base_types.h"
#include "adt.h"
/******************************************************
*
*
*全局变量
*
*
******************************************************/
#define tim2_cnt 	0XFFAA

#define tim6_10ms 110
#define tim6_100ms 1100
#define tim6_200ms 2200
#define tim6_change_speed_time 2000

#define MT_START_PWM (25)
typedef struct _t_tim
{
	uint16_t tim2_10ms;
  uint16_t tim6_cur;
}tim_t;

/******************************************************
*
*
*函数申明
*
*
******************************************************/
void MY_TIM_INIT(void);
void tim0run(void);
void tim0stop(void);
void tim2run(void);
void tim6run(void);
void tim6stop(void);
#endif
