/* user_timer.h - tim_t, PWM, timer 0/6/2 control. */
#ifndef USER_TIMER_H
#define USER_TIMER_H
#include "bt.h"
#include "hc32f005.h"
#include "base_types.h"
#include "adt.h"
#include "motor.h"
/******************************************************
*
*
*ȫ�ֱ���
*
*
******************************************************/
#define tim2_cnt 	0XFFAA

#define tim6_10ms   (((MOTOR_TIM6_ISR_HZ_APPROX)*(10u))/(1000u))
#define tim6_100ms  (((MOTOR_TIM6_ISR_HZ_APPROX)*(100u))/(1000u))
#define tim6_200ms  (((MOTOR_TIM6_ISR_HZ_APPROX)*(200u))/(1000u))
/* ISR 斜坡步：与旧版 11009 Hz × 2000 步的近 ~182 ms 墙钟时间等价 */
#define tim6_change_speed_time (((2000ul)*(MOTOR_TIM6_ISR_HZ_APPROX)+(11009ul/2))/11009ul)
/* MT_START_PWM: use motor.h (must match) */
typedef struct _t_tim
{
	uint16_t tim2_10ms;
	volatile uint16_t tim6_cur;  /* Tim6 UDF ISR 递增；主循环读清，须 volatile */
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
