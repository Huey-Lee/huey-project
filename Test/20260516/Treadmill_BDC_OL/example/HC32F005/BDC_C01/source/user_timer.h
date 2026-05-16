/*
 * Function: 定时器模块对外接口与 TIM6 节拍换算宏。
 * Method:   tim_t 保存 TIM2/TIM6 相关计数；tim6_* 由 MOTOR_TIM6_ISR_HZ_APPROX 推算毫秒级步长；声明 user_timer_init 与 TIM0/2/6 启停。
 * Name:     Huey
 * Date:     May 16, 2026 18:00
 */

#ifndef USER_TIMER_H
#define USER_TIMER_H
#include "bt.h"
#include "hc32f005.h"
#include "base_types.h"
#include "adt.h"
#include "motor.h"

#define tim2_cnt 	0XFFAA

#define tim6_10ms   (((MOTOR_TIM6_ISR_HZ_APPROX)*(10u))/(1000u))
#define tim6_100ms  (((MOTOR_TIM6_ISR_HZ_APPROX)*(100u))/(1000u))
#define tim6_200ms  (((MOTOR_TIM6_ISR_HZ_APPROX)*(200u))/(1000u))
#define tim6_change_speed_time (((2000ul)*(MOTOR_TIM6_ISR_HZ_APPROX)+(11009ul/2))/11009ul)

typedef struct _t_tim
{
	uint16_t tim2_10ms;
	volatile uint16_t tim6_cur;
}tim_t;

void user_timer_init(void);
void tim0run(void);
void tim0stop(void);
void tim2run(void);
void tim6run(void);
void tim6stop(void);
#endif
