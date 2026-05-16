#ifndef USER_TIMER_H
#define USER_TIMER_H

#include "motor.h"

#ifndef BDC_PWM_PRESCALER
#define BDC_PWM_PRESCALER (7u)
#endif

#define tim6_10ms   (((MOTOR_TIM6_ISR_HZ_APPROX) * (10u)) / (1000u))
#define tim6_100ms  (((MOTOR_TIM6_ISR_HZ_APPROX) * (100u)) / (1000u))
#define tim6_200ms  (((MOTOR_TIM6_ISR_HZ_APPROX) * (200u)) / (1000u))
#define tim6_change_speed_time \
    (((2000ul) * (MOTOR_TIM6_ISR_HZ_APPROX) + (11009ul / 2)) / 11009ul)

typedef struct _t_tim {
    uint16_t tim2_10ms;
    volatile uint16_t tim6_cur;
} tim_t;

extern tim_t tim_handle;

void user_timer_init(void);
void tim0run(void);
void tim0stop(void);
void tim2run(void);
void tim6run(void);
void tim6stop(void);

#endif
