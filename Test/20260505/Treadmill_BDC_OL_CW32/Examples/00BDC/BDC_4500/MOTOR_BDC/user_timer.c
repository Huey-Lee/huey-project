#include "user_timer.h"
#include "motor.h"
#include "bdc_hal.h"

volatile uint16_t ZHANKONBI = MT_START_PWM;
tim_t     tim_handle = {0};

void user_timer_init(void)
{
    bdc_pwm_hw_init();
}

void tim0run(void)
{
    bdc_relay_on();
}

void tim0stop(void)
{
    bdc_relay_off();
}

void tim2run(void)
{
    /* CW32：ADC 由 ATIM 节拍软件启动；保留接口兼容 user_gpio 键盘中断 */
}

void tim6run(void)
{
    bdc_pwm_enable();
}

void tim6stop(void)
{
    bdc_pwm_disable();
}
