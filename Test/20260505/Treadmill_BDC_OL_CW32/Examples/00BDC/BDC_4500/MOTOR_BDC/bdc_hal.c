#include "bdc_hal.h"
#include "bdc_board.h"
#include "motor.h"
#include "user_timer.h"
#include "user_adc.h"

extern volatile uint16_t ZHANKONBI;

#include "cw32l010.h"
#include "cw32l010_gpio.h"
#include "cw32l010_sysctrl.h"
#include "cw32l010_atim.h"
#include "cw32l010_adc.h"
#include "cw32l010_systick.h"

#ifndef BDC_PWM_PRESCALER
#define BDC_PWM_PRESCALER (7u)
#endif

void delay10us(uint16_t n)
{
    while (n--) {
        FirmwareDelay(60);
    }
}

void delay1ms(uint32_t n)
{
    while (n--) {
        SysTickDelay(1);
    }
}

void bdc_hal_init(void)
{
    InitTick(48000000u);

    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pins  = BDC_LED_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.IT    = GPIO_IT_NONE;
    GPIO_Init(BDC_LED_PORT, &g);
    BDC_LED_OFF();

    g.Pins = BDC_RELAY_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.IT   = GPIO_IT_NONE;
    GPIO_Init(BDC_RELAY_PORT, &g);
    BDC_RELAY_OFF();
}

void bdc_relay_on(void)
{
    BDC_RELAY_ON();
}

void bdc_relay_off(void)
{
    BDC_RELAY_OFF();
}

uint8_t bdc_relay_on_readback(void)
{
    return (GPIO_ReadPin(BDC_RELAY_PORT, BDC_RELAY_PIN) != GPIO_Pin_RESET) ? 1u : 0u;
}

void bdc_pwm_hw_init(void)
{
    GPIO_InitTypeDef g = {0};

    __SYSCTRL_ATIM_CLK_ENABLE();
    __SYSCTRL_GPIOB_CLK_ENABLE();

    g.Pins  = GPIO_PIN_0;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.IT    = GPIO_IT_NONE;
    GPIO_Init(CW_GPIOB, &g);
    BDC_PWM_ATIM_CH1_AF();

    ATIM_DeInit();

    ATIM_InitTypeDef tim = {0};
    tim.BufferState          = ATIM_BUFFER_DISABLE;
    tim.CounterAlignedMode   = ATIM_COUNT_ALIGN_MODE_EDGE;
    tim.CounterDirection     = ATIM_COUNTING_UP;
    tim.CounterOPMode        = ATIM_OP_MODE_REPETITIVE;
    tim.Prescaler            = BDC_PWM_PRESCALER;
    tim.ReloadValue          = (uint32_t)(MOTOR_PWM_PERIOD_TICKS - 1u);
    tim.RepetitionCounter    = 0u;
    ATIM_Init(&tim);

    ATIM_OCInitTypeDef oc = {0};
    oc.OCPolarity         = ATIM_OCPOLARITY_NONINVERT;
    oc.OCNPolarity        = ATIM_OCPOLARITY_NONINVERT;
    oc.OCMode             = ATIM_OCMODE_PWM1;
    oc.OCFastMode         = ATIM_OC_FAST_MODE_DISABLE;
    oc.BufferState        = ENABLE;
    oc.OCInterruptState   = DISABLE;
    oc.OCComplement       = DISABLE;
    ATIM_OC1Init(&oc);

    {
        uint32_t prd = (uint32_t)MOTOR_PWM_PERIOD_TICKS;
        uint16_t ccr = (uint16_t)(prd - MT_START_PWM);
        ATIM_SetCompare1(ccr);
    }

    ATIM_CH1Config(DISABLE);
    ATIM_ITConfig(ATIM_IT_UIE, ENABLE);
    NVIC_SetPriority(ATIM_IRQn, 2);
    NVIC_EnableIRQ(ATIM_IRQn);
}

void bdc_pwm_set_compare(uint16_t duty_ticks)
{
    (void)duty_ticks;
}

void bdc_pwm_enable(void)
{
    ATIM_SetCounterValue(0);
    ATIM_CH1Config(ENABLE);
    ATIM_Cmd(ENABLE);
}

void bdc_pwm_disable(void)
{
    ATIM_Cmd(DISABLE);
    ATIM_CH1Config(DISABLE);
}

uint8_t bdc_pwm_running(void)
{
    return (CW_ATIM->CR1_f.CEN != 0u) ? 1u : 0u;
}

void bdc_adc_arm_for_pwm_tick(void)
{
}

/* HC32：Tim6 UDF 里用 OUT_PUT=PER−ZHANKONBI 更新比较寄存器；此处同等处理 */
void bdc_atim_pwm_isr(void)
{
    if (ATIM_GetITStatus(ATIM_STATE_UIF) != SET) {
        return;
    }
    ATIM_ClearITPendingBit(ATIM_STATE_UIF);

    {
        uint32_t prd = (uint32_t)MOTOR_PWM_PERIOD_TICKS;
        uint32_t cc  = prd - (uint32_t)ZHANKONBI;
        if (cc > prd) {
            cc = 0u;
        }
        ATIM_SetCompare1((uint16_t)cc);
    }

    tim_handle.tim6_cur++;
    ADC_SoftwareStartConvCmd(ENABLE);
}
