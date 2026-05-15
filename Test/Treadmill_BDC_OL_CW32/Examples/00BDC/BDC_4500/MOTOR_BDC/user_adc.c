/* user_adc.c — CW32L010 ADC：三通道顺序采样（I / M− / M+），EOS 与参考工程 get_adc_value 对齐 */

#include "user_adc.h"
#include "bdc_board.h"
#include "motor.h"
#include "motor_drive.h"

#include "cw32l010_adc.h"
#include "cw32l010_sysctrl.h"

#include <string.h>

adc_t adc_handle;

u16 adc0data[GET_ADC_len];
u16 adc1data[GET_ADC_len];
u16 adc2data[GET_ADC_len];

static void adc_gpio_analog_inputs(void)
{
    GPIO_InitTypeDef g = {0};

    __SYSCTRL_GPIOA_CLK_ENABLE();
    g.Mode = GPIO_MODE_ANALOG;
    g.IT   = GPIO_IT_NONE;
    g.Pins = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_Init(CW_GPIOA, &g);
}

static void adc_fill_unused(ADC_InitTypeDef *s, uint32_t ch)
{
    s->ADC_IN3.ADC_InputChannel = ch;
    s->ADC_IN3.ADC_SampTime     = ADC_SampTime6Clk;
    s->ADC_IN4 = s->ADC_IN3;
    s->ADC_IN5 = s->ADC_IN3;
    s->ADC_IN6 = s->ADC_IN3;
    s->ADC_IN7 = s->ADC_IN3;
}

void clear_adcbuf(void)
{
    memset(adc0data, 0, sizeof(adc0data));
    memset(adc1data, 0, sizeof(adc1data));
    memset(adc2data, 0, sizeof(adc2data));
}

static void adc_hw_init_3ch(void)
{
    ADC_InitTypeDef s;
    memset(&s, 0, sizeof(s));

    s.ADC_ClkDiv            = ADC_Clk_Div2;
    s.ADC_ConvertMode       = ADC_ConvertMode_Once;
    s.ADC_SQREns            = ADC_SqrEns0to2;

    s.ADC_IN0.ADC_InputChannel = BDC_ADC_CH_I;
    s.ADC_IN0.ADC_SampTime     = ADC_SampTime18Clk;
    s.ADC_IN1.ADC_InputChannel = BDC_ADC_CH_MMINUS;
    s.ADC_IN1.ADC_SampTime     = ADC_SampTime18Clk;
    s.ADC_IN2.ADC_InputChannel = BDC_ADC_CH_MPLUS;
    s.ADC_IN2.ADC_SampTime     = ADC_SampTime18Clk;

    adc_fill_unused(&s, BDC_ADC_CH_I);

    ADC_Init(&s);
    ADC_Enable();
    ADC_ITConfig(ADC_IT_EOS, ENABLE);
    NVIC_SetPriority(ADC_IRQn, 3);
    NVIC_EnableIRQ(ADC_IRQn);
}

void Adc_Start(void)
{
    ADC_SoftwareStartConvCmd(ENABLE);
}

void user_adc_init(void)
{
    adc_gpio_analog_inputs();
    adc_hw_init_3ch();
    adc_handle.status  = SELECT_CH12;
    adc_handle.irq_flag = 0u;
}

void Get_Proc_loop(void)
{
}

void get_adc_value(void)
{
    static u8 flag;

    if (adc_handle.irq_flag >= GET_ADC_len) {
        adc_handle.irq_flag = 0u;

        motor.valtage_cur  = ADC_Value_Handle(adc0data, GET_ADC_len);
        motor.valtage_down = ADC_Value_Handle(adc1data, GET_ADC_len);
        motor.valtage_up   = ADC_Value_Handle(adc2data, GET_ADC_len);
        motor_drive_isr();
        flag = !flag;
        adc_handle.status = flag ? CH0StREALY : CH12StREALY;
    }
}

uint16_t ADC_Value_Handle(volatile u16 *data, u8 times)
{
    u8 k;
    u16 MAX = 0, MIN = 0;
    u16 adc_value_1;
    u16 buf[GET_ADC_len];
    u16 adc_sum = 0u;

    if (times > GET_ADC_len) {
        times = GET_ADC_len;
    }
    for (k = 0u; k < times; k++) {
        buf[k] = data[k];
    }

    MAX = buf[0];
    MIN = buf[0];
    for (k = 0u; k < times; k++) {
        if (buf[k] > MAX) {
            MAX = buf[k];
        }
        if (buf[k] < MIN) {
            MIN = buf[k];
        }
        adc_sum = (u16)(adc_sum + buf[k]);
    }
#if GET_ADC_len > 2
    adc_value_1 = (u16)((adc_sum - MAX - MIN) / (u16)(GET_ADC_len - 2u));
#else
    adc_value_1 = (u16)(adc_sum / 2u);
#endif
    return adc_value_1;
}

uint16_t adc_zhonzhi(volatile u16 *data, u8 times)
{
    u8 i, j, k;
    u16 temp;
    u16 valuehandle[GET_ADC_len];

    for (k = 0u; k < times; k++) {
        valuehandle[k] = data[k];
    }
    for (j = 1u; j < times; j++) {
        for (i = 0u; i < (u8)(times - j); i++) {
            if (valuehandle[i] > valuehandle[i + 1u]) {
                temp               = valuehandle[i];
                valuehandle[i]     = valuehandle[i + 1u];
                valuehandle[i + 1u] = temp;
            }
        }
    }
    return valuehandle[times / 2u];
}

void bdc_user_adc_irq_handler(void)
{
    if (ADC_GetITStatus(ADC_IT_EOS) == SET) {
        u8 idx = adc_handle.irq_flag;

        if (idx < GET_ADC_len) {
            adc0data[idx] = ADC_GetConversionValue(ADC_RESULT_0);
            adc1data[idx] = ADC_GetConversionValue(ADC_RESULT_1);
            adc2data[idx] = ADC_GetConversionValue(ADC_RESULT_2);
            adc_handle.irq_flag++;
            get_adc_value();
        }
        ADC_ClearITPendingBit(ADC_IT_EOS);
    }
}
