#include "plat_touchkey.h"
#include "cw32l010_adc.h"
#include "cw32l010_gpio.h"
#include "cw32l010_sysctrl.h"

/* 12bit 满量程（无键≈VDD）；分档边界为相邻两档电压中点 ×32 与 4095 相乘的整数比较 */
#define TK_ADC_FULL     4095u

/* 触摸 PAD 编号 0..8；0xFF 无有效键 */
typedef enum {
    TK_PAD_NONE = 0xFFu,
    TK_PAD0 = 0,
    TK_PAD1,
    TK_PAD2,
    TK_PAD3,
    TK_PAD4,
    TK_PAD5,
    TK_PAD6,
    TK_PAD7,
    TK_PAD8
} TkPad_t;

/* 单位均为「10ms 节拍」：与 Keypad_Handler_10ms 调用周期一致 */
static Touchkey_Params_t s_param = {
    .debounce_ticks    = 4,   /* 连续 N 次采样同档才确认，抑抖；越大越稳、越钝 */
    .sustain_ticks     = 4,   /* 每周期若仍判定为按下则把 g_energy 顶到此值，防松手毛刺 */
    .hold_start_ticks  = 150, /* 按下累计多久后算长按并开始连发（约 1.5s） */
    .repeat_ticks      = 12,  /* 长按后 K_EVT_HOLD 间隔；越小连发越快 */
};

static uint8_t  s_db_count;
static TkPad_t  s_db_pad;
static TkPad_t  s_stable_pad;

static KeyID_t prv_pad_to_key(TkPad_t pad)
{
    switch (pad) {
        case TK_PAD6: return K_ID_DN;     /* TK1 + */
        case TK_PAD4: return K_ID_MODE;   /* TK2 M */
        case TK_PAD5: return K_ID_ONOFF;  /* TK3 ON/OFF */
        case TK_PAD0: return K_ID_UP;     /* TK4 -

			*/
        default: return K_ID_NONE;
    }
}

static TkPad_t prv_classify_pad(uint16_t adc)
{
    /* sc = adc*32 与 (4095*mid_ratio*32) 等价于 adc 与 4095*mid_ratio 比较 */
    uint32_t sc = (uint32_t)adc * 32u;
    if (sc >= 29ul * TK_ADC_FULL) return TK_PAD_NONE;
    if (sc >= 25ul * TK_ADC_FULL) return TK_PAD8;
    if (sc >= 23ul * TK_ADC_FULL) return TK_PAD7;
    if (sc >= 21ul * TK_ADC_FULL) return TK_PAD6;
    if (sc >= 19ul * TK_ADC_FULL) return TK_PAD5;
    if (sc >= 17ul * TK_ADC_FULL) return TK_PAD4;
    if (sc >= 15ul * TK_ADC_FULL) return TK_PAD3;
    if (sc >= 13ul * TK_ADC_FULL) return TK_PAD2;
    if (sc >= 11ul * TK_ADC_FULL) return TK_PAD1;
    if (sc >= 9ul * TK_ADC_FULL) return TK_PAD0;
    return TK_PAD_NONE;
}

static uint16_t prv_adc_read_once(void)
{
    ADC_ClearITPendingBit(ADC_IT_EOC);
    ADC_SoftwareStartConvCmd(ENABLE);
    for (volatile uint32_t n = 0u; n < 8000u; n++) {
        if (ADC_GetITStatus(ADC_IT_EOC) == SET)
            break;
    }
    {
        uint16_t v = ADC_GetConversionValue(ADC_RESULT_0);
        ADC_ClearITPendingBit(ADC_IT_EOC);
        ADC_SoftwareStartConvCmd(DISABLE);
        return v;
    }
}

static uint16_t prv_adc_average(void)
{
    uint32_t sum = 0;
    uint8_t   k;
    for (k = 0; k < 6u; k++)
        sum += prv_adc_read_once();
    return (uint16_t)(sum / 6u);
}

void Touchkey_Init(void)
{
    GPIO_InitTypeDef g = {0};
    ADC_InitTypeDef  a = {0};

    __SYSCTRL_GPIOA_CLK_ENABLE();

    g.Pins = GPIO_PIN_5;
    g.Mode = GPIO_MODE_ANALOG;
    GPIO_Init(CW_GPIOA, &g);
    PA05_ANALOG_ENABLE();

    a.ADC_ClkDiv       = ADC_Clk_Div8;
    a.ADC_ConvertMode  = ADC_ConvertMode_Once;
    a.ADC_SQREns       = ADC_SqrEns0to1;
    a.ADC_IN0.ADC_InputChannel = ADC_InputCH5;
    a.ADC_IN0.ADC_SampTime     = ADC_SampTime70Clk;
    a.ADC_IN1.ADC_InputChannel = ADC_InputCH5;
    a.ADC_IN1.ADC_SampTime     = ADC_SampTime6Clk;
    a.ADC_IN2.ADC_InputChannel = ADC_InputCH5;
    a.ADC_IN2.ADC_SampTime     = ADC_SampTime6Clk;
    a.ADC_IN3.ADC_InputChannel = ADC_InputCH5;
    a.ADC_IN3.ADC_SampTime     = ADC_SampTime6Clk;
    a.ADC_IN4.ADC_InputChannel = ADC_InputCH5;
    a.ADC_IN4.ADC_SampTime     = ADC_SampTime6Clk;
    a.ADC_IN5.ADC_InputChannel = ADC_InputCH5;
    a.ADC_IN5.ADC_SampTime     = ADC_SampTime6Clk;
    a.ADC_IN6.ADC_InputChannel = ADC_InputCH5;
    a.ADC_IN6.ADC_SampTime     = ADC_SampTime6Clk;
    a.ADC_IN7.ADC_InputChannel = ADC_InputCH5;
    a.ADC_IN7.ADC_SampTime     = ADC_SampTime6Clk;

    ADC_Init(&a);
    (void)ADC_Enable();

    s_db_count    = 0;
    s_db_pad      = TK_PAD_NONE;
    s_stable_pad  = TK_PAD_NONE;
}

void Touchkey_SetParams(const Touchkey_Params_t *p)
{
    if (p == NULL) return;
    s_param = *p;
}

void Touchkey_GetParams(Touchkey_Params_t *p)
{
    if (p != NULL) *p = s_param;
}

KeyID_t Touchkey_Handler_10ms(void)
{
    uint16_t raw = prv_adc_average();
    TkPad_t  now = prv_classify_pad(raw);

    if (now == s_db_pad) {
        if (s_db_count < 255u) s_db_count++;
    } else {
        s_db_pad = now;
        s_db_count = 1;
    }

    if (s_db_count >= s_param.debounce_ticks) {
        if (s_stable_pad != s_db_pad)
            s_stable_pad = s_db_pad;
    }

    (void)raw;
    return prv_pad_to_key(s_stable_pad);
}

/* 供 plat_keyfun 读取长按/连发参数 */
uint16_t Touchkey_GetHoldStartTicks(void) { return s_param.hold_start_ticks; }
uint8_t  Touchkey_GetRepeatTicks(void) { return s_param.repeat_ticks; }
uint8_t  Touchkey_GetSustainTicks(void) { return s_param.sustain_ticks; }
