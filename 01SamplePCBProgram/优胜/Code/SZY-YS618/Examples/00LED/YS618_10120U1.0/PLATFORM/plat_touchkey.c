#include "plat_touchkey.h"
#include "cw32l010_adc.h"
#include "cw32l010_gpio.h"
#include "cw32l010_sysctrl.h"

#define TK_ADC_FULL     4095u

#define TK_LINE_A       0u
#define TK_LINE_B       1u

/** PA05 = CH5；PA02 = CH2。若 LINE_B 实际接 PA04 改为 ADC_InputCH4 */
#define TK_ADC_CH_LINE_A    ADC_InputCH5
#define TK_ADC_CH_LINE_B    ADC_InputCH2

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

static Touchkey_Params_t s_param = {
    .debounce_ticks    = 4,
    .sustain_ticks     = 4,
    .hold_start_ticks  = 150,
    .repeat_ticks      = 12,
};

static uint8_t   s_db_count;
static KeyID_t   s_db_key;
static KeyID_t   s_stable_key;

static const KeyID_t s_keys_line_a[9] = {
    K_ID_MODE, K_ID_NONE, K_ID_DN, K_ID_NONE, K_ID_NONE, K_ID_NONE, K_ID_9, K_ID_12, K_ID_NONE
};
static const KeyID_t s_keys_line_b[9] = {
    K_ID_P, K_ID_UP, K_ID_NONE, K_ID_NONE, K_ID_ONOFF, K_ID_6, K_ID_3, K_ID_NONE, K_ID_NONE
};

static int prv_slot_from_pad(TkPad_t p)
{
    switch (p) {
        case TK_PAD0: return 0;
        case TK_PAD1: return 1;
        case TK_PAD2: return 2;
        case TK_PAD3: return 3;
        case TK_PAD4: return 4;
        case TK_PAD5: return 5;
				case TK_PAD6: return 6;
				case TK_PAD7: return 7;
				case TK_PAD8: return 8;
        default: return -1;
    }
}

static KeyID_t prv_map_line(uint8_t line, TkPad_t p)
{
    int slot = prv_slot_from_pad(p);
    if (slot < 0)
        return K_ID_NONE;
    if (line == TK_LINE_A)
        return s_keys_line_a[slot];
    return s_keys_line_b[slot];
}

/** 与 SK 185A 单路相同分档；两路电阻网络一致时直接用同一阈值表 */
static TkPad_t prv_classify_pad(uint16_t adc)
{
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

static void prv_adc_select_ch(uint32_t ch)
{
    CW_ADC->SQRCFR_f.SQRCH0 = ch;
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

static uint16_t prv_adc_average_ch(uint32_t ch)
{
    uint32_t sum = 0;
    uint8_t  k;
    prv_adc_select_ch(ch);
    for (k = 0; k < 6u; k++)
        sum += prv_adc_read_once();
    return (uint16_t)(sum / 6u);
}

void Touchkey_Init(void)
{
    GPIO_InitTypeDef g = {0};
    ADC_InitTypeDef  a = {0};

    __SYSCTRL_GPIOA_CLK_ENABLE();

    g.Pins = GPIO_PIN_2 | GPIO_PIN_5;
    g.Mode = GPIO_MODE_ANALOG;
    GPIO_Init(CW_GPIOA, &g);
    PA02_ANALOG_ENABLE();
    PA05_ANALOG_ENABLE();

    a.ADC_ClkDiv       = ADC_Clk_Div8;
    a.ADC_ConvertMode  = ADC_ConvertMode_Once;
    a.ADC_SQREns       = ADC_SqrEns0to1;
    a.ADC_IN0.ADC_InputChannel = TK_ADC_CH_LINE_A;
    a.ADC_IN0.ADC_SampTime     = ADC_SampTime70Clk;
    a.ADC_IN1.ADC_InputChannel = TK_ADC_CH_LINE_A;
    a.ADC_IN1.ADC_SampTime     = ADC_SampTime6Clk;
    a.ADC_IN2.ADC_InputChannel = TK_ADC_CH_LINE_A;
    a.ADC_IN2.ADC_SampTime     = ADC_SampTime6Clk;
    a.ADC_IN3.ADC_InputChannel = TK_ADC_CH_LINE_A;
    a.ADC_IN3.ADC_SampTime     = ADC_SampTime6Clk;
    a.ADC_IN4.ADC_InputChannel = TK_ADC_CH_LINE_A;
    a.ADC_IN4.ADC_SampTime     = ADC_SampTime6Clk;
    a.ADC_IN5.ADC_InputChannel = TK_ADC_CH_LINE_A;
    a.ADC_IN5.ADC_SampTime     = ADC_SampTime6Clk;
    a.ADC_IN6.ADC_InputChannel = TK_ADC_CH_LINE_A;
    a.ADC_IN6.ADC_SampTime     = ADC_SampTime6Clk;
    a.ADC_IN7.ADC_InputChannel = TK_ADC_CH_LINE_A;
    a.ADC_IN7.ADC_SampTime     = ADC_SampTime6Clk;

    ADC_Init(&a);
    (void)ADC_Enable();

    s_db_count    = 0;
    s_db_key      = K_ID_NONE;
    s_stable_key  = K_ID_NONE;
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
    uint16_t ra = prv_adc_average_ch(TK_ADC_CH_LINE_A);
    uint16_t rb = prv_adc_average_ch(TK_ADC_CH_LINE_B);
    TkPad_t  pa = prv_classify_pad(ra);
    TkPad_t  pb = prv_classify_pad(rb);

    KeyID_t id_a = prv_map_line(TK_LINE_A, pa);
    KeyID_t id_b = prv_map_line(TK_LINE_B, pb);

    KeyID_t cand = K_ID_NONE;
    if (id_a != K_ID_NONE)
        cand = id_a;
    else if (id_b != K_ID_NONE)
        cand = id_b;

    if (cand == s_db_key) {
        if (s_db_count < 255u) s_db_count++;
    } else {
        s_db_key = cand;
        s_db_count = 1;
    }

    if (s_db_count >= s_param.debounce_ticks)
        s_stable_key = s_db_key;

    (void)rb;
    (void)ra;
    return s_stable_key;
}

uint16_t Touchkey_GetHoldStartTicks(void) { return s_param.hold_start_ticks; }
uint8_t  Touchkey_GetRepeatTicks(void) { return s_param.repeat_ticks; }
uint8_t  Touchkey_GetSustainTicks(void) { return s_param.sustain_ticks; }
