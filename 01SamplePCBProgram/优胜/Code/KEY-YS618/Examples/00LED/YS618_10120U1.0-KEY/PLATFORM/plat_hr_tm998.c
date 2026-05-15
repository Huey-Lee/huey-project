/**
 * TM998 PB04：算法对齐 P417A_UP_KEY_V2 heart_driver（间隔 ms 400~1000、累加、低通、斜坡），
 * 硬件用 GPIO 边沿 + BTIM1 做 us 时间戳；协议仍为上层 Treadmill_Manager_100ms 周期 Keylink 0x20。
 */
#include "plat_hr_tm998.h"

#if PLAT_USE_HR_TM998

#include "treadmills.h"
#include "cw32l010_gpio.h"
#include "cw32l010_sysctrl.h"
#include "cw32l010_btim.h"
#if PLAT_HR_REQUIRE_PANEL_KEY
#include "plat_keylink.h"
#endif

#ifndef PLAT_HR_GPIO_PIN
#define PLAT_HR_GPIO_PIN GPIO_PIN_4
#endif
#ifndef PLAT_HR_BTIM
#define PLAT_HR_BTIM CW_BTIM1
#endif

/* 与 P417 heart_driver.h 一致 */
#define HEART_SUM_CNT        2u
#define HEART_FILTER_CNT     2u
#define HEART_CAP_MIN        400u
#define HEART_CAP_MAX        1000u
#define HEART_MIN            (60000u / HEART_CAP_MAX)
#define HEART_MAX            (60000u / HEART_CAP_MIN)
#define ERROR_CNT_MAX        30u
#define MAX_HEART_RATE_STEP  3u
#define DEFAULT_HEART_RATE   75u
#define LP_NUM               10u
#define LP_A                 4u /* 0.4 */
#define LP_B                 6u /* 0.6 */

static volatile uint32_t s_ms;
static volatile uint32_t s_last_pulse_ms;

static volatile uint32_t s_last_edge_us;
static volatile uint8_t  s_have_edge;

static uint8_t           s_heart_hold;
static uint8_t           s_capture_count;
static uint16_t          s_cal_cnt;
static uint32_t          s_time_sum;
static uint8_t           s_error_cnt;
static uint16_t          s_last_time_vary;

static uint8_t           s_target_rate;
static uint8_t           s_last_rate_value;

static void prv_heart_chain_reset(void)
{
    s_have_edge      = 0u;
    s_capture_count  = 0u;
    s_cal_cnt        = 0u;
    s_time_sum       = 0u;
    s_error_cnt      = 0u;
    s_last_time_vary = 0u;
    s_heart_hold     = 0u;
    s_target_rate    = DEFAULT_HEART_RATE;
    s_last_rate_value = DEFAULT_HEART_RATE;
}

static void prv_heart_handle_ms(uint16_t captured_ms)
{
    uint8_t is_valid = (captured_ms >= HEART_CAP_MIN && captured_ms <= HEART_CAP_MAX) ? 1u : 0u;

    if (s_cal_cnt > 0u) {
        if (is_valid) {
            if (s_capture_count > HEART_FILTER_CNT)
                s_time_sum += captured_ms;
        } else {
            if (s_capture_count > HEART_FILTER_CNT && s_cal_cnt > 0u)
                s_cal_cnt--;
            if (s_error_cnt < ERROR_CNT_MAX)
                s_error_cnt++;
            else
                s_error_cnt = ERROR_CNT_MAX;
            if (s_error_cnt >= ERROR_CNT_MAX)
                s_heart_hold = 0u;
        }
    }

    if (s_capture_count > HEART_FILTER_CNT)
        s_cal_cnt++;
    s_capture_count++;

    uint8_t should_calculate =
        (s_cal_cnt > HEART_SUM_CNT) && (s_capture_count > HEART_FILTER_CNT);
    if (!should_calculate)
        return;

    s_error_cnt = 0u;
    s_heart_hold = 1u;

    if (s_last_rate_value == 0u)
        s_last_rate_value = s_target_rate = DEFAULT_HEART_RATE;

    if (s_cal_cnt > 1u) {
        uint16_t avg = (uint16_t)(s_time_sum / (uint32_t)(s_cal_cnt - 1u));
        uint16_t tv;
        if (s_last_time_vary == 0u) {
            tv = avg;
        } else {
            tv = (uint16_t)((LP_A * avg + LP_B * s_last_time_vary + (LP_NUM / 2u)) / LP_NUM);
        }
        s_last_time_vary = tv;

        if (tv != 0u) {
            uint16_t raw_rate = (uint16_t)(60000u / (uint32_t)tv);
            if (raw_rate >= HEART_MIN && raw_rate <= HEART_MAX)
                s_target_rate = (uint8_t)raw_rate;
            else
                s_target_rate = s_last_rate_value;
        }
    }

    s_cal_cnt  = 0u;
    s_time_sum = 0u;
}

/** 与 P417 HeartRate_Cal 相同，由 10ms 任务按 1s 节拍调用 */
static void prv_heart_rate_cal_1s(void)
{
    if (!s_heart_hold)
        return;

    if (s_last_rate_value < s_target_rate) {
        uint8_t d = (uint8_t)(s_target_rate - s_last_rate_value);
        if (d > MAX_HEART_RATE_STEP)
            s_last_rate_value += MAX_HEART_RATE_STEP;
        else
            s_last_rate_value = s_target_rate;
    } else if (s_last_rate_value > s_target_rate) {
        uint8_t d = (uint8_t)(s_last_rate_value - s_target_rate);
        if (d > MAX_HEART_RATE_STEP)
            s_last_rate_value -= MAX_HEART_RATE_STEP;
        else
            s_last_rate_value = s_target_rate;
    }
}

void HrTm998_MsTick_1ms(void)
{
    s_ms++;
}

void HrTm998_GpioIrqHandler(void)
{
    if ((CW_GPIOB->ISR & PLAT_HR_GPIO_PIN) == 0u)
        return;
    GPIOB_INTFLAG_CLR(PLAT_HR_GPIO_PIN);

    uint32_t t_ms = s_ms;
    uint16_t cnt  = BTIM_GetCounter(PLAT_HR_BTIM);
    uint32_t t_us = t_ms * 1000u + (uint32_t)cnt;

#if PLAT_HR_PULSE_ACTIVE_HIGH
    if (GPIO_ReadPin(CW_GPIOB, PLAT_HR_GPIO_PIN) != GPIO_Pin_SET)
        return;
#else
    if (GPIO_ReadPin(CW_GPIOB, PLAT_HR_GPIO_PIN) != GPIO_Pin_RESET)
        return;
#endif

    s_last_pulse_ms = t_ms;

    if (!s_have_edge) {
        s_last_edge_us = t_us;
        s_have_edge    = 1u;
        return;
    }

    uint32_t    du  = t_us - s_last_edge_us;
    s_last_edge_us  = t_us;
    uint16_t    iv  = (uint16_t)(du / 1000u);
    if (iv == 0u && du >= 500u)
        iv = 1u;

    prv_heart_handle_ms(iv);
}

void HrTm998_Init(void)
{
    SYSCTRL_AHBPeriphClk_Enable(SYSCTRL_AHB_PERIPH_GPIOB, ENABLE);
    PB04_DIGTAL_ENABLE();

    {
        GPIO_InitTypeDef g = {0};
        g.Pins = PLAT_HR_GPIO_PIN;
#if PLAT_HR_PULSE_ACTIVE_HIGH
        g.Mode = GPIO_MODE_INPUT;
        g.IT   = GPIO_IT_RISING;
#else
        g.Mode = GPIO_MODE_INPUT_PULLUP;
        g.IT   = GPIO_IT_FALLING;
#endif
        GPIO_Init(CW_GPIOB, &g);
    }

    NVIC_SetPriority(GPIOB_IRQn, 2);
    NVIC_EnableIRQ(GPIOB_IRQn);

    s_ms            = 0u;
    s_last_pulse_ms = 0u;
    prv_heart_chain_reset();
    g_TM.hr_bpm     = 0u;
}

void HrTm998_OnKeypadScan_10ms(void)
{
    static uint8_t s_sec_tick;
    uint32_t       now = s_ms;

    if (s_last_pulse_ms != 0u && (now - s_last_pulse_ms) > PLAT_HR_OFFHAND_MS) {
        s_last_pulse_ms = 0u;
        prv_heart_chain_reset();
        g_TM.hr_bpm = 0u;
        goto hr_out;
    }

#if PLAT_HR_REQUIRE_PANEL_KEY
    if (!Keylink_IsPanelKeyDown()) {
        g_TM.hr_bpm = 0u;
        goto hr_out;
    }
#endif

    if (++s_sec_tick >= 100u) {
        s_sec_tick = 0u;
        prv_heart_rate_cal_1s();
    }

    if (!s_heart_hold) {
        g_TM.hr_bpm = 0u;
        goto hr_out;
    }

    g_TM.hr_bpm = s_last_rate_value;

hr_out:
    ((void)0);
}

#else /* !PLAT_USE_HR_TM998 */

void HrTm998_MsTick_1ms(void) { }
void HrTm998_Init(void) { }
void HrTm998_OnKeypadScan_10ms(void) { }
void HrTm998_GpioIrqHandler(void) { }

#endif
