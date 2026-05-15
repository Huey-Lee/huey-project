/*
 * plat_beep.c  -  PWM 蜂鸣器，HA380A 同款机制 + SHNE 音效表
 *
 * Beep_Handler_1ms()：在 BTIM1 的 1 ms 中断里调用（见 bsp_timer.c），勿依赖主循环协作任务，否则 UART 等长临界路径
 * 会让节拍堆积、短音被「快进」截断。
 * 报警重复次数：treadmills.h 内 TM_ERROR_BEEP_COUNT。
 * 整机默认静音：treadmills.h 内 TM_BEEP_MUTE_DEFAULT 置 1。
 */
#include "plat_beep.h"
#include "treadmills.h"
#include "bsp_pwm.h"

#define BEEP_CLICK_ON_MS  48u
#define BEEP_PENDING_MAX  32u

static struct {
    uint16_t freq;
    uint16_t on_ms;
    uint16_t off_ms;
    uint16_t timer;
    uint8_t  count;
    uint8_t  vol;
    uint8_t  is_playing;
    uint8_t  mute_flag;
    /* 每次重新进入 ON 相后的第一个 1ms 不扣 timer，保证 on_ms 个完整毫秒鸣叫 */
    uint8_t  skip_first_timer_dec;
} g_beep;

static uint8_t g_pending_clicks;

static void Beep_AbortCurrent(void)
{
    BSP_PWM_Stop();
    g_beep.is_playing           = 0u;
    g_beep.count                = 0u;
    g_beep.timer                 = 0u;
    g_beep.skip_first_timer_dec = 0u;
}

static void Beep_ClearPending(void)
{
    g_pending_clicks = 0u;
}

void Beep_Init(void)
{
    BSP_PWM_Init();
    g_pending_clicks = 0u;
#if (TM_BEEP_MUTE_DEFAULT != 0u)
    g_beep.mute_flag = 1u;
#else
    g_beep.mute_flag = 0u;
#endif
    Beep_AbortCurrent();
}

void Beep_Stop(void)
{
    Beep_ClearPending();
    Beep_AbortCurrent();
}

uint8_t Beep_IsBusy(void)
{
    return (g_beep.count != 0u || g_beep.is_playing != 0u || g_pending_clicks != 0u)
               ? 1u
               : 0u;
}

void Beep_Play_Custom(uint16_t freq, uint8_t vol, uint16_t on, uint16_t off, uint8_t count)
{
    if (g_beep.mute_flag) return;
    g_beep.freq                 = freq;
    g_beep.vol                  = vol;
    g_beep.on_ms                = on;
    g_beep.off_ms               = off;
    g_beep.count                = count;
    g_beep.timer                  = on;
    g_beep.is_playing           = 1u;
    g_beep.skip_first_timer_dec = 1u;
    BSP_PWM_Set(freq, vol);
}

void Beep_Play(BeepMode_t mode)
{
    switch (mode) {
        case BEEP_CLICK:
            if (g_beep.mute_flag) return;
            if (g_beep.count == 0u && g_pending_clicks == 0u &&
                g_beep.is_playing == 0u) {
                Beep_Play_Custom(2000, 50, BEEP_CLICK_ON_MS, 0, 1);
            } else {
                if (g_pending_clicks < BEEP_PENDING_MAX)
                    g_pending_clicks++;
            }
            break;

        case BEEP_ALARM:
            if (g_beep.mute_flag) return;
            Beep_ClearPending();
            Beep_AbortCurrent();
            Beep_Play_Custom(2000, 50, 100, 100, TM_ERROR_BEEP_COUNT);
            break;

        case BEEP_POWER_ON:
            if (g_beep.mute_flag) return;
            Beep_AbortCurrent();
            Beep_Play_Custom(2000, 50, 300, 0, 1);
            break;

        case BEEP_LONG:
            if (g_beep.mute_flag) return;
            Beep_AbortCurrent();
            Beep_Play_Custom(2000, 50, 500, 0, 1);
            break;

        case BEEP_PAIR_OK:
            if (g_beep.mute_flag) return;
            Beep_AbortCurrent();
            Beep_Play_Custom(2000, 50, 80, 130, 2);
            break;

        default:
            break;
    }
}

void Beep_Handler_1ms(void)
{
    if (g_beep.count == 0u) {
        if (g_pending_clicks > 0u) {
            if (g_beep.mute_flag) {
                g_pending_clicks--;
                return;
            }
            g_pending_clicks--;
            Beep_Play_Custom(2000, 50, BEEP_CLICK_ON_MS, 0, 1);
            return;
        }
        return;
    }

    if (g_beep.skip_first_timer_dec) {
        g_beep.skip_first_timer_dec = 0u;
        return;
    }

    if (g_beep.timer > 0u) {
        g_beep.timer--;
    } else {
        if (g_beep.is_playing) {
            BSP_PWM_Stop();
            g_beep.is_playing = 0u;
            g_beep.timer       = g_beep.off_ms;
            if (g_beep.off_ms == 0u) g_beep.count = 0u;
        } else {
            if (--g_beep.count > 0u) {
                g_beep.is_playing           = 1u;
                g_beep.timer                = g_beep.on_ms;
                g_beep.skip_first_timer_dec = 1u;
                if (!g_beep.mute_flag) {
                    BSP_PWM_Set(g_beep.freq, g_beep.vol);
                }
            }
        }
    }
}
