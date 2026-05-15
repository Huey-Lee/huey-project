/*
 * plat_beep.c  -  Buzzer driver (PWM-based, 1 ms tick)
 *
 * Call Beep_Handler_1ms() from the 1 ms system timer interrupt.
 * BEEP_ALARM count: TM_ERROR_BEEP_COUNT. Mute default: TM_BEEP_MUTE_DEFAULT in treadmills.h.
 */
#include "plat_beep.h"
#include "treadmills.h"
#include "bsp_pwm.h"

/* 较长短按更自然；快连时靠队列续播、不硬截断前一声 (Beep_Play 不再覆盖正在响的 CLICK) */
#define BEEP_CLICK_ON_MS  48u
#define BEEP_PENDING_MAX  6u  /* 排队深度，打满后多余按键音丢弃 */

static struct {
    uint16_t freq;
    uint16_t on_ms;
    uint16_t off_ms;
    uint16_t timer;
    uint8_t  count;
    uint8_t  vol;
    uint8_t  is_playing;
    uint8_t  mute_flag;
} g_beep;

/* 有蜂鸣/间歇仍在进行的 sequence 中再点按键，排队保证每声播满再播下一声 */
static uint8_t g_pending_clicks;

void Beep_Init(void) {
    BSP_PWM_Init();
#if (TM_BEEP_MUTE_DEFAULT != 0u)
    g_beep.mute_flag = 1u;
#else
    g_beep.mute_flag = 0u;
#endif
    g_pending_clicks  = 0u;
}

void Beep_Play_Custom(uint16_t freq, uint8_t vol, uint16_t on, uint16_t off, uint8_t count)
{
    if (g_beep.mute_flag) return;
    g_beep.freq       = freq;
    g_beep.vol        = vol;
    g_beep.on_ms      = on;
    g_beep.off_ms     = off;
    g_beep.count      = count;
    g_beep.timer      = on;
    g_beep.is_playing = 1;
    BSP_PWM_Set(freq, vol);
}

void Beep_Play(BeepMode_t mode)
{
    switch (mode) {
        case BEEP_CLICK:
            if (g_beep.mute_flag) return;
            if (g_beep.count == 0u && g_pending_clicks == 0u) {
                Beep_Play_Custom(2000, 50, BEEP_CLICK_ON_MS, 0, 1);
            } else {
                if (g_pending_clicks < BEEP_PENDING_MAX) g_pending_clicks++;
            }
            break;
        case BEEP_ALARM:    Beep_Play_Custom(2000, 50, 100, 100, TM_ERROR_BEEP_COUNT); break;
        case BEEP_POWER_ON: Beep_Play_Custom(2000, 50, 300,   0, 1);  break;
        case BEEP_PAIR_OK:  Beep_Play_Custom(2000, 50, 80, 130, 2);  break;
        default: break;
    }
}

void Beep_Handler_1ms(void)
{
    if (g_beep.count == 0u) {
        if (g_pending_clicks > 0u) {
            if (g_beep.mute_flag) return; /* 勿先 pending-- 后 Custom 失败 */
            g_pending_clicks--;
            Beep_Play_Custom(2000, 50, BEEP_CLICK_ON_MS, 0, 1);
        } else {
            return;
        }
    }

    if (g_beep.timer > 0) {
        g_beep.timer--;
    } else {
        if (g_beep.is_playing) {
            BSP_PWM_Stop();
            g_beep.is_playing = 0;
            g_beep.timer      = g_beep.off_ms;
            if (g_beep.off_ms == 0) g_beep.count = 0u;
        } else {
            if (--g_beep.count > 0) {
                g_beep.is_playing = 1;
                g_beep.timer      = g_beep.on_ms;
                BSP_PWM_Set(g_beep.freq, g_beep.vol);
            }
        }
    }
}
