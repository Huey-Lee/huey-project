/*
 * plat_beep.c  -  Buzzer driver (PWM-based, 1 ms tick)
 *
 * Call Beep_Handler_1ms() from the 1 ms system timer interrupt.
 * BEEP_ALARM count is controlled by TM_ERROR_BEEP_COUNT in treadmills.h.
 */
#include "plat_beep.h"
#include "treadmills.h"
#include "bsp_pwm.h"

/* Shorter than old 60 ms: crisper key feel; RF double-frames less likely to "cut" mid-beep. */
#define BEEP_CLICK_ON_MS 45u

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

void Beep_Init(void) {
    BSP_PWM_Init();
    g_beep.mute_flag = 0;
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
    if (mode == BEEP_CLICK) {
        /* Re-entrance (e.g. RF glitches) restarts the driver and can sound "half" a beep. */
        if (g_beep.is_playing && g_beep.count == 1u && g_beep.off_ms == 0u
            && g_beep.on_ms == BEEP_CLICK_ON_MS) {
            return;
        }
    }
    switch (mode) {
        case BEEP_CLICK:    Beep_Play_Custom(2000, 50, BEEP_CLICK_ON_MS, 0, 1);  break;
        case BEEP_LIMIT:    Beep_Play_Custom(2000, 50,  60,  60, 1);  break;
        case BEEP_ALARM:    Beep_Play_Custom(2000, 50, 100, 100, TM_ERROR_BEEP_COUNT); break;
        case BEEP_POWER_ON: Beep_Play_Custom(2000, 50, 300,   0, 1);  break;
        default: break;
    }
}

void Beep_Handler_1ms(void)
{
    if (g_beep.count == 0) return;

    if (g_beep.timer > 0) {
        g_beep.timer--;
    } else {
        if (g_beep.is_playing) {
            BSP_PWM_Stop();
            g_beep.is_playing = 0;
            g_beep.timer      = g_beep.off_ms;
            if (g_beep.off_ms == 0) g_beep.count = 0;
        } else {
            if (--g_beep.count > 0) {
                g_beep.is_playing = 1;
                g_beep.timer      = g_beep.on_ms;
                BSP_PWM_Set(g_beep.freq, g_beep.vol);
            }
        }
    }
}
