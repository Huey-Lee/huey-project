#include "plat_beep.h"
#include "bsp_pwm.h"

static struct {
  uint16_t freq; 
	uint16_t on_ms; 
	uint16_t off_ms;
  uint16_t timer; 
	uint8_t count; 
	uint8_t vol;
  uint8_t  is_playing; 
	uint8_t mute_flag;
} g_beep_dev;

void Beep_Init(void) {
    BSP_PWM_Init();
    g_beep_dev.mute_flag = 0; // ??????0, ???1
}

void Beep_Play_Custom(uint16_t freq, uint8_t vol, uint16_t on, uint16_t off, uint8_t count) {
    if (g_beep_dev.mute_flag) return;
    g_beep_dev.freq = freq; g_beep_dev.vol = vol;
    g_beep_dev.on_ms = on; g_beep_dev.off_ms = off;
    g_beep_dev.count = count; g_beep_dev.timer = on;
    g_beep_dev.is_playing = 1;
    BSP_PWM_Set(freq, vol);
}

void Beep_Play(BeepMode_t mode) {
    switch (mode) {
        case BEEP_CLICK:    	Beep_Play_Custom(BEEP_FREQ_HZ, 50, 50, 0, 1);       break;
        case BEEP_LIMIT:   	 	Beep_Play_Custom(BEEP_FREQ_HZ, 50, 200, 0, 1);      break;
        case BEEP_ALARM:   	 	Beep_Play_Custom(BEEP_FREQ_HZ, 50, 100, 100, 10);   break;
        case BEEP_POWER_ON: 	Beep_Play_Custom(BEEP_FREQ_HZ, 50, 400, 0, 1);      break;
        case BEEP_PROG_SHIFT:	Beep_Play_Custom(BEEP_FREQ_HZ, 50, 400, 0, 1);			break;
    }
}

void Beep_Handler_1ms(void) {
    if (g_beep_dev.count == 0) return;
    if (g_beep_dev.timer > 0) {
        g_beep_dev.timer--;
    } else {
        if (g_beep_dev.is_playing) { // ?????????????
            BSP_PWM_Stop();
            g_beep_dev.is_playing = 0;
            g_beep_dev.timer = g_beep_dev.off_ms;
            if (g_beep_dev.off_ms == 0) g_beep_dev.count = 0; // ??????????
        } else { // ????????????
            if (--g_beep_dev.count > 0) {
                g_beep_dev.is_playing = 1;
                g_beep_dev.timer = g_beep_dev.on_ms;
                BSP_PWM_Set(g_beep_dev.freq, g_beep_dev.vol);
            }
        }
    }
}
