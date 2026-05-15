#include "plat_rf.h"

#define RF_PIN_LEVEL()      GPIO_ReadPin(CW_GPIOB, GPIO_PIN_0) /* 接收 PB00 */

/* 算法阈值（基于 60us tick） */
#define TICKS_SYNC_MIN      60
#define TICKS_MAX_LIMIT     450
#define TICKS_GLITCH        2

RF_Msg_t g_rf_msg = {0};

/* 24 位协议：16 位 ID + 8 位键值（旧的 32 位带校验协议已移除） */
void RF433_Handler_60us(void) {
    static uint8_t  state = 0;
    static uint32_t h_ticks = 0, l_ticks = 0, rx_reg = 0;
    static uint8_t  bit_cnt = 0, last_pin = 1;

    uint8_t cur_pin = RF_PIN_LEVEL();
    if (cur_pin) h_ticks++; else l_ticks++;

    if (cur_pin != last_pin) {
        if (cur_pin == 1) {
            if (state == 0) {
                if (l_ticks > TICKS_SYNC_MIN && l_ticks < TICKS_MAX_LIMIT) {
                    state   = 1;
                    bit_cnt = 0;
                    rx_reg  = 0;
                }
            } else {
                rx_reg <<= 1;
                if (h_ticks > l_ticks) rx_reg |= 1;
                if (++bit_cnt == 24) {
                    g_rf_msg.addr = (rx_reg >> 8) & 0xFFFF;
                    g_rf_msg.key  = rx_reg & 0xFF;
                    g_rf_msg.vld  = true;
                    state = 0;
                }
            }
            h_ticks = 0;
        } else {
            if (h_ticks > TICKS_MAX_LIMIT) state = 0;
            l_ticks = 0;
        }
        last_pin = cur_pin;
    }

    if (l_ticks > TICKS_MAX_LIMIT || h_ticks > TICKS_MAX_LIMIT) {
        state = 0;
        l_ticks = 0;
        h_ticks = 0;
    }
}
