#include "plat_rf.h"

#define RF_PIN_LEVEL()      GPIO_ReadPin(CW_GPIOB, GPIO_PIN_0) // 接收引脚 PB00

/* 算法阈值 (基于 60us tick) */
#define TICKS_SYNC_MIN      60    // 同步头最短 3.6ms
#define TICKS_MAX_LIMIT     450   // 超时 27ms
#define TICKS_GLITCH        2     // 毛刺过滤

RF_Msg_t g_rf_msg = {0};

void RF433_Handler_60us(void) {
    static uint8_t  state = 0; 	// 0:Idle, 1:Receiving
    static uint32_t h_ticks = 0, l_ticks = 0, rx_reg = 0;
    static uint8_t  bit_cnt = 0, last_pin = 1;

    uint8_t cur_pin = RF_PIN_LEVEL();
    if (cur_pin) h_ticks++; else l_ticks++;

    if (cur_pin != last_pin) {
        if (cur_pin == 1) { 	// 下降沿结束 (Low结束)
            if (state == 0) {
                if (l_ticks > TICKS_SYNC_MIN && l_ticks < TICKS_MAX_LIMIT) {
                    state = 1; bit_cnt = 0; rx_reg = 0;
                }
            } else {
                rx_reg <<= 1;
                if (h_ticks > l_ticks) rx_reg |= 1; // 自适应比例算法核心
                if (++bit_cnt == 24) {
                    g_rf_msg.addr = (rx_reg >> 8) & 0xFFFF;
                    g_rf_msg.key  = rx_reg & 0xFF;
                    g_rf_msg.type = RF_TYPE_24BIT;
					g_rf_msg.vld  = true;			// 24位协议收到后立即标记，防止等待超时导致延迟
                } else if (bit_cnt == 32) {
                    uint8_t sum = (uint8_t)((rx_reg>>24) + (rx_reg>>16) + (rx_reg>>8));
                    if (sum == (uint8_t)(rx_reg & 0xFF)) {
                        g_rf_msg.addr = (rx_reg >> 16) & 0xFFFF;
                        g_rf_msg.key  = (rx_reg >> 8) & 0xFF;
                        g_rf_msg.type = RF_TYPE_32BIT;
                        g_rf_msg.vld = true;
                    }
                    state = 0;
                }
            }
            h_ticks = 0;
        } else { // 上升沿结束 (High结束) 
            if (h_ticks > TICKS_MAX_LIMIT) state = 0;
            l_ticks = 0;
        }
        last_pin = cur_pin;
    }

    // 超时重置
    if (l_ticks > TICKS_MAX_LIMIT || h_ticks > TICKS_MAX_LIMIT) {
        state = 0; l_ticks = 0; h_ticks = 0;
    }
}



