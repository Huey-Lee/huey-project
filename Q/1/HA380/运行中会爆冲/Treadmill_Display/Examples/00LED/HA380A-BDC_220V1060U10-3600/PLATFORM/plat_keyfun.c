#include "plat_keyfun.h"
#include "plat_rf.h"
#include "plat_beep.h"
#include "treadmills.h"

/* 调节按键手感参数 (10ms为基准) */
#define RF_SUSTAIN_TICKS     30   // 300ms 维持窗口
#define HOLD_START_TICKS    150   // 1000ms (1.0秒) 后才判定为长按，解决误触发
#define REPEAT_RATE_TICKS    30   // 300ms 连发速率 (调数值越小越快)

static uint16_t g_energy = 0;
static uint16_t g_duration = 0;
static uint16_t g_repeat_tmr = 0;
static KeyID_t  g_active_id = K_ID_NONE;
static _Bool    g_is_holding = 0;

static KeyID_t Map_RF(uint8_t c) {
    switch (c) {
        case 0x04: case 0x2E: return K_ID_UP;
        case 0x05: case 0x4C: return K_ID_DN;
        case 0x1A: case 0x5B: return K_ID_ONOFF;
        case 0x01: case 0x3D: return K_ID_MODE;
        default: return K_ID_NONE;
    }
}

/**
 * @brief 按键引擎核心
 */
void Keypad_Handler_10ms(void) {
    // 1. 原始信号解析
    if (g_rf_msg.vld) {
        __disable_irq();
        uint8_t r_key = g_rf_msg.key;
        g_rf_msg.vld = false;
        __enable_irq();

        KeyID_t new_id = Map_RF(r_key);
        if (new_id != K_ID_NONE) {
            // 只有当前无按键时，才允许产生一次 PRESS 事件
            if (g_active_id == K_ID_NONE) {
                g_active_id = new_id;
                g_duration = 0;
                g_is_holding = 0;
                g_repeat_tmr = 0;
                Keypad_On_Event(g_active_id, K_EVT_PRESS); // 触发第一次点击
            }
            // 只要信号在，就持续喂能，防止触发 RELEASE
            g_energy = RF_SUSTAIN_TICKS; 
        }
    }

    if (g_active_id == K_ID_NONE) return;

    // 2. 状态计时
    if (g_energy > 0) {
        g_energy--;
        g_duration++;

        // 判断长按 (需超过 1.0 秒)
        if (g_duration >= HOLD_START_TICKS) {
            g_is_holding = 1;
            if (++g_repeat_tmr >= REPEAT_RATE_TICKS) {
                g_repeat_tmr = 0;
                Keypad_On_Event(g_active_id, K_EVT_HOLD); // 触发长按连发
            }
        }
    } else {
        // 信号消失 300ms 以后，才判定彻底松开
        Keypad_On_Event(g_active_id, K_EVT_RELEASE);
        g_active_id = K_ID_NONE;
        g_duration = 0;
        g_is_holding = 0;
    }
}

void Keypad_On_Event(KeyID_t id, KeyEvt_t evt) {

    // 先传给业务层，由业务层决定是否有效
    bool consumed = Treadmill_On_Event((uint8_t)id, (uint8_t)evt);

    // 只有有效操作才响按键音
    if (evt == K_EVT_PRESS && consumed) {
        Beep_Play(BEEP_CLICK); 
    }
}

