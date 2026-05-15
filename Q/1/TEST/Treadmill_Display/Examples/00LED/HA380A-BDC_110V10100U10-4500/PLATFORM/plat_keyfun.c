#include "plat_keyfun.h"
#include "plat_rf.h"
#include "plat_beep.h"
#include "treadmills.h"

/* 调节按键手感参数 (10ms 为基准) */
/* 无新包后计满才 RELEASE；原 30≈300ms 易吃连按 */
#define RF_SUSTAIN_TICKS     16u  /* 无新包约 160ms 后算松开；要更快可 10~12u */
#define HOLD_START_TICKS    150u  /* 1.0s 后进入长按/连发 */
#define REPEAT_RATE_TICKS     30u  /* 连发周期 */

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
            if (g_active_id == K_ID_NONE) {
                g_active_id = new_id;
                g_duration  = 0;
                g_is_holding  = 0;
                g_repeat_tmr  = 0;
                Keypad_On_Event(g_active_id, K_EVT_PRESS);
            } else if (new_id != g_active_id) {
                /* 快按换键：原逻辑不 RELEASE 导致新键 PRESS 永远进不来 */
                Keypad_On_Event(g_active_id, K_EVT_RELEASE);
                g_active_id  = new_id;
                g_duration   = 0;
                g_is_holding     = 0;
                g_repeat_tmr     = 0;
                Keypad_On_Event(g_active_id, K_EVT_PRESS);
            }
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
        _Bool was_holding = g_is_holding;
        Keypad_On_Event(g_active_id, K_EVT_RELEASE);
        if (was_holding) {
            /* 仅「曾进入过长按」后真正松开(非换键路径)；换键的 RELEASE 走上面 vld 分支不经过此处 */
            Beep_Play(BEEP_CLICK);
        }
        g_active_id  = K_ID_NONE;
        g_duration   = 0;
        g_is_holding = 0;
    }
}

void Keypad_On_Event(KeyID_t id, KeyEvt_t evt) {

    // 先传给业务层，由业务层决定是否有效
    bool consumed = Treadmill_On_Event((uint8_t)id, (uint8_t)evt);

    if (evt == K_EVT_PRESS && consumed) {
        /* 进减速时 Enter_Stopping 已先响，此处去重 */
        if (!Treadmill_ConsumeKeyBeepSuppress()) {
            Beep_Play(BEEP_CLICK);
        }
    }
}

