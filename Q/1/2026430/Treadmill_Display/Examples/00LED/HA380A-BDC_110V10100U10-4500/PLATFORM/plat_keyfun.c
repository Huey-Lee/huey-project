#include "plat_keyfun.h"
#include "plat_rf.h"
#include "plat_beep.h"
#include "treadmills.h"

/* 调节按键手感参数 (10ms 为基准) */
#define RF_SUSTAIN_TICKS     20u  /* 无新包约 200ms 后算松开
                                   * 同一按键发两个RF码（如0x04/0x2E），码间间隔可达~150ms；
                                   * SUSTAIN必须大于该间隔，否则两码各自触发PRESS → 跳两下 */
#define HOLD_START_TICKS    100u  /* 1.0s 后进入长按/连发 */
#define REPEAT_RATE_TICKS     30u  /* 连发周期 */
/* 防误触：同一键码连续收到 N 包才确认，滤单次噪声包
 * THRESH=1：即时响应（原始行为），灵敏度优先；如误触严重可改 2 */
#define RF_DEBOUNCE_THRESH    1u

static uint16_t g_energy = 0;
static uint16_t g_duration = 0;
static uint16_t g_repeat_tmr = 0;
static KeyID_t  g_active_id = K_ID_NONE;
static _Bool    g_is_holding = 0;
static uint8_t  s_dbc_cnt = 0;       /* 防抖：当前键码连续收到次数 */
static KeyID_t  s_dbc_id  = K_ID_NONE; /* 防抖：正在计数的键码 */

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
    // 1. 原始信号解析 + 防抖确认
    if (g_rf_msg.vld) {
        __disable_irq();
        uint8_t r_key = g_rf_msg.key;
        g_rf_msg.vld = false;
        __enable_irq();

        KeyID_t new_id = Map_RF(r_key);
        if (new_id != K_ID_NONE) {
            /* 防抖：同一键码累计；换键或噪声（不同码）重置计数 */
            if (new_id == s_dbc_id) {
                if (s_dbc_cnt < RF_DEBOUNCE_THRESH) s_dbc_cnt++;
            } else {
                s_dbc_id  = new_id;
                s_dbc_cnt = 1;
            }

            /* 连续收到 RF_DEBOUNCE_THRESH 包才确认按键有效 */
            if (s_dbc_cnt >= RF_DEBOUNCE_THRESH) {
                if (g_active_id == K_ID_NONE) {
                    g_active_id  = new_id;
                    g_duration   = 0;
                    g_is_holding = 0;
                    g_repeat_tmr = 0;
                    Keypad_On_Event(g_active_id, K_EVT_PRESS);
                } else if (new_id != g_active_id) {
                    /* 快按换键：先 RELEASE 旧键，再 PRESS 新键 */
                    Keypad_On_Event(g_active_id, K_EVT_RELEASE);
                    g_active_id  = new_id;
                    g_duration   = 0;
                    g_is_holding = 0;
                    g_repeat_tmr = 0;
                    Keypad_On_Event(g_active_id, K_EVT_PRESS);
                }
            }
            g_energy = RF_SUSTAIN_TICKS;
        } else {
            /* 收到无效码（噪声）：重置防抖，不影响已确认的 active_id */
            s_dbc_cnt = 0;
            s_dbc_id  = K_ID_NONE;
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
        /* 超时松开：同步清防抖状态，避免下次重用旧键码跳过防抖 */
        s_dbc_cnt   = 0;
        s_dbc_id    = K_ID_NONE;
        Keypad_On_Event(g_active_id, K_EVT_RELEASE);
        g_active_id  = K_ID_NONE;
        g_duration   = 0;
        g_is_holding = 0;
    }
}

void Keypad_On_Event(KeyID_t id, KeyEvt_t evt) {

    // 先传给业务层，由业务层决定是否有效
    bool consumed = Treadmill_On_Event((uint8_t)id, (uint8_t)evt);

    if (evt == K_EVT_PRESS && consumed && Treadmill_LowerBoardReady()) {
        /* 全显/参数下发期间屏蔽按键蜂鸣；进减速时 Enter_Stopping 已先响，此处去重 */
        if (!Treadmill_ConsumeKeyBeepSuppress()) {
            Beep_Play(BEEP_CLICK);
        }
    }
}

