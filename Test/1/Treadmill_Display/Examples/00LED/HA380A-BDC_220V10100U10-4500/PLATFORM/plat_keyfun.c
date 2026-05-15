#include "plat_keyfun.h"
#include "plat_rf.h"
#include "plat_beep.h"
#include "treadmills.h"
#if TM_RF_PAIRING_ENABLE
#include "plat_rf_store.h"
#endif

/* 调节按键手感参数 (10ms 为基准) */
#define RF_SUSTAIN_TICKS     16u  /* 无新包约 160ms 后算松开；要更快可 10~12u */
#define HOLD_START_TICKS    150u
#define REPEAT_RATE_TICKS     30u

static uint16_t g_energy = 0;
static uint16_t g_duration = 0;
static uint16_t g_repeat_tmr = 0;
static KeyID_t  g_active_id = K_ID_NONE;
static _Bool    g_is_holding = 0;

#if TM_RF_PAIRING_ENABLE
static uint16_t s_pair_cap_addr;

static void RF_Pair_Tick(void)
{
    static uint16_t s_elapsed_10;
    static uint16_t s_hold_10;
    static _Bool    s_pair_done;

    if (s_elapsed_10 < 0xFFFEu) {
        s_elapsed_10++;
    }

    if (s_pair_done) {
        return;
    }

    if (s_elapsed_10 > TM_RF_PAIR_WINDOW_10MS) {
        return;
    }

    if (g_active_id == K_ID_UP) {
        s_hold_10++;
        if (s_hold_10 >= TM_RF_PAIR_HOLD_10MS) {
            if (RFStore_WritePairedAddr(s_pair_cap_addr) == 0u) {
                Beep_Play(BEEP_PAIR_OK);
                s_pair_done = 1;
            } else {
                s_hold_10 = 0u;
            }
        }
    } else {
        s_hold_10 = 0;
    }
}
#endif

static KeyID_t Map_RF(uint8_t c) {
    switch (c) {
        case 0x2E: return K_ID_UP;
        case 0x4C: return K_ID_DN;
        case 0x5B: return K_ID_ONOFF;
        case 0x3D: return K_ID_MODE;
        default: return K_ID_NONE;
    }
}

void Keypad_Handler_10ms(void) {
    if (g_rf_msg.vld) {
        __disable_irq();
        uint8_t  r_key   = g_rf_msg.key;
        uint32_t rf_addr = g_rf_msg.addr;
        g_rf_msg.vld = false;
        __enable_irq();

        KeyID_t new_id = Map_RF(r_key);
#if TM_RF_PAIRING_ENABLE
        if (new_id == K_ID_UP) {
            s_pair_cap_addr = (uint16_t)(rf_addr & 0xFFFFu);
        }
#endif
        if (new_id != K_ID_NONE) {
            if (g_active_id == K_ID_NONE) {
                g_active_id = new_id;
                g_duration  = 0;
                g_is_holding  = 0;
                g_repeat_tmr  = 0;
                Keypad_On_Event(g_active_id, K_EVT_PRESS);
            } else if (new_id != g_active_id) {
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

#if TM_RF_PAIRING_ENABLE
    RF_Pair_Tick();
#endif

    if (g_active_id == K_ID_NONE) return;

    if (g_energy > 0) {
        g_energy--;
        g_duration++;

        if (g_duration >= HOLD_START_TICKS) {
            g_is_holding = 1;
            if (++g_repeat_tmr >= REPEAT_RATE_TICKS) {
                g_repeat_tmr = 0;
                Keypad_On_Event(g_active_id, K_EVT_HOLD);
            }
        }
    } else {
        _Bool was_holding = g_is_holding;
        Keypad_On_Event(g_active_id, K_EVT_RELEASE);
        if (was_holding) {
            Beep_Play(BEEP_CLICK);
        }
        g_active_id  = K_ID_NONE;
        g_duration   = 0;
        g_is_holding = 0;
    }
}

void Keypad_On_Event(KeyID_t id, KeyEvt_t evt) {

    bool consumed = Treadmill_On_Event((uint8_t)id, (uint8_t)evt);

    if (evt == K_EVT_PRESS && consumed) {
        if (!Treadmill_ConsumeKeyBeepSuppress()) {
            Beep_Play(BEEP_CLICK);
        }
    }
}
