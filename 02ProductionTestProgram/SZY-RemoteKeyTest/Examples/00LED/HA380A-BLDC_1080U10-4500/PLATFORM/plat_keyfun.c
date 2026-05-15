#include "plat_keyfun.h"
#include "plat_rf.h"
#include "app_remote_demo.h"
#if TM_RF_PAIRING_ENABLE
#include "plat_rf_store.h"
#include "plat_beep.h"
#endif

#define RF_SUSTAIN_TICKS     20u
#define HOLD_START_TICKS    100u
#define REPEAT_RATE_TICKS     30u
#define RF_DEBOUNCE_THRESH    1u

static uint16_t g_energy = 0;
static uint16_t g_duration = 0;
static uint16_t g_repeat_tmr = 0;
static KeyID_t  g_active_id = K_ID_NONE;
static _Bool    g_is_holding = 0;
static uint8_t  s_dbc_cnt = 0;
static KeyID_t  s_dbc_id  = K_ID_NONE;

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
            if (new_id == s_dbc_id) {
                if (s_dbc_cnt < RF_DEBOUNCE_THRESH) s_dbc_cnt++;
            } else {
                s_dbc_id  = new_id;
                s_dbc_cnt = 1;
            }

            if (s_dbc_cnt >= RF_DEBOUNCE_THRESH) {
                if (g_active_id == K_ID_NONE) {
                    g_active_id  = new_id;
                    g_duration   = 0;
                    g_is_holding = 0;
                    g_repeat_tmr = 0;
                    Keypad_On_Event(g_active_id, K_EVT_PRESS);
                } else if (new_id != g_active_id) {
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
            s_dbc_cnt = 0;
            s_dbc_id  = K_ID_NONE;
        }
    }

    if (g_active_id != K_ID_NONE) {
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
            s_dbc_cnt   = 0;
            s_dbc_id    = K_ID_NONE;
            Keypad_On_Event(g_active_id, K_EVT_RELEASE);
            g_active_id  = K_ID_NONE;
            g_duration   = 0;
            g_is_holding = 0;
        }
    }

#if TM_RF_PAIRING_ENABLE
    RF_Pair_Tick();
#endif
}

void Keypad_On_Event(KeyID_t id, KeyEvt_t evt) {
    App_RemoteDemo_OnKey(id, evt);
}
