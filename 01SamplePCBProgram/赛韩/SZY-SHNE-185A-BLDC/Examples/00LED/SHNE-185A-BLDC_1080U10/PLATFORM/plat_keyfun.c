#include "plat_keyfun.h"
#include "plat_rf.h"
#include "plat_beep.h"
#include "treadmills.h"

#if PLAT_USE_TOUCHKEY
#include "plat_touchkey.h"
#endif

/* 射频专用手感 (10ms)；触摸的长按/连发见 Touchkey_Params_t */
#define RF_SUSTAIN_TICKS     30
#define HOLD_START_TICKS    150
#define REPEAT_RATE_TICKS    30

typedef enum {
    KEY_SRC_NONE = 0,
    KEY_SRC_TOUCH,
    KEY_SRC_RF
} KeyInSrc_t;

static uint16_t   g_energy = 0;
static uint16_t   g_duration = 0;
static uint16_t   g_repeat_tmr = 0;
static KeyID_t    g_active_id = K_ID_NONE;
static KeyInSrc_t g_src       = KEY_SRC_NONE;

static KeyID_t Map_RF(uint8_t c) {
    switch (c) {
        case 0x04: case 0x2E: return K_ID_UP;
        case 0x05: case 0x4C: return K_ID_DN;
        case 0x1A: case 0x5B: return K_ID_ONOFF;
        case 0x01: case 0x3D: return K_ID_MODE;
        default: return K_ID_NONE;
    }
}

static uint16_t prv_hold_start(void)
{
#if PLAT_USE_TOUCHKEY && PLAT_USE_RF433
    if (g_src == KEY_SRC_TOUCH)
        return Touchkey_GetHoldStartTicks();
    return HOLD_START_TICKS;
#elif PLAT_USE_TOUCHKEY
    return Touchkey_GetHoldStartTicks();
#else
    return HOLD_START_TICKS;
#endif
}

static uint8_t prv_repeat_rate(void)
{
#if PLAT_USE_TOUCHKEY && PLAT_USE_RF433
    if (g_src == KEY_SRC_TOUCH)
        return Touchkey_GetRepeatTicks();
    return REPEAT_RATE_TICKS;
#elif PLAT_USE_TOUCHKEY
    return Touchkey_GetRepeatTicks();
#else
    return REPEAT_RATE_TICKS;
#endif
}

static uint8_t prv_sustain(void)
{
#if PLAT_USE_TOUCHKEY && PLAT_USE_RF433
    if (g_src == KEY_SRC_TOUCH)
        return Touchkey_GetSustainTicks();
    return RF_SUSTAIN_TICKS;
#elif PLAT_USE_TOUCHKEY
    return Touchkey_GetSustainTicks();
#else
    return RF_SUSTAIN_TICKS;
#endif
}

void Keypad_Handler_10ms(void) {
    KeyID_t touch_id = K_ID_NONE;
#if PLAT_USE_TOUCHKEY
    touch_id = Touchkey_Handler_10ms();
#endif

    KeyID_t rf_id = K_ID_NONE;
#if PLAT_USE_RF433
    if (g_rf_msg.vld) {
        __disable_irq();
        uint8_t r_key = g_rf_msg.key;
        g_rf_msg.vld = false;
        __enable_irq();
        rf_id = Map_RF(r_key);
    }
#endif

    KeyID_t new_id = K_ID_NONE;
#if PLAT_USE_TOUCHKEY && PLAT_USE_RF433
    if (touch_id != K_ID_NONE)
        new_id = touch_id;
    else if (rf_id != K_ID_NONE)
        new_id = rf_id;
#elif PLAT_USE_TOUCHKEY
    new_id = touch_id;
#else
    new_id = rf_id;
#endif

    /* 故障界面：不响应按键、不触发按键音；清空内部按键状态以免误积累 */
    if (g_TM.state == TM_STATE_ERROR) {
        g_active_id   = K_ID_NONE;
        g_energy       = 0;
        g_duration     = 0;
        g_repeat_tmr   = 0;
        g_src          = KEY_SRC_NONE;
        return;
    }

    if (new_id != K_ID_NONE) {
        if (g_active_id == K_ID_NONE) {
            g_active_id = new_id;
            g_duration = 0;
            g_repeat_tmr = 0;
#if PLAT_USE_TOUCHKEY && PLAT_USE_RF433
            g_src = (touch_id == new_id) ? KEY_SRC_TOUCH : KEY_SRC_RF;
#elif PLAT_USE_TOUCHKEY
            g_src = KEY_SRC_TOUCH;
#else
            g_src = KEY_SRC_RF;
#endif
            g_energy = prv_sustain();
            Keypad_On_Event(g_active_id, K_EVT_PRESS);
        } else if (new_id == g_active_id) {
            g_energy = prv_sustain();
        }
    }

    if (g_active_id == K_ID_NONE) return;

    if (g_energy > 0) {
        g_energy--;
        g_duration++;

        if (g_duration >= prv_hold_start()) {
            if (++g_repeat_tmr >= prv_repeat_rate()) {
                g_repeat_tmr = 0;
                Keypad_On_Event(g_active_id, K_EVT_HOLD);
            }
        }
    } else {
        Keypad_On_Event(g_active_id, K_EVT_RELEASE);
        g_active_id = K_ID_NONE;
        g_duration = 0;
        g_src = KEY_SRC_NONE;
    }
}

void Keypad_On_Event(KeyID_t id, KeyEvt_t evt) {
    if (g_TM.state == TM_STATE_ERROR)
        return;

    bool consumed = Treadmill_On_Event((uint8_t)id, (uint8_t)evt);

    if (evt == K_EVT_PRESS && consumed && !Treadmill_ConsumeKeyBeepSuppress())
        Beep_Play(BEEP_CLICK);
}
