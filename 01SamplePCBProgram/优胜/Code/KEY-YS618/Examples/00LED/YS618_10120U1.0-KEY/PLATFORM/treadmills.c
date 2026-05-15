#include "treadmills.h"
#include "plat_touchkey.h"
#include "plat_gpio_keys.h"
#include "plat_aerolink.h"
#include "plat_ble.h"
#if PLAT_USE_KEYLINK
#include "plat_keylink.h"
#endif
#include "plat_hr_tm998.h"

#define TM_BOOT_TICKS        10u
#define TM_HEARTBEAT_CYCLES  5u

TM_Control_t g_TM;

void Treadmill_Init(void)
{
    g_TM.state       = TM_STATE_BOOT;
    g_TM.step_rate   = 0;
    g_TM.timer_100ms = 0;
    g_TM.speed_0p1   = 0;
    g_TM.dist_mm     = 0;
    g_TM.kcal        = 0;
    g_TM.time_s      = 0;
    g_TM.hr_bpm      = 0;
    g_TM.app_ctrl_ok = 0;
}

void AeroLink_OnFrameReceived(AeroFrame_t *f)
{
    if (f->fc == 0x01u && f->sfc == 0x01u) {
        g_TM.step_rate = (uint16_t)((uint16_t)f->data[4] << 8 | f->data[5]);
        /* 心率改由 TM998(PB04) 检测并经 Keylink 上发；不再用下位步频冒充 BPM */
        Ble_NotifyMetricsFromDisplay();
        return;
    }
    if (f->fc == 0x01u && f->sfc == 0x00u && g_TM.state != TM_STATE_BOOT) {
        uint16_t spd = (uint16_t)((uint16_t)f->data[0] << 8 | f->data[1]);
        uint8_t  st  = f->data[2];

        g_TM.speed_0p1 = spd;
        if (st == 9u)
            g_TM.state = TM_STATE_RUNNING;
        else if (st == 7u)
            g_TM.state = TM_STATE_IDLE;

        g_TM.kcal = (uint16_t)((g_TM.dist_mm / 1000u) * 7u / 100u);
        Ble_NotifyMetricsFromDisplay();
    }
}

void Treadmill_Manager_100ms(void)
{
    static uint16_t hb = 0;
    static uint8_t  sec_div = 0;

    Ble_OnTimer100ms();

    if (g_TM.state == TM_STATE_BOOT) {
        if (++g_TM.timer_100ms >= TM_BOOT_TICKS) {
            g_TM.state       = TM_STATE_IDLE;
            g_TM.timer_100ms = 0;
        }
        return;
    }

    if (g_TM.state == TM_STATE_RUNNING && g_TM.speed_0p1 > 0u) {
        g_TM.dist_mm += ((uint32_t)g_TM.speed_0p1 * 1000u) / 36u;
        g_TM.kcal = (uint16_t)((g_TM.dist_mm / 1000u) * 7u / 100u);
        if (++sec_div >= 10u) {
            sec_div = 0;
            if (g_TM.time_s < 0xFFFFFFFEu)
                g_TM.time_s++;
        }
    } else
        sec_div = 0;

    if (++hb >= TM_HEARTBEAT_CYCLES) {
        hb = 0;
#if PLAT_USE_KEYLINK
        Keylink_SendHeartbeat(g_TM.hr_bpm);
#endif
    }
}

static void prv_apply_hotkey_speed(uint16_t sp)
{
    g_TM.speed_0p1 = sp;
#if PLAT_USE_KEYLINK
    Keylink_SendSpeed0p1((uint8_t)((sp > 255u) ? 255u : sp));
#endif
    Ble_NotifyMetricsFromDisplay();
}

void Treadmill_On_Event(uint8_t key_id, uint8_t evt)
{
    if (evt == K_EVT_RELEASE)
        return;
    if (evt == K_EVT_HOLD)
        return;
    if (evt != K_EVT_PRESS)
        return;

    if (g_TM.state == TM_STATE_BOOT) {
        if (key_id == K_ID_ON || key_id == K_ID_MODE) {
            g_TM.state       = TM_STATE_IDLE;
            g_TM.timer_100ms = 0;
        }
        return;
    }

    if (key_id == K_ID_ON)
        g_TM.state = TM_STATE_RUNNING;
    else if (key_id == K_ID_OFF)
        g_TM.state = TM_STATE_IDLE;
    else if (g_TM.state == TM_STATE_RUNNING) {
        /* 运行中加减速由显示板计算，经下行 0x30 同步到本地 speed_0p1 */
        if (key_id == K_ID_HK_2)
            prv_apply_hotkey_speed(20);
        else if (key_id == K_ID_HK_4)
            prv_apply_hotkey_speed(40);
        else if (key_id == K_ID_HK_5)
            prv_apply_hotkey_speed(50);
        else if (key_id == K_ID_HK_6)
            prv_apply_hotkey_speed(60);
        else if (key_id == K_ID_HK_8)
            prv_apply_hotkey_speed(80);
        else if (key_id == K_ID_HK_10)
            prv_apply_hotkey_speed(100);
        else if (key_id == K_ID_HK_12)
            prv_apply_hotkey_speed(120);
    }
}

#define KP_FALLBACK_HOLD_START   100
#define KP_FALLBACK_REPEAT       15
#define KP_SPD_LONG_TICKS        100u
#define KP_SPD_REPEAT_TICKS      15u
#define KP_FALLBACK_SUSTAIN      4

static uint16_t   s_kp_energy = 0;
static uint16_t   s_kp_duration = 0;
static uint16_t   s_kp_repeat_tmr = 0;
static KeyID_t    s_kp_active_id = K_ID_NONE;

static uint16_t prv_kp_hold_start(void)
{
#if PLAT_USE_TOUCHKEY
    return Touchkey_GetHoldStartTicks();
#else
    return KP_FALLBACK_HOLD_START;
#endif
}

static uint8_t prv_kp_repeat_rate(void)
{
#if PLAT_USE_TOUCHKEY
    return Touchkey_GetRepeatTicks();
#else
    return KP_FALLBACK_REPEAT;
#endif
}

static uint8_t prv_kp_sustain(void)
{
#if PLAT_USE_TOUCHKEY
    return Touchkey_GetSustainTicks();
#else
    return KP_FALLBACK_SUSTAIN;
#endif
}

static int prv_kp_is_spd(KeyID_t id)
{
    return (id == K_ID_UP || id == K_ID_DN) ? 1 : 0;
}

static uint16_t prv_kp_hold_gate(KeyID_t id)
{
    if (g_TM.state == TM_STATE_RUNNING && prv_kp_is_spd(id))
        return KP_SPD_LONG_TICKS;
    return prv_kp_hold_start();
}

static uint8_t prv_kp_repeat_gate(KeyID_t id)
{
    if (g_TM.state == TM_STATE_RUNNING && prv_kp_is_spd(id))
        return (uint8_t)KP_SPD_REPEAT_TICKS;
    return prv_kp_repeat_rate();
}

static void prv_keypad_on_event(KeyID_t id, KeyEvt_t evt)
{
#if PLAT_USE_KEYLINK
    Keylink_OnKeypadEvent(id, evt);
#endif
    Treadmill_On_Event((uint8_t)id, (uint8_t)evt);
    Ble_OnPanelKey(id, evt);
}

void Keypad_Handler_10ms(void)
{
    KeyID_t new_id = K_ID_NONE;

#if PLAT_USE_TOUCHKEY
    new_id = Touchkey_Handler_10ms();
#endif
#if PLAT_USE_GPIO_KEYS
    if (new_id == K_ID_NONE)
        new_id = GpioKeys_Handler_10ms();
#endif

    if (new_id != K_ID_NONE) {
        if (s_kp_active_id == K_ID_NONE) {
            s_kp_active_id  = new_id;
            s_kp_duration   = 0;
            s_kp_repeat_tmr = 0;
            s_kp_energy     = prv_kp_sustain();
            prv_keypad_on_event(s_kp_active_id, K_EVT_PRESS);
        } else if (new_id == s_kp_active_id) {
            s_kp_energy = prv_kp_sustain();
        }
    }

    if (s_kp_active_id != K_ID_NONE) {
        if (s_kp_energy > 0) {
            s_kp_energy--;
            s_kp_duration++;
            if (s_kp_duration >= prv_kp_hold_gate(s_kp_active_id)) {
                if (++s_kp_repeat_tmr >= prv_kp_repeat_gate(s_kp_active_id)) {
                    s_kp_repeat_tmr = 0;
                    prv_keypad_on_event(s_kp_active_id, K_EVT_HOLD);
                }
            }
        } else {
            prv_keypad_on_event(s_kp_active_id, K_EVT_RELEASE);
            s_kp_active_id  = K_ID_NONE;
            s_kp_duration   = 0;
        }
    }

#if PLAT_USE_HR_TM998
    HrTm998_OnKeypadScan_10ms();
#endif
}
