/**
 * @file plat_keylink_rx.c
 * @brief 显示板：UART2 PB03 收按键板 Keylink，与按键板 plat_keylink 发帧一致
 */
#include "plat_keylink_rx.h"
#include "bsp_uart.h"
#include "plat_keyfun.h"
#include "plat_beep.h"
#include "treadmills.h"

#define KEYLK_SOF             0x55u
#define KEYLK_EOF             0xAAu
#define KEYLK_TYPE_KEYDOWN    0x10u
#define KEYLK_TYPE_KEYUP      0x11u
#define KEYLK_TYPE_HEARTBEAT  0x20u
#define KEYLK_TYPE_SPEED0P1   0x30u
/** 显示板 → 按键板：链路透传（PB02 TX），与按键板 0x20 心跳方向相反 */
#define KEYLK_TYPE_DISP_LINK  0x22u

#define KEYLK_DATA_START      0x01u
#define KEYLK_DATA_STOP       0x02u
#define KEYLK_DATA_SPD_UP     0x03u
#define KEYLK_DATA_SPD_DOWN   0x04u
#define KEYLK_DATA_MODE       0x05u
#define KEYLK_DATA_PROG       0x06u
#define KEYLK_DATA_INC_UP     0x07u
#define KEYLK_DATA_INC_DOWN   0x08u
#define KEYLK_DATA_HOTKEY_4K  0x09u
#define KEYLK_DATA_HOTKEY_6K  0x0Au
#define KEYLK_DATA_HOTKEY_8K  0x0Bu
#define KEYLK_DATA_HOTKEY_10K 0x0Cu
#define KEYLK_DATA_HOTKEY_2K  0x0Du
#define KEYLK_DATA_HOTKEY_5K  0x0Eu
#define KEYLK_DATA_HOTKEY_12K 0x0Fu

#define KEYLK_REPEAT_NONE     0xFFu

static uint16_t s_keylink_wd_ticks;

void KeylinkRx_OnValidFrame(void)
{
    s_keylink_wd_ticks = 0;
}

void KeylinkRx_ResetWatchdog(void)
{
    s_keylink_wd_ticks = 0;
}

uint8_t KeylinkRx_WatchdogExpired100ms(uint8_t monitor_active)
{
    if (!monitor_active) {
        s_keylink_wd_ticks = 0;
        return 0u;
    }
    if (s_keylink_wd_ticks < KEYLINK_RX_WATCHDOG_MAX)
        s_keylink_wd_ticks++;
    return (s_keylink_wd_ticks >= KEYLINK_RX_WATCHDOG_MAX) ? 1u : 0u;
}

static uint8_t s_spd_saw_hold = 0;

static int prv_is_speed_key_data(uint8_t d)
{
    switch (d) {
    case KEYLK_DATA_SPD_UP:
    case KEYLK_DATA_SPD_DOWN:
    case KEYLK_DATA_INC_UP:
    case KEYLK_DATA_INC_DOWN:
        return 1;
    default:
        return 0;
    }
}

static uint8_t s_rx_st = 0;
static uint8_t s_rx_type;
static uint8_t s_rx_data;
static uint8_t s_repeat_data = KEYLK_REPEAT_NONE;

static KeyID_t prv_map_data_to_id(uint8_t d)
{
    switch (d) {
    case KEYLK_DATA_SPD_UP:   return K_ID_UP;
    case KEYLK_DATA_SPD_DOWN: return K_ID_DN;
    case KEYLK_DATA_MODE:     return K_ID_MODE;
    case KEYLK_DATA_PROG:     return K_ID_P;
    case KEYLK_DATA_INC_UP:   return K_ID_UP;
    case KEYLK_DATA_INC_DOWN: return K_ID_DN;
    default: return K_ID_NONE;
    }
}

static void prv_dispatch_start(void)
{
    if (g_TM.state == TM_STATE_BOOT || g_TM.state == TM_STATE_IDLE || g_TM.state == TM_STATE_SETTING)
        Keypad_On_Event(K_ID_ONOFF, K_EVT_PRESS);
}

static void prv_dispatch_stop(void)
{
    if (g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_COUNTDOWN)
        Keypad_On_Event(K_ID_ONOFF, K_EVT_PRESS);
}

static void prv_hotkey_speed(uint8_t d, uint16_t *out)
{
    switch (d) {
    case KEYLK_DATA_HOTKEY_2K:  *out = 20;  break;
    case KEYLK_DATA_HOTKEY_4K:  *out = 40;  break;
    case KEYLK_DATA_HOTKEY_5K:  *out = 50;  break;
    case KEYLK_DATA_HOTKEY_6K:  *out = 60;  break;
    case KEYLK_DATA_HOTKEY_8K:  *out = 80;  break;
    case KEYLK_DATA_HOTKEY_10K: *out = 100; break;
    case KEYLK_DATA_HOTKEY_12K: *out = 120; break;
    default: *out = 0; break;
    }
}

static void prv_on_framed(uint8_t type, uint8_t data)
{
    if (type == KEYLK_TYPE_DISP_LINK)
        return;
    /* E17：安全锁故障时屏蔽按键板除链路透传外的所有键值与调速 */
    if (g_TM.state == TM_STATE_ERROR && g_TM.error_code == TM_ERR_SAFETY_LOCK)
        return;
    if (type == KEYLK_TYPE_HEARTBEAT) {
        Treadmill_Keylink_SetHeartRate(data);
        return;
    }
    if (type == KEYLK_TYPE_SPEED0P1) {
        Treadmill_Keylink_SetSpeed0p1((uint16_t)data);
        s_repeat_data = KEYLK_REPEAT_NONE;
        return;
    }

    if (type != KEYLK_TYPE_KEYDOWN && type != KEYLK_TYPE_KEYUP)
        return;

    if (type == KEYLK_TYPE_KEYDOWN) {
        if (data == KEYLK_DATA_START) {
            prv_dispatch_start();
            s_repeat_data = KEYLK_REPEAT_NONE;
            return;
        }
        if (data == KEYLK_DATA_STOP) {
            prv_dispatch_stop();
            s_repeat_data = KEYLK_REPEAT_NONE;
            return;
        }
        {
            uint16_t hk = 0;
            prv_hotkey_speed(data, &hk);
            if (hk != 0u) {
                if (data == KEYLK_DATA_HOTKEY_4K || data == KEYLK_DATA_HOTKEY_6K ||
                    data == KEYLK_DATA_HOTKEY_8K || data == KEYLK_DATA_HOTKEY_10K)
                    Beep_Play(BEEP_CLICK);
                Treadmill_Keylink_ApplyHotkey(hk);
                s_repeat_data = KEYLK_REPEAT_NONE;
                return;
            }
        }
        {
            KeyID_t id = prv_map_data_to_id(data);
            if (id != K_ID_NONE) {
                if (data == s_repeat_data) {
                    Keypad_On_Event(id, K_EVT_HOLD);
                    if (prv_is_speed_key_data(data))
                        s_spd_saw_hold = 1;
                } else {
                    if (s_repeat_data != KEYLK_REPEAT_NONE) {
                        KeyID_t old_id = prv_map_data_to_id(s_repeat_data);
                        if (old_id != K_ID_NONE)
                            Keypad_On_Event(old_id, K_EVT_RELEASE);
                    }
                    s_repeat_data = data;
                    if (g_TM.state == TM_STATE_RUNNING && prv_is_speed_key_data(data))
                        s_spd_saw_hold = 0;
                    else
                        Keypad_On_Event(id, K_EVT_PRESS);
                }
            }
        }
        return;
    }

    if (data == KEYLK_DATA_START || data == KEYLK_DATA_STOP) {
        s_repeat_data = KEYLK_REPEAT_NONE;
        return;
    }
    {
        uint16_t hk = 0;
        prv_hotkey_speed(data, &hk);
        if (hk != 0u) {
            s_repeat_data = KEYLK_REPEAT_NONE;
            return;
        }
    }
    {
        KeyID_t id = prv_map_data_to_id(data);
        if (id != K_ID_NONE && data == s_repeat_data) {
            if (g_TM.state == TM_STATE_RUNNING && prv_is_speed_key_data(data) && !s_spd_saw_hold)
                Keypad_On_Event(id, K_EVT_PRESS);
            Keypad_On_Event(id, K_EVT_RELEASE);
            s_repeat_data   = KEYLK_REPEAT_NONE;
            s_spd_saw_hold = 0;
        }
    }
}

static void prv_tx_display_link(void)
{
    uint8_t t = KEYLK_TYPE_DISP_LINK;
    uint8_t d = 0u;
    uint8_t x = (uint8_t)(t ^ d);
    BSP_Uart_WriteByte(CW_UART2, KEYLK_SOF);
    BSP_Uart_WriteByte(CW_UART2, t);
    BSP_Uart_WriteByte(CW_UART2, d);
    BSP_Uart_WriteByte(CW_UART2, x);
    BSP_Uart_WriteByte(CW_UART2, KEYLK_EOF);
}

void KeylinkRx_OnTimer100ms(void)
{
    static uint8_t div;
    if (++div < 5u)
        return;
    div = 0u;
    prv_tx_display_link();
}

void KeylinkRx_SendLinkPing(void)
{
    prv_tx_display_link();
}

void KeylinkRx_SendTargetSpeed0p1(uint8_t speed_0p1)
{
    uint8_t t = KEYLK_TYPE_SPEED0P1;
    uint8_t d = speed_0p1;
    uint8_t x = (uint8_t)(t ^ d);
    BSP_Uart_WriteByte(CW_UART2, KEYLK_SOF);
    BSP_Uart_WriteByte(CW_UART2, t);
    BSP_Uart_WriteByte(CW_UART2, d);
    BSP_Uart_WriteByte(CW_UART2, x);
    BSP_Uart_WriteByte(CW_UART2, KEYLK_EOF);
}

void KeylinkRx_Handler_1ms(void)
{
    uint8_t b;

    while (AeroBuf_Pop(&uart2_rx_buf, &b)) {
        switch (s_rx_st) {
        case 0:
            if (b == KEYLK_SOF)
                s_rx_st = 1;
            break;
        case 1:
            s_rx_type = b;
            s_rx_st   = 2;
            break;
        case 2:
            s_rx_data = b;
            s_rx_st   = 3;
            break;
        case 3:
            if (((uint8_t)(s_rx_type ^ s_rx_data)) != b) {
                s_rx_st = 0;
            } else {
                s_rx_st = 4;
            }
            break;
        case 4:
            if (b == KEYLK_EOF) {
                KeylinkRx_OnValidFrame();
                prv_on_framed(s_rx_type, s_rx_data);
            }
            s_rx_st = 0;
            break;
        default:
            s_rx_st = 0;
            break;
        }
    }
}
