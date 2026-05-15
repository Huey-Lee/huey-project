#include "plat_keylink.h"
#include "bsp_uart.h"
#include "sys_aerobuf.h"
#include "treadmills.h"
#include "plat_ble.h"

static void prv_send(uint8_t type, uint8_t data)
{
    uint8_t x = (uint8_t)(type ^ data);
    BSP_Uart_WriteByte(CW_UART1, KEYLK_SOF);
    BSP_Uart_WriteByte(CW_UART1, type);
    BSP_Uart_WriteByte(CW_UART1, data);
    BSP_Uart_WriteByte(CW_UART1, x);
    BSP_Uart_WriteByte(CW_UART1, KEYLK_EOF);
}

static uint8_t prv_map_key_data(KeyID_t id)
{
    switch (id) {
    case K_ID_ON:
        return KEYLK_DATA_START;
    case K_ID_OFF:
        return KEYLK_DATA_STOP;
    case K_ID_UP:
        return KEYLK_DATA_SPD_UP;
    case K_ID_DN:
        return KEYLK_DATA_SPD_DOWN;
    case K_ID_MODE:
        return KEYLK_DATA_MODE;
    case K_ID_P:
        return KEYLK_DATA_PROG;
    case K_ID_INC_UP:
        return KEYLK_DATA_INC_UP;
    case K_ID_INC_DOWN:
        return KEYLK_DATA_INC_DOWN;
    case K_ID_HK_2:
        return KEYLK_DATA_HOTKEY_2K;
    case K_ID_HK_4:
        return KEYLK_DATA_HOTKEY_4K;
    case K_ID_HK_5:
        return KEYLK_DATA_HOTKEY_5K;
    case K_ID_HK_6:
        return KEYLK_DATA_HOTKEY_6K;
    case K_ID_HK_8:
        return KEYLK_DATA_HOTKEY_8K;
    case K_ID_HK_10:
        return KEYLK_DATA_HOTKEY_10K;
    case K_ID_HK_12:
        return KEYLK_DATA_HOTKEY_12K;
    default:
        return 0;
    }
}

static uint8_t s_last_key_data   = 0;
static uint8_t s_key_down_active = 0;

void Keylink_OnKeypadEvent(KeyID_t id, KeyEvt_t evt)
{
    if (evt == K_EVT_RELEASE) {
        if (s_key_down_active) {
            prv_send(KEYLK_TYPE_KEYUP, s_last_key_data);
            s_key_down_active = 0;
        }
        return;
    }

    {
        uint8_t d = prv_map_key_data(id);
        if (d == 0)
            return;

        if (evt == K_EVT_PRESS) {
            s_last_key_data     = d;
            s_key_down_active   = 1;
            prv_send(KEYLK_TYPE_KEYDOWN, d);
            return;
        }
        if (evt == K_EVT_HOLD) {
            if (!s_key_down_active) {
                s_last_key_data   = d;
                s_key_down_active = 1;
            }
            prv_send(KEYLK_TYPE_KEYDOWN, d);
        }
    }
}

void Keylink_SendHeartbeat(uint8_t bpm)
{
    prv_send(KEYLK_TYPE_HEARTBEAT, bpm);
}

uint8_t Keylink_IsPanelKeyDown(void)
{
    return s_key_down_active;
}

void Keylink_SendSpeed0p1(uint8_t speed_0p1)
{
    prv_send(KEYLK_TYPE_SPEED0P1, speed_0p1);
}

void Keylink_SimKeyTap(KeyID_t id)
{
    Keylink_OnKeypadEvent(id, K_EVT_PRESS);
    Keylink_OnKeypadEvent(id, K_EVT_RELEASE);
}

#if PLAT_USE_KEYLINK

static uint8_t s_d_st;
static uint8_t s_d_type;
static uint8_t s_d_data;
static uint16_t s_disp_idle_ms;
static uint8_t s_disp_link_ok;

void Keylink_DisplayPoll_1ms(void)
{
    uint8_t b;

    if (s_disp_idle_ms < 0xFFFFu)
        s_disp_idle_ms++;

    while (AeroBuf_Pop(&uart1_rx_buf, &b)) {
        switch (s_d_st) {
        case 0:
            if (b == KEYLK_SOF)
                s_d_st = 1u;
            break;
        case 1:
            s_d_type = b;
            s_d_st   = 2u;
            break;
        case 2:
            s_d_data = b;
            s_d_st   = 3u;
            break;
        case 3:
            if (((uint8_t)(s_d_type ^ s_d_data)) != b)
                s_d_st = 0u;
            else
                s_d_st = 4u;
            break;
        case 4:
            if (b == KEYLK_EOF) {
                if (s_d_type == KEYLK_TYPE_DISP_LINK && s_d_data == 0u) {
                    s_disp_link_ok  = 1u;
                    s_disp_idle_ms = 0u;
                } else if (s_d_type == KEYLK_TYPE_SPEED0P1) {
                    uint16_t v = (uint16_t)s_d_data;
                    if (v < TM_SPEED_0P1_MIN)
                        v = TM_SPEED_0P1_MIN;
                    if (v > TM_SPEED_0P1_MAX)
                        v = TM_SPEED_0P1_MAX;
                    g_TM.speed_0p1 = v;
                    Ble_NotifyMetricsFromDisplay();
                }
            }
            s_d_st = 0u;
            break;
        default:
            s_d_st = 0u;
            break;
        }
    }

    if (s_disp_idle_ms >= 2000u)
        s_disp_link_ok = 0u;
}

uint8_t Keylink_DisplayLinkOk(void)
{
    return s_disp_link_ok;
}

#else

void Keylink_DisplayPoll_1ms(void) { }

uint8_t Keylink_DisplayLinkOk(void)
{
    return 0u;
}

uint8_t Keylink_IsPanelKeyDown(void)
{
    return 0u;
}

#endif
