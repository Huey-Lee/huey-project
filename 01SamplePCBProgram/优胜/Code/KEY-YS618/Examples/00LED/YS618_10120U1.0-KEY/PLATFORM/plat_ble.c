/**
 * @file plat_ble.c
 * @brief 蓝牙模组协议 V2.3（UART2 PB02/PB03），与显示板 Keylink / g_TM 同步
 */
#include "plat_ble.h"
#include "bsp_uart.h"
#include "plat_keylink.h"
#include "treadmills.h"

#define BLE_SOF_CFG  0x53u
#define BLE_EOF_CFG  0x54u
#define BLE_SOF_DAT  0x57u
#define BLE_EOF_DAT  0x58u

#define BLE_APPENDIX_IDLE  0x01u
#define BLE_APPENDIX_RUN   0x0Du

static const uint8_t s_cfg_feature[] = {
    0x53, 0x04, 0x01, 0x08, 0x04, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B, 0x54
};
static const uint8_t s_cfg_status_idle[] = {
    0x53, 0x04, 0x02, 0x01, 0x01, 0x06, 0x54
};
static const uint8_t s_cfg_speed_range[] = {
    0x53, 0x04, 0x03, 0x06, 0x64, 0x00, 0x58, 0x07, 0x0A, 0x00, 0x30, 0x54
};
static const uint8_t s_cfg_dev_type[] = {
    0x53, 0x04, 0x0A, 0x04, 0x01, 0x00, 0x02, 0x01, 0x08, 0x54
};

static const uint8_t s_sport_tpl[24] = {
    0x0D, 0x9C, 0x05, 0x70, 0x03, 0x8C, 0x00, 0x00, 0x32, 0x00, 0x00, 0x32, 0x00,
    0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x3C, 0x00
};

typedef enum {
    BLE_PHASE_WAIT_BT_INIT = 0,
    BLE_PHASE_CFG_PAUSE,
    BLE_PHASE_CFG_SEND,
    BLE_PHASE_RUNNING
} BlePhase_t;

static BlePhase_t s_phase         = BLE_PHASE_WAIT_BT_INIT;
static uint8_t    s_cfg_idx       = 0;
static uint16_t   s_cfg_timer     = 0;
static uint8_t    s_bt_link       = 0;
static uint8_t    s_sport_counter = 0;

static void prv_tx_bytes(const uint8_t *p, unsigned n)
{
    unsigned i;
    for (i = 0; i < n; i++)
        BSP_Uart_WriteByte(CW_UART2, p[i]);
}

static void prv_tx_dat(uint8_t cmd, uint8_t code, const uint8_t *data, uint8_t len)
{
    uint8_t buf[72];
    unsigned i = 0;
    uint8_t fcs;
    uint8_t k;

    if ((unsigned)len + 6u > sizeof(buf))
        return;
    buf[i++] = BLE_SOF_DAT;
    buf[i++] = cmd;
    buf[i++] = code;
    buf[i++] = len;
    fcs = (uint8_t)(cmd ^ code ^ len);
    if (data != NULL && len > 0u) {
        for (k = 0; k < len; k++) {
            buf[i++] = data[k];
            fcs ^= data[k];
        }
    }
    buf[i++] = fcs;
    buf[i++] = BLE_EOF_DAT;
    prv_tx_bytes(buf, i);
}

static void prv_cfg_ack_bt_init(void)
{
    const uint8_t pkt[6] = { BLE_SOF_CFG, 0x03u, 0x00u, 0x00u, 0x03u, BLE_EOF_CFG };
    prv_tx_bytes(pkt, sizeof(pkt));
}

static void prv_tx_cfg_raw(const uint8_t *frame, unsigned len)
{
    prv_tx_bytes(frame, len);
}

static void prv_reply_app(uint8_t code, uint8_t result)
{
    uint8_t d[1];
    d[0] = result;
    prv_tx_dat(0x03u, code, d, 1u);
}

static void prv_build_send_sport(void)
{
    uint8_t d[24];
    uint8_t i;
    uint16_t spd001 = (uint16_t)g_TM.speed_0p1 * 10u;
    uint16_t dm     = (uint16_t)(g_TM.dist_mm / 1000u);
    if (dm > 0xFFF0u)
        dm = 0xFFF0u;

    for (i = 0; i < 24u; i++)
        d[i] = s_sport_tpl[i];

    d[0] = (g_TM.state == TM_STATE_RUNNING) ? BLE_APPENDIX_RUN : BLE_APPENDIX_IDLE;
    d[3] = (uint8_t)(spd001 & 0xFFu);
    d[4] = (uint8_t)(spd001 >> 8);
    d[5] = (uint8_t)(dm & 0xFFu);
    d[6] = (uint8_t)(dm >> 8);

    d[15] = (uint8_t)(g_TM.kcal & 0xFFu);
    d[16] = (uint8_t)(g_TM.kcal >> 8);

    d[20] = g_TM.hr_bpm;
    d[21] = (uint8_t)(g_TM.time_s & 0xFFu);
    d[22] = (uint8_t)((g_TM.time_s >> 8) & 0xFFu);

    prv_tx_dat(0x01u, 0x00u, d, 24u);
}

static void prv_apply_speed_001(uint16_t v001)
{
    g_TM.speed_0p1 = (uint16_t)((v001 + 5u) / 10u);
    if (g_TM.speed_0p1 > 250u)
        g_TM.speed_0p1 = 250u;
#if PLAT_USE_KEYLINK
    Keylink_SendSpeed0p1((uint8_t)g_TM.speed_0p1);
#endif
    {
        uint8_t opd[2];
        opd[0] = (uint8_t)(v001 & 0xFFu);
        opd[1] = (uint8_t)(v001 >> 8);
        prv_tx_dat(0x02u, 0x05u, opd, 2u);
    }
    prv_build_send_sport();
}

static void prv_sim_panel_start(void)
{
#if PLAT_USE_KEYLINK
    Keylink_SimKeyTap(K_ID_ON);
#endif
    Treadmill_On_Event((uint8_t)K_ID_ON, (uint8_t)K_EVT_PRESS);
    prv_tx_dat(0x02u, 0x04u, NULL, 0u);
    prv_build_send_sport();
}

static void prv_sim_panel_stop(uint8_t mode)
{
#if PLAT_USE_KEYLINK
    Keylink_SimKeyTap(K_ID_OFF);
#endif
    Treadmill_On_Event((uint8_t)K_ID_OFF, (uint8_t)K_EVT_PRESS);
    prv_tx_dat(0x02u, 0x02u, &mode, 1u);
    prv_build_send_sport();
}

static void prv_cfg_emit_next(void)
{
    if (s_cfg_idx >= 4u) {
        s_phase = BLE_PHASE_RUNNING;
        return;
    }
    switch (s_cfg_idx) {
    case 0:
        prv_tx_cfg_raw(s_cfg_feature, (unsigned)sizeof(s_cfg_feature));
        break;
    case 1:
        prv_tx_cfg_raw(s_cfg_status_idle, (unsigned)sizeof(s_cfg_status_idle));
        break;
    case 2:
        prv_tx_cfg_raw(s_cfg_speed_range, (unsigned)sizeof(s_cfg_speed_range));
        break;
    case 3:
        prv_tx_cfg_raw(s_cfg_dev_type, (unsigned)sizeof(s_cfg_dev_type));
        break;
    default:
        break;
    }
    s_cfg_idx++;
    if (s_cfg_idx >= 4u) {
        s_phase = BLE_PHASE_RUNNING;
        s_cfg_timer = 0u;
    } else
        s_cfg_timer = 2u;
}

static void prv_handle_cfg_in(uint8_t cmd, uint8_t code, uint8_t len, const uint8_t *d)
{
    (void)d;
    if (cmd == 0x03u && code == 0x00u && len == 0u) {
        prv_cfg_ack_bt_init();
        s_phase      = BLE_PHASE_CFG_PAUSE;
        s_cfg_timer  = 15u;
        s_cfg_idx    = 0u;
    }
}

static void prv_handle_dat_frame(uint8_t cmd, uint8_t code, uint8_t len, const uint8_t *dat)
{
    if (cmd == 0x04u) {
        if (code == 0x01u && len == 0u)
            s_bt_link = 1u;
        else if (code == 0x02u && len == 0u)
            s_bt_link = 0u;
        return;
    }

    if (cmd != 0x03u)
        return;

    if (code == 0x00u && len == 0u) {
        g_TM.app_ctrl_ok = 1u;
        prv_reply_app(0x00u, 0x01u);
        return;
    }
    if (code == 0x01u && len == 0u) {
        g_TM.dist_mm = 0u;
        g_TM.kcal    = 0u;
        g_TM.time_s  = 0u;
        g_TM.speed_0p1 = 0u;
        g_TM.app_ctrl_ok = 0u;
        prv_reply_app(0x01u, 0x01u);
        prv_build_send_sport();
        return;
    }
    if (code == 0x02u && len == 2u) {
        uint16_t v = (uint16_t)dat[0] | ((uint16_t)dat[1] << 8);
        prv_apply_speed_001(v);
        prv_reply_app(0x02u, 0x01u);
        return;
    }
    if (code == 0x07u && len == 0u) {
        prv_sim_panel_start();
        prv_reply_app(0x07u, 0x01u);
        return;
    }
    if (code == 0x08u && len == 1u) {
        prv_sim_panel_stop(dat[0]);
        prv_reply_app(0x08u, 0x01u);
        return;
    }
    prv_reply_app(code, 0x02u);
}

void Ble_Init(void)
{
    s_phase          = BLE_PHASE_WAIT_BT_INIT;
    s_cfg_idx        = 0u;
    s_cfg_timer      = 0u;
    s_bt_link        = 0u;
    s_sport_counter  = 0u;
    g_TM.app_ctrl_ok = 0u;
}

void Ble_ProcessRx(void)
{
    static uint8_t st = 0;
    static uint8_t sof = 0, cmd = 0, code = 0, len = 0;
    static uint8_t data[64];
    static uint8_t dcnt = 0;
    uint8_t b;

    while (AeroBuf_Pop(&uart2_rx_buf, &b)) {
        switch (st) {
        case 0:
            if (b == BLE_SOF_CFG || b == BLE_SOF_DAT) {
                sof = b;
                st  = 1u;
            }
            break;
        case 1:
            cmd = b;
            st  = 2u;
            break;
        case 2:
            code = b;
            st   = 3u;
            break;
        case 3:
            len = b;
            dcnt = 0;
            if (len > (uint8_t)sizeof(data)) {
                st = 0;
                break;
            }
            st = (len == 0u) ? 5u : 4u;
            break;
        case 4:
            data[dcnt++] = b;
            if (dcnt >= len)
                st = 5u;
            break;
        case 5: {
            uint8_t xc = (uint8_t)(cmd ^ code ^ len);
            uint8_t k;
            for (k = 0; k < len; k++)
                xc ^= data[k];
            if (xc != b) {
                st = 0;
                break;
            }
            st = 6u;
            break;
        }
        case 6:
            if ((sof == BLE_SOF_CFG && b != BLE_EOF_CFG) || (sof == BLE_SOF_DAT && b != BLE_EOF_DAT)) {
                st = 0;
                break;
            }
            if (sof == BLE_SOF_DAT)
                prv_handle_dat_frame(cmd, code, len, data);
            else
                prv_handle_cfg_in(cmd, code, len, data);
            st = 0;
            break;
        default:
            st = 0;
            break;
        }
    }
}

void Ble_OnTimer100ms(void)
{
    if (s_phase == BLE_PHASE_CFG_PAUSE) {
        if (s_cfg_timer > 0u)
            s_cfg_timer--;
        if (s_cfg_timer == 0u) {
            s_phase     = BLE_PHASE_CFG_SEND;
            s_cfg_idx   = 0u;
            prv_cfg_emit_next();
        }
        return;
    }
    if (s_phase == BLE_PHASE_CFG_SEND) {
        if (s_cfg_timer > 0u)
            s_cfg_timer--;
        if (s_cfg_timer == 0u)
            prv_cfg_emit_next();
        return;
    }

    if (s_phase != BLE_PHASE_RUNNING || s_bt_link == 0u)
        return;

    if (++s_sport_counter >= 5u) {
        s_sport_counter = 0u;
        prv_build_send_sport();
    }
}

void Ble_OnPanelKey(KeyID_t id, KeyEvt_t evt)
{
    if (evt != K_EVT_PRESS)
        return;
    if (s_phase != BLE_PHASE_RUNNING || s_bt_link == 0u)
        return;

    switch (id) {
    case K_ID_ON:
        prv_tx_dat(0x02u, 0x04u, NULL, 0u);
        break;
    case K_ID_OFF: {
        uint8_t m = 0x01u;
        prv_tx_dat(0x02u, 0x02u, &m, 1u);
        break;
    }
    case K_ID_UP:
    case K_ID_DN:
    case K_ID_HK_2:
    case K_ID_HK_4:
    case K_ID_HK_5:
    case K_ID_HK_6:
    case K_ID_HK_8:
    case K_ID_HK_10:
    case K_ID_HK_12: {
        uint16_t v001 = (uint16_t)g_TM.speed_0p1 * 10u;
        uint8_t op[2];
        op[0] = (uint8_t)(v001 & 0xFFu);
        op[1] = (uint8_t)(v001 >> 8);
        prv_tx_dat(0x02u, 0x05u, op, 2u);
        break;
    }
    default:
        break;
    }
}

void Ble_NotifyMetricsFromDisplay(void)
{
    s_sport_counter = 4u;
}
