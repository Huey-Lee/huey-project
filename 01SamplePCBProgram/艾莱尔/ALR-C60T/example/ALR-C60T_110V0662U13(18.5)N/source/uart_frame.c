/*
 * uart_frame.c - Display to motor 17-byte BLDC frame (same layout as HA380 plat_aerolink).
 */

#include "common.h"
#include "uart_frame.h"
#include "treadmills.h"
#include "param.h"
#include "led_disp.h"
#include "indecate.h"
#include "beep.h"

void cmd_proc(void);

T_QUEUE rx_queue;
u8 rx_buf[RX_BUFF_SIZE];

extern u8 down_control;

void UART_Send_Buf(u8 UARTPort, u8 *ptr, u8 len);

/* RX state machine */
typedef enum {
    S_H1 = 0,
    S_H2,
    S_CID,
    S_FC,
    S_SFC,
    S_DATA,
    S_CSH,
    S_CSL,
    S_T1,
    S_T2
} LinkState_t;

typedef struct {
    u8 cid;
    u8 fc;
    u8 sfc;
    u8 data[8];
} AeroFrame_t;

static LinkState_t s_rxst = S_H1;
static AeroFrame_t s_rx;
static u16         s_cs_acc;
static u16         s_cs_exp;
static u8          s_didx;
static u8          s_factory_ui_phase;
/* From lower FC01/00 Data3/Data4 max/min gear (0.1 km/h units). */
static u16         s_lower_gear_max = PROTO_GEAR_TENTH_FALLBACK_MAX;
static u16         s_lower_gear_min = PROTO_GEAR_TENTH_FALLBACK_MIN;

/* 心跳约 4 帧/秒(250ms)时，UART_HB_PER_1S≈1s；用于发令后核对下位状态/齿轮并重发 */
#define UART_HB_PER_1S          (4u)
#define UART_TX_VERIFY_RETRIES  (3u)

static u8  s_vrun_pending;
static u8  s_vrun_hb;
static u8  s_vrun_retries;
static u16 s_vrun_target_internal;

static u8  s_vspd_pending;
static u8  s_vspd_hb;
static u8  s_vspd_retries;
static u16 s_vspd_target_internal;
static u16 s_vspd_target_gear;
static u16 s_vspd_ref_gear;

static u8  s_vstop_pending;
static u8  s_vstop_hb;
static u8  s_vstop_retries;

/* TX helpers */
static u16 CalcCs(u8 cid, u8 fc, u8 sfc, const u8 *d8)
{
    u16 s = (u16)cid + (u16)fc + (u16)sfc;
    u8  i;
    for (i = 0; i < 8u; i++) {
        s += (u16)d8[i];
    }
    return s;
}

static void AeroLink_Send(u8 fc, u8 sfc, const u8 *payload)
{
    u8  pay[8];
    u16 cs;
    u8  i;
    u8  out[17];

    for (i = 0; i < 8u; i++) {
        pay[i] = payload ? payload[i] : 0u;
    }
    cs = CalcCs(TREAD_CID_SEND, fc, sfc, pay);

    out[0]  = TREAD_HEAD1;
    out[1]  = TREAD_HEAD2;
    out[2]  = TREAD_CID_SEND;
    out[3]  = fc;
    out[4]  = sfc;
    for (i = 0; i < 8u; i++) {
        out[5 + i] = pay[i];
    }
    out[13] = (u8)(cs >> 8);
    out[14] = (u8)(cs & 0xFFu);
    out[15] = TREAD_TAIL1;
    out[16] = TREAD_TAIL2;
    UART_Send_Buf(UART0, out, 17u);
}

static void Send_GearWord(u16 spd01)
{
    u8 d[8];
    d[0] = (u8)(spd01 >> 8);
    d[1] = (u8)(spd01 & 0xFFu);
    d[2] = d[3] = d[4] = d[5] = d[6] = d[7] = 0u;
    AeroLink_Send(FC_GEAR, 0x00u, d);
}

static void Send_Run(u16 gear01)
{
    u8 i;
    /* Lower expects START then current gear; repeat for link robustness */
    for (i = 0u; i < 3u; i++) {
        AeroLink_Send(FC_CONTROL, SFC_CTRL_START, 0);
        Send_GearWord(gear01);
    }
}

/* dat = treadmills.param.speed / 10 (see uart_frame_tx CMD_SPEED callers). */
static u16 disp_tenth_from_cmd_dat(u8 dat)
{
    return (u16)((u16)dat * 10u / 16u);
}

/*
 * Map display 0.1 km/h band to lower gear word; bounds from RX 01/00 Data3/4,
 * fallback PROTO_GEAR_TENTH_*; disp_tenth==0 -> send 0.
 */
static u16 map_user_disp_to_protocol_gear(u16 disp_tenth)
{
    u16 d0 = USER_SPEED_DISP_TENTH_MIN;
    u16 d1 = USER_SPEED_DISP_TENTH_MAX;
    u16 p0 = s_lower_gear_min;
    u16 p1 = s_lower_gear_max;
    u32 num;
    u16 p;

    if (disp_tenth == 0u) {
        return 0u;
    }
    if (p1 <= p0) {
        p0 = PROTO_GEAR_TENTH_FALLBACK_MIN;
        p1 = PROTO_GEAR_TENTH_FALLBACK_MAX;
    }
    if (disp_tenth <= d0) {
        return p0;
    }
    if (disp_tenth >= d1) {
        return p1;
    }
    if (d1 <= d0) {
        return p0;
    }
    num = (u32)(disp_tenth - d0) * (u32)(p1 - p0);
    p   = (u16)(p0 + (num + (u32)(d1 - d0) / 2u) / (u32)(d1 - d0));
    if (p < p0) {
        p = p0;
    }
    if (p > p1) {
        p = p1;
    }
    return p;
}

/* RX: protocol gear word -> display 0.1 km/h steps */
static u16 disp_tenth_from_protocol_gear(u16 p_gear)
{
    u16 d1 = USER_SPEED_DISP_TENTH_MAX;
    u16 p0 = s_lower_gear_min;
    u16 p1 = s_lower_gear_max;

    if (p_gear == 0u) {
        return 0u;
    }
    if (p1 <= p0) {
        p0 = PROTO_GEAR_TENTH_FALLBACK_MIN;
        p1 = PROTO_GEAR_TENTH_FALLBACK_MAX;
    }
    if (p_gear >= p1) {
        return d1;
    }
    return (u16)(((u32)p_gear * (u32)d1 + (u32)p1 / 2u) / (u32)p1);
}

/* Display tenth -> internal speed; UI uses speed/16 as 0.1 km/h (0..d1 -> 0..DEFAULT_SPEED_MAX) */
static u16 speed_internal_from_disp_tenth(u16 disp)
{
    u16 d1 = USER_SPEED_DISP_TENTH_MAX;

    if (disp == 0u) {
        return 0u;
    }
    if (disp >= d1) {
        return DEFAULT_SPEED_MAX;
    }
    if (d1 == 0u) {
        return 0u;
    }
    return (u16)(((u32)disp * (u32)DEFAULT_SPEED_MAX + (u32)d1 / 2u) / (u32)d1);
}

u16 uart_prog_kmh_to_internal(u8 kmh_whole)
{
    return speed_internal_from_disp_tenth((u16)kmh_whole * 10u);
}

static u16 protocol_gear_from_cmd_dat(u8 dat)
{
    return map_user_disp_to_protocol_gear(disp_tenth_from_cmd_dat(dat));
}

/* internal_spd/16 => 0.1km/h????????????r?????? MOTOR_RUN ?? u8 dat ???????? */
static u16 protocol_gear_from_internal_spd(u16 internal_spd)
{
    return map_user_disp_to_protocol_gear((u16)(internal_spd / 16u));
}

static u8 lower_status_is_running(u8 sr)
{
    if (sr == 0u || sr == 3u || sr == 7u || sr == 10u) {
        return 0u;
    }
    return 1u;
}

static u8 lower_status_is_stopped(u8 sr, u16 gear_word)
{
    if (sr == 10u || sr == 7u) {
        return 1u;
    }
    if (gear_word == 0u) {
        return 1u;
    }
    return 0u;
}

static u16 fb_internal_from_spd_word(u16 spd_word)
{
    u16 disp = disp_tenth_from_protocol_gear(spd_word);
    return speed_internal_from_disp_tenth(disp);
}

static void uart_tx_verify_clear(void)
{
    s_vrun_pending  = 0u;
    s_vspd_pending  = 0u;
    s_vstop_pending = 0u;
    s_vrun_hb       = 0u;
    s_vspd_hb       = 0u;
    s_vstop_hb      = 0u;
}

static void motor_run_arm_and_tx(u16 internal_spd)
{
    s_vstop_pending         = 0u;
    s_vrun_target_internal  = internal_spd;
    s_vrun_pending          = 1u;
    s_vrun_hb               = 0u;
    s_vrun_retries          = UART_TX_VERIFY_RETRIES;
    Send_Run(protocol_gear_from_internal_spd(internal_spd));
}

static void speed_cmd_arm_and_tx(u8 dat)
{
    s_vspd_target_internal = speed_internal_from_disp_tenth(disp_tenth_from_cmd_dat(dat));
    s_vspd_target_gear     = protocol_gear_from_cmd_dat(dat);
    s_vspd_pending         = 1u;
    s_vspd_hb              = 0u;
    s_vspd_retries         = UART_TX_VERIFY_RETRIES;
    s_vspd_ref_gear        = 0xFFFFu;
    Send_GearWord(s_vspd_target_gear);
}

static void uart_frame_hb_tx_verify(u16 spd_word, u8 status_run)
{
    u16 fb;

    if (s_vrun_pending != 0u && treadmills.status == STATUS_RUNNING) {
        if (lower_status_is_running(status_run) != 0u) {
            s_vrun_pending = 0u;
        } else {
            s_vrun_hb++;
            if (s_vrun_hb >= UART_HB_PER_1S) {
                s_vrun_hb = 0u;
                if (s_vrun_retries > 0u) {
                    s_vrun_retries--;
                    Send_Run(protocol_gear_from_internal_spd(s_vrun_target_internal));
                } else {
                    s_vrun_pending = 0u;
                }
            }
        }
    }

    if (s_vspd_pending != 0u && treadmills.status == STATUS_RUNNING) {
        fb = fb_internal_from_spd_word(spd_word);
        if (fb + DEFAULT_SPEED_STEP >= s_vspd_target_internal &&
            fb <= s_vspd_target_internal + DEFAULT_SPEED_STEP) {
            s_vspd_pending = 0u;
        } else {
            if (s_vspd_hb == 0u || s_vspd_ref_gear == 0xFFFFu) {
                s_vspd_ref_gear = spd_word;
            }
            s_vspd_hb++;
            if (s_vspd_hb >= UART_HB_PER_1S) {
                if (spd_word == s_vspd_ref_gear) {
                    if (s_vspd_retries > 0u) {
                        s_vspd_retries--;
                        Send_GearWord(s_vspd_target_gear);
                        Send_GearWord(s_vspd_target_gear);
                        s_vspd_ref_gear = 0xFFFFu;
                    } else {
                        s_vspd_pending = 0u;
                    }
                } else {
                    s_vspd_ref_gear = spd_word;
                }
                s_vspd_hb = 0u;
            }
        }
    }

    if (s_vstop_pending != 0u &&
        (treadmills.status == STATUS_STOP || treadmills.status == STATUS_STOP_WAIT ||
         treadmills.status == STATUS_STOP_OVER)) {
        if (lower_status_is_stopped(status_run, spd_word) != 0u) {
            s_vstop_pending = 0u;
        } else {
            s_vstop_hb++;
            if (s_vstop_hb >= UART_HB_PER_1S) {
                s_vstop_hb = 0u;
                if (s_vstop_retries > 0u) {
                    s_vstop_retries--;
                    AeroLink_Send(FC_CONTROL, SFC_CTRL_STOP, 0);
                } else {
                    s_vstop_pending = 0u;
                }
            }
        }
    }
}

void uart_frame_motor_run_internal_spd(u16 internal_spd)
{
    motor_run_arm_and_tx(internal_spd);
}

static u8 run_status_from_data(const u8 *d)
{
    u16 be;
    u16 le;

    be = ((u16)d[2] << 8) | (u16)d[3];
    if (be <= 10u) {
        return (u8)be;
    }
    le = ((u16)d[3] << 8) | (u16)d[2];
    if (le <= 10u) {
        return (u8)le;
    }
    if (d[2] <= 10u) {
        return d[2];
    }
    if (d[3] <= 10u) {
        return d[3];
    }
    return d[2];
}

/* FC01/SFC01 fault: lv1 bit i => code i+1; lv2 bit0=>14 bit1=>15 */
static u8 map_fault_from_lower(u16 err_lv1, u8 err_lv2)
{
    u8  i;
    u8  code = 0u;

    if (err_lv1 == 0u && err_lv2 == 0u) {
        return 0u;
    }
    for (i = 0u; i < 13u; i++) {
        if (err_lv1 & (1u << i)) {
            code = (u8)(i + 1u);
            break;
        }
    }
    if (err_lv2 & (1u << 0)) {
        code = ERROR_BLDC_COMM;
    }
    if (err_lv2 & (1u << 1)) {
        code = ERROR_BLDC_OTEMP;
    }
    return code;
}

static void factory_test_ui_step(void)
{
    switch (s_factory_ui_phase % 3u) {
    case 0u:
        BEEP_SWITCH_ON_1_CNT();
        indecate.led_auto  = LED_ON;
        indecate.led_speed = LED_ON;
        indecate.led_updp  = LED_ON;
        indecate.led_downdp = LED_OFF;
        indecate.led_time    = LED_OFF;
        indecate.led_distance = LED_OFF;
        indecate.led_calorie = LED_OFF;
        set_seg_val(1234);
        break;
    case 1u:
        indecate.led_auto  = LED_OFF;
        indecate.led_speed = LED_OFF;
        indecate.led_updp  = LED_OFF;
        indecate.led_downdp = LED_ON;
        indecate.led_time    = LED_ON;
        indecate.led_distance = LED_ON;
        indecate.led_calorie = LED_ON;
        set_seg_val(5678);
        break;
    default:
        indecate.led_auto  = LED_OFF;
        indecate.led_speed = LED_OFF;
        indecate.led_updp  = LED_OFF;
        indecate.led_downdp = LED_OFF;
        indecate.led_time    = LED_OFF;
        indecate.led_distance = LED_OFF;
        indecate.led_calorie = LED_OFF;
        set_seg_val(8888);
        break;
    }
    s_factory_ui_phase++;
}

// UART pins
static void App_PortInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    DDL_ZERO_STRUCT(stcGpioCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPort3, GpioPin5, &stcGpioCfg);
    Gpio_SetAfMode(GpioPort3, GpioPin5, GpioAf1);

    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPort3, GpioPin6, &stcGpioCfg);
    Gpio_SetAfMode(GpioPort3, GpioPin6, GpioAf1);
}

static void _UartBaudCfg(void)
{
    uint16_t         timer = 0;
    stc_uart_baud_cfg_t stcBaud;
    stc_bt_cfg_t        stcBtCfg;

    DDL_ZERO_STRUCT(stcBaud);
    DDL_ZERO_STRUCT(stcBtCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1, TRUE);

    stcBaud.bDbaud  = 0u;
    stcBaud.u32Baud = 4800u; /* Match HA380 lower board; use 2400 if your drive uses 2400 */
    stcBaud.enMode  = UartMode1;
    stcBaud.u32Pclk = Sysctrl_GetPClkFreq();
    timer           = Uart_SetBaudRate(M0P_UART1, &stcBaud);

    stcBtCfg.enMD = BtMode2;
    stcBtCfg.enCT = BtTimer;
    Bt_Init(TIM1, &stcBtCfg);
    Bt_ARRSet(TIM1, timer);
    Bt_Cnt16Set(TIM1, timer);
    Bt_Run(TIM1);
}

static void App_UartInit(void)
{
    stc_uart_cfg_t stcCfg;

    _UartBaudCfg();

    stcCfg.enRunMode = UartMode1;
    Uart_Init(M0P_UART1, &stcCfg);

    Uart_EnableIrq(M0P_UART1, UartRxIrq);
    Uart_ClrStatus(M0P_UART1, UartRC);
    EnableNvic(UART1_IRQn, IrqLevel3, TRUE);
}

void uart0_init(void)
{
    App_PortInit();
    App_UartInit();
}

extern u8 uart_time_error;

void Uart1_IRQHandler(void)
{
    u8 dat;
    if (TRUE == Uart_GetStatus(M0P_UART1, UartRC)) {
        Uart_ClrStatus(M0P_UART1, UartRC);
        dat             = Uart_ReceiveData(M0P_UART1);
        uart_time_error = 0;
        Enter_queue(&rx_queue, dat);
    }
}

void uart_frame_init(void)
{
    Create_Queue(&rx_queue, &rx_buf[0], RX_BUFF_SIZE);

    s_rxst               = S_H1;
    s_factory_ui_phase   = 0u;
    s_lower_gear_max     = PROTO_GEAR_TENTH_FALLBACK_MAX;
    s_lower_gear_min     = PROTO_GEAR_TENTH_FALLBACK_MIN;
}

void UART_Send_Buf(u8 UARTPort, u8 *ptr, u8 len)
{
    (void)UARTPort;
    for (; len > 0u; len--) {
        Uart_SendDataPoll(M0P_UART1, *ptr);
        ptr++;
    }
}

void uart_frame_tx(u8 cmd, u8 dat)
{
    switch (cmd) {
    case CMD_ACK:
        AeroLink_Send(FC_HEARTBEAT, 0x00u, 0);
        break;
    case CMD_STATUS:
        if (dat == STATUS_MOTOR_STOP) {
            s_vrun_pending = 0u;
            s_vspd_pending = 0u;
            AeroLink_Send(FC_CONTROL, SFC_CTRL_STOP, 0);
            s_vstop_pending = 1u;
            s_vstop_hb      = 0u;
            s_vstop_retries = UART_TX_VERIFY_RETRIES;
        }
        break;
    case CMD_SPEED:
        speed_cmd_arm_and_tx(dat);
        break;
    case CMD_STOP_BURST:
        if (dat >= 10u) {
            uart_tx_verify_clear();
            AeroLink_Send(FC_CONTROL, SFC_CTRL_ESTOP, 0);
        } else {
            s_vrun_pending = 0u;
            s_vspd_pending = 0u;
            AeroLink_Send(FC_CONTROL, SFC_CTRL_STOP, 0);
            s_vstop_pending = 1u;
            s_vstop_hb      = 0u;
            s_vstop_retries = UART_TX_VERIFY_RETRIES;
        }
        break;
    default:
        break;
    }
}

void uart_frame_tx_2(u8 cmd, u8 dat0, u8 dat1)
{
    if (cmd == CMD_STATUS && dat0 == STATUS_MOTOR_RUN) {
        motor_run_arm_and_tx(speed_internal_from_disp_tenth(disp_tenth_from_cmd_dat(dat1)));
    } else if (cmd == CMD_STATUS && dat0 == STATUS_MOTOR_STOP) {
        s_vrun_pending = 0u;
        s_vspd_pending = 0u;
        AeroLink_Send(FC_CONTROL, SFC_CTRL_STOP, 0);
        s_vstop_pending = 1u;
        s_vstop_hb      = 0u;
        s_vstop_retries = UART_TX_VERIFY_RETRIES;
    } else {
        uart_frame_tx(cmd, dat0);
    }
}

void uart_frame_loop(void)
{
    u8 dat;
    u8 ret;

    ret = Denter_queue(&rx_queue, &dat);
    if (!ret) {
        return;
    }

    switch (s_rxst) {
    case S_H1:
        s_rxst = (dat == TREAD_HEAD1) ? S_H2 : S_H1;
        break;
    case S_H2:
        s_rxst = ((dat == TREAD_HEAD2) || (dat == TREAD_HEAD2_ALT)) ? S_CID : S_H1;
        break;
    case S_CID:
        if (dat == TREAD_CID_RECV) {
            s_rx.cid  = dat;
            s_cs_acc  = (u16)dat;
            s_rxst    = S_FC;
        } else {
            s_rxst = S_H1;
        }
        break;
    case S_FC:
        s_rx.fc  = dat;
        s_cs_acc += (u16)dat;
        s_rxst = S_SFC;
        break;
    case S_SFC:
        s_rx.sfc = dat;
        s_cs_acc += (u16)dat;
        s_didx    = 0u;
        s_rxst    = S_DATA;
        break;
    case S_DATA:
        s_rx.data[s_didx++] = dat;
        s_cs_acc += (u16)dat;
        if (s_didx >= 8u) {
            s_rxst = S_CSH;
        }
        break;
    case S_CSH:
        s_cs_exp = (u16)dat << 8;
        s_rxst   = S_CSL;
        break;
    case S_CSL:
        s_cs_exp |= (u16)dat;
        s_rxst = (s_cs_exp == s_cs_acc) ? S_T1 : S_H1;
        break;
    case S_T1:
        s_rxst = (dat == TREAD_TAIL1) ? S_T2 : S_H1;
        break;
    case S_T2:
        if (dat == TREAD_TAIL2) {
            cmd_proc();
        }
        s_rxst = S_H1;
        break;
    default:
        s_rxst = S_H1;
        break;
    }
}

void cmd_proc(void)
{
    u16 err_lv1;
    u8  err_lv2;
    u16 spd_word;
    u8  status_run;
    u8  ec;

    treadmills.ack = 1;

    if (s_rx.cid == TREAD_CID_RECV && s_rx.fc == 0xFFu && s_rx.sfc == 0xFFu) {
        factory_test_ui_step();
        return;
    }

    if (s_rx.fc == FC_HEARTBEAT && s_rx.sfc == 0x01u) {
        err_lv1 = ((u16)s_rx.data[0] << 8) | (u16)s_rx.data[1];
        err_lv2 = s_rx.data[2];
        ec      = map_fault_from_lower(err_lv1, err_lv2);
        if (ec != 0u) {
            if (treadmills.error_code == 0u) {
                treadmills.error_code = ec;
            }
            uart_tx_verify_clear();
            treadmills.status = STATUS_ERROR;
        }
        return;
    }

    if (s_rx.fc == FC_HEARTBEAT && s_rx.sfc == 0x00u) {
        u16 max_g = ((u16)s_rx.data[4] << 8) | (u16)s_rx.data[5];
        u16 min_g = ((u16)s_rx.data[6] << 8) | (u16)s_rx.data[7];

        if (max_g != 0u || min_g != 0u) {
            u16 hi = max_g;
            u16 lo = min_g;
            if (hi < lo) {
                u16 sw = hi;
                hi     = lo;
                lo     = sw;
            }
            if (hi >= lo && hi <= 500u && lo >= 1u) {
                s_lower_gear_max = hi;
                s_lower_gear_min = lo;
            }
        }

        spd_word   = ((u16)s_rx.data[0] << 8) | (u16)s_rx.data[1];
        status_run = run_status_from_data(s_rx.data);

        if (down_control == 0u) {
            down_control = 13u;
        }

        if (status_run == 3u) {
            uart_tx_verify_clear();
            treadmills.status = STATUS_ERROR;
            return;
        }

        if ((treadmills.status == STATUS_STOP_WAIT) ||
            (treadmills.status == STATUS_STOP_OVER)) {
            if (status_run == 7u) {
                treadmills_on_lower_poweron_after_stop();
            } else if (status_run == 10u) {
                treadmills_on_lower_standby_after_stop();
            } else if (treadmills_natural_stop_hold_off == 0u) {
                u16 disp = disp_tenth_from_protocol_gear(spd_word);
                u16 spd  = speed_internal_from_disp_tenth(disp);
                treadmills.param.speed     = spd;
                treadmills.display.index   = PARAM_SET_SPEED_ADC;
                treadmills.display.mode    = IDEL;
                treadmills_display(PARAM_SET_SPEED_ADC);
            }
        }

        uart_frame_hb_tx_verify(spd_word, status_run);
        return;
    }

}
