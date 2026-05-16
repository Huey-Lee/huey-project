/*
 * Function: 上控 UART 帧收发、状态机解析、命令分发与参数写入。
 * Method:   rx_fifo 收字节；uart_frame_loop 组帧校验后 cmd_proc；Uart1_IRQHandler 写入 ringfifo；应答经 UART_Send_Buf。
 * Name:     Huey
 * Date:     May 16, 2026 18:00
 */

#include "uart_frame.h"
#include "motor.h"
#include "uart.h"
#include "user_usart.h"
#include "user_timer.h"
#include "motor_drive.h"

ee_param_t  rx_setparam;

static u16 uart_kmh_payload_to_speed_scale(u8 kbuf)
{
    if (kbuf < 10u)
        kbuf = 10u;
    if (kbuf > 100u)
        kbuf = 100u;
    return (u16)kbuf * 10u;
}

static void uart_clamp_voltage_max_payload_to_grid(u8 *v)
{
    u8 cap;

    if (motor_grid_profile_known == 0u)
        return;
    cap = motor_grid_env_220 ? (u8)MAX_RUN_V_220V : (u8)MAX_RUN_V_110V;
    if (*v > cap)
        *v = cap;
}

void uart_sync_rx_voltage_max_from_motor(void)
{
    rx_setparam.voltage_max = motor.adjust_max_voltage;
}

ringfifo_t rx_fifo;
u8         rx_buf[RX_BUFF_SIZE];
uart_frame_t uart_frame;
uart_frame_t uart_tx_frame;
extern u8 relay_wait_en;

void uart_frame_init(void)
{
    ringfifo_init(&rx_fifo, &rx_buf[0], RX_BUFF_SIZE);
    rx_setparam.treadmills_speed_max = 100u;
    rx_setparam.voltage_max          = 90u;
    rx_setparam.voltage_min          = 20u;

    uart_frame.sof    = START_OF_FRAME;
    uart_frame.eof    = END_OF_FRAME;
    uart_tx_frame.sof = START_OF_FRAME;
    uart_tx_frame.eof = END_OF_FRAME;
}

void UART_Send_Buf(uint8_t UARTPort, uint8_t *ptr, uint8_t len)
{
    for (uint8_t l = 0; l < len; l++) {
        Uart_SendDataPoll(M0P_UART1, ptr[l]);
    }
}

void uart_frame_tx(u8 cmd, u8 dat)
{
    u8 xorval = 0;
    uart_tx_frame.cmd    = cmd;    xorval ^= cmd;
    uart_tx_frame.len    = 1;     xorval ^= 1;
    uart_tx_frame.buf[0] = dat;   xorval ^= dat;
    uart_tx_frame.buf[1] = xorval;
    uart_tx_frame.crch   = 0;
    uart_tx_frame.crcl   = END_OF_FRAME;
    UART_Send_Buf(UART0, (u8*)&uart_tx_frame, uart_tx_frame.len + 6);
}

void uart_frame_tx_2(u8 cmd, u8 dat0, u8 dat1)
{
    u8 xorval = 0;
    uart_tx_frame.cmd    = cmd;    xorval ^= cmd;
    uart_tx_frame.len    = 2;     xorval ^= 2;
    uart_tx_frame.buf[0] = dat0;  xorval ^= dat0;
    uart_tx_frame.buf[1] = dat1;  xorval ^= dat1;
    uart_tx_frame.crch   = xorval;
    uart_tx_frame.crcl   = 0;
    uart_tx_frame.eof    = END_OF_FRAME;
    UART_Send_Buf(UART0, (u8*)&uart_tx_frame, uart_tx_frame.len + 6);
}

/* 约 500 ms 轮询一次：若有待应答事件则主动上报错误码或 ACK */
void communication_checkout(void)
{
    if (ctr.ack == 1) {
        if (ctr.error_code) {
            uart_frame_tx(CMD_ERROR, ctr.error_code);  /* 上报故障码 */
        } else {
            uart_frame_tx(CMD_ACK, 1);                 /* 正常应答 */
        }
        ctr.ack = 0;
    }
}

extern u8 uart_time_error;

/* 接收状态机：每次主循环从队列取出一个字节进行解析
 * 帧格式：[0x7F][CMD][LEN][DATA×LEN][XOR/校验][0x00][0x7E]
 *   sm=0：等待帧头 0x7F
 *   sm=1：收 CMD
 *   sm=2：收 LEN（最大 2 字节 payload）
 *   sm=3：收 payload + 2 字节校验
 *   sm=4：收帧尾 0x7E，校验通过后调用 cmd_proc() */
void uart_frame_loop(void)
{
    static u8  sm = 0, len = 0;
    static u8 *ptr;
    u8 dat;

    if (ringfifo_get(&rx_fifo, &dat, 1u) != 1)
        return;  /* 无数据 */

    /* 任意状态收到帧头则重置状态机
     * 【Bug修复】不在此处清超时计数：单个 0x7F 可能是线路噪声，
     * 只有在帧校验完整通过后才清零，防止噪声阻止 E01 报警 */
    if (dat == uart_frame.sof) { sm = 1; return; }

    switch (sm) {
    case 1:  /* 接收命令字 */
        uart_frame.cmd = dat;
        sm = 2;
        break;
    case 2:  /* 接收数据长度（超过 2 视为非法，丢弃） */
        uart_frame.len = dat;
        if (uart_frame.len > 2) { sm = 0; return; }
        len = 0;
        ptr = &uart_frame.buf[0];
        sm = 3;
        break;
    case 3:  /* 接收 payload + 2 字节校验（共 len+2 字节存入 buf） */
        ptr[len++] = dat;
        if (len >= (uart_frame.len + 2)) sm = 4;
        break;
    case 4:  /* 接收帧尾，校验后分发命令 */
        if (dat == uart_frame.eof) {
            uint8_t i, xsum, m7, expect, s2;
            ptr = &uart_frame.cmd;
            dat = 0;
            /* XOR 校验：CMD ^ LEN ^ DATA... */
            for (len = 0; len < (uart_frame.len + 2); len++) dat ^= ptr[len];
            xsum = dat;
            /* 备用校验：(CMD + LEN + DATA...) % 7 */
            s2 = (uint8_t)(ptr[0] + ptr[1]);
            for (i = 0; i < uart_frame.len; i++) s2 = (uint8_t)(s2 + ptr[2 + i]);
            m7     = (uint8_t)(s2 % 7u);
            expect = ptr[uart_frame.len + 2u];
            /* XOR 或 %7 任一通过即认为帧有效，通过后才清通信超时计数 */
            if (xsum == expect || m7 == expect) {
                uart_time_error = 0;  /* 帧有效：清超时（从噪声处移到此处） */
                cmd_proc();
            }
        }
        sm = 0;
        break;
    default:
        break;
    }
}

extern u8 speed_step;

/* 将 Kcomp 值限制在 0~10，防止上控下发超范围导致过补偿 */
static u8 ClampKiv(u8 raw)
{
    return (raw > 10u) ? 10u : raw;
}

/* 命令分发处理：帧校验通过后调用 */
void cmd_proc(void)
{
    u8 kiv_val = 0;

    switch (uart_frame.cmd) {

    /* 启动/停止指令：buf[0]=0 停机，buf[0]=1 启动（buf[1] 为初始速度）
     * LEN=0 时 buf[] 仅占位校验字节，不能与有效帧混用（校验通过后仍可能出现异常短帧）。 */
    case CMD_STATUS:
        if (uart_frame.len < 1u)
            break;
        if (motor.en != uart_frame.buf[0]) {
            if (uart_frame.buf[0] == 0) {
                ctr.ack               = 1;
                motor.en              = 0;
                motor.set_speed_scale = MT_STOP_SPEED;
                ctr.status            = STATUS_STOP;
            } else if (uart_frame.buf[0] == 1) {
                ctr.ack = 1;
                motor.en = 1;
                ctr.status = STATUS_RUN;
                /* 【Bug修复】LEN=1 时 buf[1] 无效（上控未发初速），使用当前设定速度或最低速
                 * LEN=2 时 buf[1] 才是有效的初速值 */
                if (uart_frame.len >= 2u) {
                    u16 ss = uart_kmh_payload_to_speed_scale(uart_frame.buf[1]);

                    if (ss > motor.adjust_speed_max)
                        ss = motor.adjust_speed_max;
                    motor.set_speed_scale = ss;
                } else {
                    /* 仅有启停字节，速度维持上次设定（或最低速兜底） */
                    if (motor.set_speed_scale < (u16)(motor.sta_speed_scale))
                        motor.set_speed_scale = motor.sta_speed_scale;
                }
                motor.adjust_sta_voltage = motor.start_valtage;
                motor.cur_speed_scale    = motor.sta_speed_scale;
            }
        }
        break;

    /* PID 参数指令（开环模式不使用，静默接受以保持协议兼容） */
    case CMD_PID_P:
    case CMD_PID_I:
    case CMD_PID_D:
        break;

    /* 起步力度等级（1~10）：等级越高起步电压越高、爬坡越快 */
    case CMD_STA_LEVEL:
        if (uart_frame.len < 1u)
            break;
        if (uart_frame.buf[0] > 10) uart_frame.buf[0] = 10;
        switch (uart_frame.buf[0]) {
            /* start_valtage：起步第一拍目标电压（ISR 与同路径 adjust_now_voltage 同量纲）
             * V 粗算便于对照显示器：≈ ×0.126（与 cut_adc→V 同一种比例）
             * start_add_val：每步爬坡加的「计数」，单位同 start_valtage，不是 V */
            case 0:
            case 1:  motor.start_valtage = 40;  motor.start_add_val = 3;  break;
            case 2:  motor.start_valtage = 80;  motor.start_add_val = 5;  break;
            case 3:  motor.start_valtage = 100; motor.start_add_val = 5;  break;
            case 4:  motor.start_valtage = 120; motor.start_add_val = 5;  break;
            case 5:  motor.start_valtage = 140; motor.start_add_val = 6;  break;
            /* 6~10 压缩：峰值封顶约 20 V（159 ADC），仍保持档位越高越强 */
            case 6:  motor.start_valtage = 145; motor.start_add_val = 6;  break;
            case 7:  motor.start_valtage = 149; motor.start_add_val = 7;  break;
            case 8:  motor.start_valtage = 152; motor.start_add_val = 8;  break;
            case 9:  motor.start_valtage = 156; motor.start_add_val = 9;  break;
            case 10: motor.start_valtage = 159; motor.start_add_val = 9;  break;
            default: motor.start_valtage = 80;  motor.start_add_val = 5;  break;
        }
        break;

    case CMD_STATUS_INQUIRE:  /* 状态查询，暂不处理（上控主动查询） */
        break;

    /* 判停目标电压（2~25 V）：越小皮带停得越彻底，越大停得越早 */
    case CMD_END_LEVEL:
        if (uart_frame.len < 1u)
            break;
        if (uart_frame.buf[0] > 25) uart_frame.buf[0] = 25;
        if (uart_frame.buf[0] < 2)  uart_frame.buf[0] = 2;
        motor.end_speed_scale = uart_frame.buf[0];
        break;

    /* 加速步长（1~10）：值越大速度变化越快 */
    case CMD_ACCELERATED_LEVEL:
        if (uart_frame.len < 1u)
            break;
        if (uart_frame.buf[0] > 10) uart_frame.buf[0] = 10;
        if (uart_frame.buf[0] < 1)  uart_frame.buf[0] = 1;
        motor.accelerated = uart_frame.buf[0];
        break;

    /* 减速步长（1~10） */
    case CMD_DECELERATION_LEVEL:
        if (uart_frame.len < 1u)
            break;
        if (uart_frame.buf[0] > 10) uart_frame.buf[0] = 10;
        if (uart_frame.buf[0] < 1)  uart_frame.buf[0] = 1;
        motor.deceleration = uart_frame.buf[0];
        break;

    /* 速度指令：buf[0] 载荷 10~100（1~10 km/h）→ ×10 转为内部斜坡 100~1000 */
    case CMD_SPEED:
        if (uart_frame.len < 1u)
            break;
        ctr.ack = 1;
        if (uart_frame.buf[0] < 10u)
            uart_frame.buf[0] = 10u;
        if (uart_frame.buf[0] > rx_setparam.treadmills_speed_max)
            uart_frame.buf[0] = rx_setparam.treadmills_speed_max;
        motor.set_speed_scale = uart_kmh_payload_to_speed_scale(uart_frame.buf[0]);
        break;

    case CMD_ACK:
        ctr.ack = 1;
        break;

    case CMD_RESET:
        break;

    case CMD_WRITE_PARAM:
        break;

    case CMD_READ_PARAM:
        break;

    /* 工装/质检：瞬时拉高加减速步长（与量产 CMD_ACCELERATED_LEVEL 1~10 不同量纲）。
     * 与老 H1/C03 产线用法一致勿删（C03 曾写 80；现按需求固定 50/50）。*/
    case CMD_TEST:
        motor.accelerated  = 50u;
        motor.deceleration = 50u;
        break;

    /* 急停：立即断电（E06），对应安全开关或紧急按钮 */
    case CMD_STATUS_URGENT_STOP:
        motor_hard_fault_set(ERROR_06);
        break;

    /* 最高速度上限（载荷 10~100 ↔ 内部 100~1000），同时刷新电压斜率 */
    case CMD_TREADMILLS_SPEED_MAX:
        if (uart_frame.len < 1u)
            break;
        if (uart_frame.buf[0] >= 10u && uart_frame.buf[0] <= 100u) {
            rx_setparam.treadmills_speed_max = uart_frame.buf[0];
            motor.adjust_speed_max           =
                uart_kmh_payload_to_speed_scale(rx_setparam.treadmills_speed_max);
            motor_recalc_voltage_param_from_cmd();
        }
        break;

    /* 最高运行电压（ADC），同时更新斜率 */
    case CMD_VOLTAGE_MAX:
        if (uart_frame.len < 1u)
            break;
        {
            u8 v = uart_frame.buf[0];

            uart_clamp_voltage_max_payload_to_grid(&v);
            if (v > rx_setparam.voltage_min) { /* 必须高于最低电压 */
                rx_setparam.voltage_max  = v;
                motor.adjust_max_voltage = rx_setparam.voltage_max;
                motor_recalc_voltage_param_from_cmd();
                motor_drive_refresh_over_voltage_from_vmax();
            }
        }
        break;

    /* 最低运行电压（ADC），同时更新斜率 */
    case CMD_VOLTAGE_MIN:
        if (uart_frame.len < 1u)
            break;
        if (uart_frame.buf[0] < rx_setparam.voltage_max) {  /* 必须低于最高电压 */
            rx_setparam.voltage_min  = uart_frame.buf[0];
            motor.adjust_min_voltage = rx_setparam.voltage_min;
            motor_recalc_voltage_param_from_cmd();
        }
        break;

    case CMD_OVER_CURRENT_MAX: {
        if (uart_frame.len < 1u)
            break;
        u32 raw_val = (u32)uart_frame.buf[0] * 10u;

        if (raw_val < (u32)MOTOR_OVER_CURRENT_ADC_FLOOR)
            raw_val = (u32)MOTOR_OVER_CURRENT_ADC_FLOOR;
        if (raw_val > (u32)MOTOR_OVER_CURRENT_ADC_CEIL)
            raw_val = (u32)MOTOR_OVER_CURRENT_ADC_CEIL;
        motor.adjust_ove_curren = (u16)raw_val;
        oc_lim_full_mirror        = motor.adjust_ove_curren;
        break;
    }

    /* CMD_OVER_VALTAGE_MAX：不解析载荷；over_voltage 仅由 vmax（CMD_VOLTAGE_MAX → adjust_max_voltage）推导，见 motor_drive_refresh_over_voltage_from_vmax */
    case CMD_OVER_VALTAGE_MAX:
        motor_drive_refresh_over_voltage_from_vmax();
        break;

    /* 各速度档位的 IR 补偿系数 Kcomp（0~10），由上控按 km/h 逐档下发
     * ClampKiv 防止超范围；写入 speed_param[] 供 motor_drive.c 运行时查表 */
    case CMD_KIV_1KM:
        if (uart_frame.len < 1u)
            break;
        kiv_val = ClampKiv(uart_frame.buf[0]);
        rx_setparam.kiv[KIV_1KM_INDEX] = kiv_val;
        speed_param[0].kiv = kiv_val;
        speed_param[1].kiv = kiv_val;  /* 1 km/h 和 0 档共用同一 Kcomp */
        break;
    case CMD_KIV_2KM:
        if (uart_frame.len < 1u)
            break;
        kiv_val = ClampKiv(uart_frame.buf[0]);
        rx_setparam.kiv[KIV_2KM_INDEX] = kiv_val;
        speed_param[2].kiv = kiv_val;
        break;
    case CMD_KIV_3KM:
        if (uart_frame.len < 1u)
            break;
        kiv_val = ClampKiv(uart_frame.buf[0]);
        rx_setparam.kiv[KIV_3KM_INDEX] = kiv_val;
        speed_param[3].kiv = kiv_val;
        break;
    case CMD_KIV_4KM:
        if (uart_frame.len < 1u)
            break;
        kiv_val = ClampKiv(uart_frame.buf[0]);
        rx_setparam.kiv[KIV_4KM_INDEX] = kiv_val;
        speed_param[4].kiv = kiv_val;
        break;
    case CMD_KIV_5KM:
        if (uart_frame.len < 1u)
            break;
        kiv_val = ClampKiv(uart_frame.buf[0]);
        rx_setparam.kiv[KIV_5KM_INDEX] = kiv_val;
        speed_param[5].kiv = kiv_val;
        break;
    case CMD_KIV_6KM:
        if (uart_frame.len < 1u)
            break;
        kiv_val = ClampKiv(uart_frame.buf[0]);
        rx_setparam.kiv[KIV_6KM_INDEX] = kiv_val;
        speed_param[6].kiv = kiv_val;
        break;
    case CMD_KIV_7KM:
        if (uart_frame.len < 1u)
            break;
        kiv_val = ClampKiv(uart_frame.buf[0]);
        rx_setparam.kiv[KIV_7KM_INDEX] = kiv_val;
        speed_param[7].kiv = kiv_val;
        break;
    case CMD_KIV_8KM:
        if (uart_frame.len < 1u)
            break;
        kiv_val = ClampKiv(uart_frame.buf[0]);
        rx_setparam.kiv[KIV_8KM_INDEX] = kiv_val;
        speed_param[8].kiv = kiv_val;
        break;
    case CMD_KIV_9KM:
        if (uart_frame.len < 1u)
            break;
        kiv_val = ClampKiv(uart_frame.buf[0]);
        rx_setparam.kiv[KIV_9KM_INDEX] = kiv_val;
        speed_param[9].kiv = kiv_val;
        break;
    case CMD_KIV_10KM:
        if (uart_frame.len < 1u)
            break;
        kiv_val = ClampKiv(uart_frame.buf[0]);
        rx_setparam.kiv[KIV_10KM_INDEX] = kiv_val;
        speed_param[10].kiv = kiv_val;
        break;
    case CMD_KIV_11KM:
        if (uart_frame.len < 1u)
            break;
        kiv_val = ClampKiv(uart_frame.buf[0]);
        rx_setparam.kiv[KIV_11KM_INDEX] = kiv_val;
        speed_param[11].kiv = kiv_val;
        break;
    case CMD_KIV_12KM:
        if (uart_frame.len < 1u)
            break;
        kiv_val = ClampKiv(uart_frame.buf[0]);
        rx_setparam.kiv[KIV_12KM_INDEX] = kiv_val;
        speed_param[12].kiv = kiv_val;
        break;

    default:
        break;
    }
}

/* UART1 接收中断：每收到一个字节就压入环形队列，由 uart_frame_loop() 在主循环中解析 */
void Uart1_IRQHandler(void)
{
    uint8_t reg;
    if (TRUE == Uart_GetStatus(M0P_UART1, UartRC)) {
        Uart_ClrStatus(M0P_UART1, UartRC);
        reg = Uart_ReceiveData(M0P_UART1);
        ringfifo_putin(&rx_fifo, &reg, 1u);
    }
}
