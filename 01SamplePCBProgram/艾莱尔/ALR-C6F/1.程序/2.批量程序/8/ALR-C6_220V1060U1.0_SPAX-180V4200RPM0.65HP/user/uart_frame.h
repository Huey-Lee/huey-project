/*
 * uart_frame.h
 *
 *  Created on: 2019年12月11日
 *      Author: Administrator
 */

#ifndef USER_UART_FRAME_H_
#define USER_UART_FRAME_H_
#include "common.h"
#include "queue.h"

#define UART1_RX_BUFF_SIZE (150)  
#define UART0_RX_BUFF_SIZE (150)

#define MAX_DATA_LEN  (20)
typedef struct _t_ProtocolFrame {
    u8 sof; u8 cmd; u8 subcmd; u8 len;
    u8 buf[MAX_DATA_LEN]; u8 fcs; u8 end;
} ProtocolFrame_t;

extern T_QUEUE UART1_rx_queue;
extern T_QUEUE UART0_rx_queue;
extern ProtocolFrame_t frame;



/* ============================================================
 * AeroLink 协议常量 (AA BB CID FC SFC DATA[8] CS_H CS_L EE FF)
 * ============================================================ */
#define AERO_HEAD1       0xAA
#define AERO_HEAD2       0xBB
#define AERO_TAIL1       0xEE
#define AERO_TAIL2       0xFF
#define AERO_CID_SEND    0x0A   /* 上控 → 下控 */
#define AERO_CID_RECV    0x09   /* 下控 → 上控 */

#define AERO_FC_HEARTBEAT  0x01 /* heartbeat FC */
#define AERO_FC_STARTSTOP  0x02 /* start/stop FC */
#define AERO_SFC_HB        0x00 /* heartbeat SFC */
#define AERO_SFC_START     0x00 /* start SFC */
#define AERO_SFC_STOP      0x01 /* stop SFC */
#define AERO_FC_GEAR       0x03 /* 速度档位 SFC=0x00 */

/* 下控运行状态码 (运行位点帧 data[2]) */
#define LOWER_STATUS_IDLE     7  /* 待机 */
#define LOWER_STATUS_RUNNING  9  /* 电机运行 */
#define LOWER_STATUS_STOPPING 10 /* 电机停止中 */
/* 下控反馈全局变量（由接收回调更新） */
extern u16 g_fbk_speed;      /* 下控反馈实时速度 (与上控速度单位一致) */
extern u8  g_lower_status;   /* 下控运行状态 */

/* AeroLink 发送接口 */
extern void aerolink_send_heartbeat(void);
extern void aerolink_send_start(u16 speed);   /* speed = treadmills.param.speed */
extern void aerolink_send_stop(void);
extern void aerolink_send_speed(u16 speed);   /* speed = treadmills.param.speed */

/* 控制循环（每 100ms 调用：心跳 + 启停命令可靠重发） */
extern void aerolink_ctrl_loop(void);

extern void uart_frame_init(void);
extern void uart1_frame_loop(void);
/* uart1_frame_tx removed */


extern void uart0_frame_loop(void);
extern void uart0_send_frame(u8 sof, u8 cmd, u8 subcmd, u8 len, u8 *buf, u8 end);
extern void bt_power_send(void);

extern void send_motion_data_by_status(void);
extern void send_motion_data(u8 status_code);
extern void bt_panel_send_cmd(u8 subcmd, u8 len, u8* buf);
extern void bt_panel_send_speed(void);
#endif /* USER_UART_FRAME_H_ */
