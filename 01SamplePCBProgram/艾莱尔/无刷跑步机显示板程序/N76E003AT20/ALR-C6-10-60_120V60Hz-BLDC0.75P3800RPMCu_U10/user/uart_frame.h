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

#define START_OF_FRAME  (0X7F)
#define END_OF_FRAME    (0X7E)

#define STATUS_MOTOR_RUN (1)
#define STATUS_MOTOR_STOP (0)


typedef enum _cmd_e
{
	CMD_STATUS=1,
	CMD_SPEED,
	CMD_ACK,
	CMD_ERROR,
	CMD_RESET,
	CMD_STOP_OVER,
	CMD_WRITE_PARAM,
	CMD_READ_PARAM,
	CMD_TEST,
	CMD_STATUS_INQUIRE,
	CMD_STATUS_URGENT_STOP,
  
  CMD_TREADMILLS_SPEED_MAX,   //12 跑步机最大速度 0x0C
  CMD_VOLTAGE_MAX,            //13 最大电压  0x0D
  CMD_VOLTAGE_MIN,            //14 最小电压   0x0E
  CMD_OVER_CURRENT_MAX,       //15 最大电流值  0x0F
  CMD_KIV_1KM,                //16  KIV参数   0x10 
  CMD_KIV_2KM,                //17  KIV参数   0x11 
  CMD_KIV_3KM,                //18  KIV参数   0x12 
  CMD_KIV_4KM,                //19  KIV参数   0x13 
  CMD_KIV_5KM,                //20  KIV参数   0x14 
  CMD_KIV_6KM,                //21  KIV参数   0x15 
  CMD_KIV_7KM,                //22  KIV参数   0x16 
  CMD_KIV_8KM,                //23  KIV参数   0x17 
  CMD_KIV_9KM,                //24  KIV参数   0x18 
  CMD_KIV_10KM,               //25  KIV参数   0x19 
  CMD_KIV_11KM,               //26  KIV参数   0x1A 
  CMD_KIV_12KM,               //27  KIV参数   0x1B 	
//	CMD_STATUS_HEART,            //28
  
  CMD_STA_LEVEL,
  CMD_END_LEVEL,
  CMD_Accelerated_LEVEL,
  CMD_deceleration_LEVEL,
  CMD_PID_P,
  CMD_PID_I,
  CMD_PID_D,
  
   
  CMD_OVER_VALTAGE_MAX
}cmd_e;


#define DATA_BUF_SIZE  (2)
typedef struct _t_uart_frame
{
	u8 sof;
	u8 cmd;
	u8 len;
	u8 buf[DATA_BUF_SIZE];
	u8 crch;
	u8 crcl;
	u8 eof;
	u8 is_ok;
}uart_frame_t;

#define MAX_DATA_LEN  (20)
typedef struct _t_ProtocolFrame
{
    u8 sof;
    u8 cmd;
    u8 subcmd;
    u8 len;
    u8 buf[MAX_DATA_LEN];
		u8 fcs;
    u8 end;
} ProtocolFrame_t;

extern T_QUEUE UART1_rx_queue;
extern T_QUEUE UART0_rx_queue;
extern uart_frame_t uart_frame;
extern uart_frame_t uart_tx_frame;

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

#define AERO_FC_HEARTBEAT  0x01 /* 心跳 */
#define AERO_FC_CONTROL    0x02 /* 启停: sfc=0x00启动, sfc=0x01停止 */
#define AERO_FC_GEAR       0x03 /* 速度档位 */

/* 下控运行状态码 */
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
extern void uart1_frame_tx(u8 cmd,u8 dat);
extern void uart1_frame_tx_2(u8 cmd,u8 dat0,u8 dat1);

extern void uart0_frame_loop(void);
extern void uart0_send_frame(u8 sof, u8 cmd, u8 subcmd, u8 len, u8 *buf, u8 end);
extern void bt_power_send(void);

extern void send_motion_data_by_status(void);
extern void send_motion_data(u8 status_code);
extern void bt_panel_send_cmd(u8 subcmd, u8 len, u8* buf);
extern void bt_panel_send_speed(void);
#endif /* USER_UART_FRAME_H_ */
