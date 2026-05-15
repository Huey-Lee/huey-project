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

//蓝牙模块通讯
typedef struct _t_uart0_frame
{
	u8 sof;
	u8 cmd;
  u8 subcmd;
	u8 buf[20];
}uart0_frame_t;

extern T_QUEUE UART1_rx_queue;
extern T_QUEUE UART0_rx_queue;
extern uart_frame_t uart_frame;
extern uart_frame_t uart_tx_frame;
extern uart0_frame_t uart0_frame;
extern uart0_frame_t uart0_tx_frame;


extern void uart_frame_init(void);
extern void uart1_frame_loop(void);
extern void uart1_frame_tx(u8 cmd,u8 dat);
extern void uart1_frame_tx_2(u8 cmd,u8 dat0,u8 dat1);

extern void uart0_frame_loop(void);
extern void uart0_frame_tx_0(u8 cmd,u8 subcmd);
extern void uart0_frame_tx_1(u8 cmd,u8 subcmd,u8 dat);
extern void uart0_frame_tx_2(u8 cmd,u8 subcmd,u8 dat0,u8 dat1);
extern void uart0_frame_tx_3(u8 cmd,u8 subcmd,u8 dat0,u8 dat1,u8 dat2); 
extern void uart0_frame_tx_4(u8 cmd,u8 subcmd,u8 dat0,u8 dat1,u8 dat2,u8 dat3);
extern void uart0_frame_tx_12(u8 cmd,u8 subcmd,u8 dat0,u8 dat1,u8 dat2,u8 dat3,u8 dat4,u8 dat5,u8 dat6,u8 dat7,u8 dat8,u8 dat9,u8 dat10,u8 dat11);

#endif /* USER_UART_FRAME_H_ */
