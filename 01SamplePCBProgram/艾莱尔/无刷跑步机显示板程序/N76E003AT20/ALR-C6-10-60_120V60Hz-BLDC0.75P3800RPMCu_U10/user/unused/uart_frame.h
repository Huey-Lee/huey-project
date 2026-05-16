/*
 * uart_frame.h
 *
 *  Created on: 2019Äê12ÔÂ11ÈÕ
 *      Author: Administrator
 */

#ifndef USER_UART_FRAME_H_
#define USER_UART_FRAME_H_
#include "common.h"
#include "queue.h"

#define RX_BUFF_SIZE (300)
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
}cmd_e;



#define DATA_BUF_SIZE  (1)
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


extern T_QUEUE rx_queue;
extern uart_frame_t uart_frame;
extern uart_frame_t uart_tx_frame;
extern void uart_frame_init(void);
extern void uart_frame_loop(void);
extern void uart_frame_tx(u8 cmd,u8 dat);
extern void uart0_init(void);


#endif /* USER_UART_FRAME_H_ */
