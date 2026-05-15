/* uart_frame.h - display protocol, frame struct, RX buffer. */
#ifndef USER_UART_FRAME_H_
#define USER_UART_FRAME_H_
#include "queue.h"

#define UART0 0

#define RX_BUFF_SIZE   (135)
#define START_OF_FRAME  (0X7F)
#define END_OF_FRAME    (0X7E)


/* KIV table index, one per km (display set) */
#define KIV_1KM_INDEX     0
#define KIV_2KM_INDEX     1
#define KIV_3KM_INDEX     2
#define KIV_4KM_INDEX     3
#define KIV_5KM_INDEX     4
#define KIV_6KM_INDEX     5
#define KIV_7KM_INDEX     6
#define KIV_8KM_INDEX     7
#define KIV_9KM_INDEX     8
#define KIV_10KM_INDEX     9
#define KIV_11KM_INDEX     10
#define KIV_12KM_INDEX     11
#define KIV_13KM_INDEX     12
#define KIV_14KM_INDEX     13


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
	CMD_STATUS_INQUIRE,      /* query */
	CMD_STATUS_URGENT_STOP,  /* e-stop from display */
  
  CMD_TREADMILLS_SPEED_MAX,   /* 0x0C */
  CMD_VOLTAGE_MAX,            /* 0x0D */
  CMD_VOLTAGE_MIN,            /* 0x0E */
  CMD_OVER_CURRENT_MAX,       /* 0x0F */
  CMD_KIV_1KM,                /* 0x10 */
  CMD_KIV_2KM,                /* 0x11 */
  CMD_KIV_3KM,                /* 0x12 */
  CMD_KIV_4KM,                /* 0x13 */
  CMD_KIV_5KM,                /* 0x14 */
  CMD_KIV_6KM,                /* 0x15 */
  CMD_KIV_7KM,                /* 0x16 */
  CMD_KIV_8KM,                /* 0x17 */
  CMD_KIV_9KM,                /* 0x18 */
  CMD_KIV_10KM,               /* 0x19 */
  CMD_KIV_11KM,               /* 0x1A */
  CMD_KIV_12KM,               /* 0x1B */
	
	CMD_STA_LEVEL,
	CMD_END_LEVEL,
	CMD_Accelerated_LEVEL,
	CMD_deceleration_LEVEL,
	
	CMD_PID_P,
	CMD_PID_I,
	CMD_PID_D,
	CMD_OVER_VALTAGE_MAX,
	CMD_KIV_13KM = 0x30,
	CMD_KIV_14KM = 0x31,
}cmd_e;


#define DATA_BUF_SIZE  (2)
#define KIV_NUM        (14)

typedef struct _t_uart_frame
{
	u8 sof;
	u8 cmd;
	u8 len;
	u8 buf[DATA_BUF_SIZE];
	u8 crch;   /* checksum high */
	u8 crcl;   /* unused / low (protocol) */
	u8 eof;
}uart_frame_t;


typedef struct _ee_param_t
{
  u8 sof;
  u8 treadmills_speed_max;       /* 0.1 unit max */
  u8 voltage_max;
  u8 voltage_min;
  u8 over_current_max_k;
  u8 kiv[KIV_NUM];               /* from display, per Km row */
}ee_param_t;



extern T_QUEUE rx_queue;
extern uart_frame_t uart_frame;
extern uart_frame_t uart_tx_frame;

extern void uart_frame_init(void);
extern void uart_frame_loop(void);
extern void uart_frame_tx(u8 cmd,u8 dat);
extern void uart_frame_tx_2(u8 cmd,u8 dat0,u8 dat1);
extern void communication_checkout(void);
extern void uart0_init(void);
extern void cmd_proc(void);
void UART_Send_Buf(uint8_t UARTPort, uint8_t *ptr,uint8_t len);
#endif /* USER_UART_FRAME_H_ */
