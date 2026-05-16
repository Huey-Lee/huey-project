/*
 * uart_frame.h - UART1: 17-byte BLDC (same as C60T reference); UART0: module/BT protocol
 */

#ifndef __UART_FRAME_H__
#define __UART_FRAME_H__

#include "common.h"
#include "queue.h"

#define RX_BUFF_SIZE (300)
#define UART0_RX_BUFF_SIZE (150)
#define TREAD_HEAD1      0xAAu
#define TREAD_HEAD2      0xBBu
#define TREAD_HEAD2_ALT  0xB8u
#define TREAD_TAIL1      0xEEu
#define TREAD_TAIL2      0xFFu
#define TREAD_CID_SEND   0x0Au
#define TREAD_CID_RECV   0x09u
#define FC_HEARTBEAT     0x01u
#define FC_CONTROL       0x02u
#define FC_GEAR          0x03u
#define SFC_CTRL_START   0x00u
#define SFC_CTRL_STOP    0x01u
#define SFC_CTRL_ESTOP   0x02u

#ifndef UART_SAFETY_LOCK_USE_ESTOP
#define UART_SAFETY_LOCK_USE_ESTOP  0
#endif

#define STATUS_MOTOR_RUN   (1u)
#define STATUS_MOTOR_STOP  (0u)

typedef enum {
    CMD_STATUS      = 1u,
    CMD_SPEED       = 2u,
    CMD_ACK         = 3u,
    CMD_STOP_BURST  = 31u,
} uart_cmd_e;

void uart_frame_init(void);
void uart_frame_loop(void);
void uart_frame_tx(u8 cmd, u8 dat);
void uart_frame_tx_2(u8 cmd, u8 dat0, u8 dat1);
u16 uart_prog_kmh_to_internal(u8 kmh_whole);
void uart_frame_motor_run_internal_spd(u16 internal_spd);
void uart_frame_reset_run_verify_state(void);
void uart_frame_safety_lock_stop_sequence(void);

/* UART0：0.6~6.2km/h（内部 100~996）与协议速度字节 1~10 线性对应；0=无速度/停机显示 */
u8  uart0_internal_speed_to_module_km(u16 internal_spd);
u16 uart0_module_km_to_internal_speed(u8 module_k);

void UartInit(void);

#define UART0 0
#define UART1 1

void UART_Send_Buf(u8 UARTPort, u8 *ptr, u8 len);

typedef struct _t_uart0_frame {
    u8 sof;
    u8 cmd;
    u8 subcmd;
    u8 buf[20];
} uart0_frame_t;

extern T_QUEUE UART0_rx_queue;
extern uart0_frame_t uart0_frame;
extern uart0_frame_t uart0_tx_frame;

void uart0_frame_loop(void);
void uart0_frame_tx_0(u8 cmd, u8 subcmd);
void uart0_frame_tx_1(u8 cmd, u8 subcmd, u8 dat);
void uart0_frame_tx_2(u8 cmd, u8 subcmd, u8 dat0, u8 dat1);
void uart0_frame_tx_3(u8 cmd, u8 subcmd, u8 dat0, u8 dat1, u8 dat2);
void uart0_frame_tx_4(u8 cmd, u8 subcmd, u8 dat0, u8 dat1, u8 dat2, u8 dat3);
void uart0_frame_tx_12(u8 cmd, u8 subcmd, u8 dat0, u8 dat1, u8 dat2, u8 dat3,
                       u8 dat4, u8 dat5, u8 dat6, u8 dat7, u8 dat8, u8 dat9,
                       u8 dat10, u8 dat11);

#endif /* __UART_FRAME_H__ */
