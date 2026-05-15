/*
 * uart_frame.h - BLDC 17-byte link (display <-> motor)
 */

#ifndef __UART_FRAME_H__
#define __UART_FRAME_H__

#include "common.h"
#include "queue.h"

#define RX_BUFF_SIZE (300)

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

#define STATUS_MOTOR_RUN   (1u)
#define STATUS_MOTOR_STOP  (0u)

/* App cmd ids; CMD_STOP_BURST kept 31 (old CMD_deceleration_LEVEL) */
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
/* P program: table stores whole km/h (e.g. 3 = 3.0); same internal + protocol map as 0.6~6.2 manual */
u16 uart_prog_kmh_to_internal(u8 kmh_whole);
void uart_frame_motor_run_internal_spd(u16 internal_spd);
void uart0_init(void);

#define UART0 0
#define UART1 1

void UART_Send_Buf(u8 UARTPort, u8 *ptr, u8 len);

#endif /* __UART_FRAME_H__ */
