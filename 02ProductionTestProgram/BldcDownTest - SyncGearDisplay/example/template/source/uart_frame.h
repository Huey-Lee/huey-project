/**
 * @file    uart_frame.h
 * @author  HueyLee
 * @brief   Communication Protocol Stack
 * @date		2026??3??1??
 */

#ifndef __UART_FRAME_H__
#define __UART_FRAME_H__

#include "common.h"
#include "queue.h"

#pragma pack(push, 1)

#define RX_BUFF_SIZE    (300)
#define PROTOCOL_SOF1   (0xAA)
#define PROTOCOL_SOF2   (0xBB)
#define PROTOCOL_LEN    (0x0A)
#define PROTOCOL_EOF1   (0xEE)
#define PROTOCOL_EOF2   (0xFF)

#define CTRL_CODE_DOWN  (0x0A) // ???????????????
#define CTRL_CODE_UP    (0x09) // ?????????????

#define LOWER_ST_WAIT_START  (7u) /* ??????? */

typedef union {
    struct {
        u8 sof1;            // 1: 0xAA
        u8 sof2;            // 2: 0xBB
        u8 ctrl;            // 3: 0x0A (??????/????)
        u8 func;            // 4: ??????
        u8 sub_func;        // 5: ???????
        u8 d1_h;            // 6: ????1??¦Ë
        u8 d1_l;            // 7: ????1??¦Ë
        u8 d2_h;            // 8: ????2??¦Ë
        u8 d2_l;            // 9: ????2??¦Ë
        u8 d3_h;            // 10: ????3??¦Ë
        u8 d3_l;            // 11: ????3??¦Ë
        u8 d4_h;            // 12: ????4??¦Ë
        u8 d4_l;            // 13: ????4??¦Ë
        u8 cs_h;            // 14: §µ???¦Ë (0x00)
        u8 cs_l;            // 15: §µ???¦Ë (????)
        u8 eof1;            // 16: 0xEE
        u8 eof2;            // 17: 0xFF
    } m;
    u8 raw[17];             // ???????????
} uart_frame_t;

#pragma pack(pop)


typedef enum {
    CMD_INFO   = 0x01,      // ???????/???????
    CMD_ANALOG = 0x02,      // ????????/????/??? 
    CMD_TEST   = 0xFF       // ???????? 
} cmd_e;

/* ?????????????? */
extern uart_frame_t uart_frame;
extern uart_frame_t uart_tx_frame;
extern void uart_frame_init(void);
extern void uart_frame_loop(void);
extern void uart_frame_tx(u8 func, u8 sub, u8 data_low);
extern void uart_frame_tx_d1(u8 func, u8 sub, u16 data1_hl);
extern void uart0_init(void);

#define UART0 0
#define UART1 1

void UART_Send_Buf(u8 UARTPort, u8 *ptr, u8 len);

#endif
