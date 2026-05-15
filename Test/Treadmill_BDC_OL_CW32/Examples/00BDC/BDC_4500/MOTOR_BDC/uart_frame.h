/* uart_frame.h - display protocol, frame struct, RX buffer. */
#ifndef USER_UART_FRAME_H_
#define USER_UART_FRAME_H_

#include <stdint.h>
#include "queue.h"

#define UART0 0

#define RX_BUFF_SIZE   (135)
#define START_OF_FRAME  (0X7F)
#define END_OF_FRAME    (0X7E)

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

    CMD_TREADMILLS_SPEED_MAX,
    CMD_VOLTAGE_MAX,
    CMD_VOLTAGE_MIN,
    CMD_OVER_CURRENT_MAX,
    CMD_KIV_1KM,
    CMD_KIV_2KM,
    CMD_KIV_3KM,
    CMD_KIV_4KM,
    CMD_KIV_5KM,
    CMD_KIV_6KM,
    CMD_KIV_7KM,
    CMD_KIV_8KM,
    CMD_KIV_9KM,
    CMD_KIV_10KM,
    CMD_KIV_11KM,
    CMD_KIV_12KM,

    CMD_STA_LEVEL,
    CMD_END_LEVEL,
    CMD_ACCELERATED_LEVEL,
    CMD_DECELERATION_LEVEL,

    CMD_PID_P,
    CMD_PID_I,
    CMD_PID_D,
    CMD_OVER_VALTAGE_MAX,
} cmd_e;

#define DATA_BUF_SIZE  (2)
#define KIV_NUM        (12)

typedef struct _t_uart_frame
{
    uint8_t sof;
    uint8_t cmd;
    uint8_t len;
    uint8_t buf[DATA_BUF_SIZE];
    uint8_t crch;
    uint8_t crcl;
    uint8_t eof;
} uart_frame_t;

typedef struct _ee_param_t
{
    uint8_t sof;
    uint8_t treadmills_speed_max;
    uint8_t voltage_max;
    uint8_t voltage_min;
    uint8_t over_current_max_k;
    uint8_t kiv[KIV_NUM];
} ee_param_t;

extern T_QUEUE rx_queue;
extern uart_frame_t uart_frame;
extern uart_frame_t uart_tx_frame;

extern void uart_frame_init(void);
extern void uart_frame_loop(void);
extern void uart_frame_tx(uint8_t cmd, uint8_t dat);
extern void uart_frame_tx_2(uint8_t cmd, uint8_t dat0, uint8_t dat1);
extern void communication_checkout(void);

extern void cmd_proc(void);
void UART_Send_Buf(uint8_t UARTPort, uint8_t *ptr, uint8_t len);

#endif /* USER_UART_FRAME_H_ */
