#ifndef __PLAT_AEROLINK_H
#define __PLAT_AEROLINK_H

#include "main.h"

/* 7F+CMD+LEN+DATA+CHK+0+7E. CHK: 默认异或；接收时异或或 (CMD+LEN+DATA)%7 均判有效。
 * READ_PARAM(8) 上显发 %7 得 0x02：7F 08 01 00 02 00 7E. 勿用 LEN=0(否则 0x08^0=0x08). */
#define AEROLINK_STX           0x7F
#define AEROLINK_ETX           0x7E
#define AEROLINK_MAX_DATA      8u

/* 命令字 1.. */
#define CMD_STATUS                 1u
#define CMD_SPEED                  2u
#define CMD_ACK                    3u
#define CMD_ERROR                  4u
#define CMD_RESET                  5u
#define CMD_STOP_OVER              6u
#define CMD_WRITE_PARAM            7u
#define CMD_READ_PARAM             8u
#define CMD_TEST                   9u
#define CMD_STATUS_INQUIRE         10u
#define CMD_STATUS_URGENT_STOP     11u

/* 与 HC-32_1 example uart_frame.h 中 cmd_e 枚举一致（1 递增到 CMD_OVER_VALTAGE_MAX=35）。 */
#define CMD_TREADMILLS_SPEED_MAX   12u
#define CMD_VOLTAGE_MAX            13u
#define CMD_VOLTAGE_MIN            14u
#define CMD_OVER_CURRENT_MAX       15u
#define CMD_KIV_1KM                16u
#define CMD_KIV_2KM                17u
#define CMD_KIV_3KM                18u
#define CMD_KIV_4KM                19u
#define CMD_KIV_5KM                20u
#define CMD_KIV_6KM                21u
#define CMD_KIV_7KM                22u
#define CMD_KIV_8KM                23u
#define CMD_KIV_9KM                24u
#define CMD_KIV_10KM               25u
#define CMD_KIV_11KM               26u
#define CMD_KIV_12KM               27u
#define CMD_STA_LEVEL              28u   /* 1..5，见下控 uart_frame.c */
#define CMD_END_LEVEL              29u
#define CMD_ACCELERATED_LEVEL      30u
#define CMD_DECELERATION_LEVEL     31u
#define CMD_PID_P                  32u
#define CMD_PID_I                  33u
#define CMD_PID_D                  34u
#define CMD_OVER_VOLTAGE_MAX       35u

#define STATUS_MOTOR_STOP          0u
#define STATUS_MOTOR_RUN           1u

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t data[AEROLINK_MAX_DATA];
} AeroRecvFrame_t;

void AeroLink_Send(uint8_t cmd, uint8_t len, const uint8_t *pData); /* len=0 时 pData 可 NULL */
void AeroLink_Handler(void);
extern void AeroLink_OnFrameReceived(AeroRecvFrame_t *pFrame);

#endif
