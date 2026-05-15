#ifndef __PLAT_AEROLINK_H
#define __PLAT_AEROLINK_H

#include "main.h"

/*============================================================================
 * 17 字节固定帧（与 00YS626_1060U1.0 一致）
 * AA BB | CID | FC | SFC | Data1H..Data4L(8B) | CSH CSL | EE FF
 * 校验：16 位无符号和，范围为 [CID .. 数据末字节]
 * 上控发送 CID=TREAD_CID_SEND(0x0A)，下控上报 TREAD_CID_RECV(0x09)
 *============================================================================*/

#define TREAD_HEAD1      0xAAu
#define TREAD_HEAD2      0xBBu
#define TREAD_TAIL1      0xEEu
#define TREAD_TAIL2      0xFFu
#define TREAD_CID_SEND   0x0Au  /* 上控 -> 下控 */
#define TREAD_CID_RECV   0x09u  /* 下控 -> 上控 */

#define FC_HEARTBEAT     0x01u
#define FC_CONTROL       0x02u
#define FC_GEAR          0x03u

/* 与下控约定：进测帧 AA BB 09 FF FF（TREAD_CID_RECV + FC/SFC=0xFF），见 prv_uart_rx_is_factory_test */
#define CMD_STATUS                 1u

#define STATUS_MOTOR_STOP          0u

typedef struct {
    uint8_t cid;
    uint8_t fc;
    uint8_t sfc;
    uint8_t data[8];
} AeroFrame_t;

void AeroLink_Handler(void);
void AeroLink_Send(uint8_t fc, uint8_t sfc, uint8_t *pData);

extern void AeroLink_OnFrameReceived(AeroFrame_t *f);

#endif
