#ifndef __PLAT_AEROLINK_H
#define __PLAT_AEROLINK_H

#include "main.h"

/* 协议常量 - 严格匹配 17 字节 */
#define TREAD_HEAD1      0xAA
#define TREAD_HEAD2      0xBB
#define TREAD_TAIL1      0xEE
#define TREAD_TAIL2      0xFF
#define TREAD_CID_SEND   0x0A  // 上控 -> 下控
#define TREAD_CID_RECV   0x09  // 下控 -> 上控

/* 功能码 FC 定义 */
#define FC_HEARTBEAT     0x01  // 心跳/状态
#define FC_CONTROL       0x02  // 启停控制
#define FC_GEAR          0x03  // 档位调节
#define FC_TEST          0xFF  // 出厂测试


/* 结构化接收帧 */
typedef struct {
    uint8_t cid;
    uint8_t fc;
    uint8_t sfc;
    uint8_t data[8]; // 对应图表中的 数据1H~数据4L
} AeroFrame_t;

/* API */
void AeroLink_Handler(void); 
void AeroLink_Send(uint8_t fc, uint8_t sfc, uint8_t *pData);
extern void AeroLink_OnFrameReceived(AeroFrame_t *f); // 业务回调

#endif
