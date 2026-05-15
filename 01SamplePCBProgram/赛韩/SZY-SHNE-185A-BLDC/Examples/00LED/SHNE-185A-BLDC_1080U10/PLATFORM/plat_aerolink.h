#ifndef __PLAT_AEROLINK_H
#define __PLAT_AEROLINK_H

#include "main.h"

/* 协议基础常量 */
#define TREAD_HEAD1      0xAA
#define TREAD_HEAD2      0xBB
#define TREAD_HEAD2_ALT  0xB8  /* 部分下控→上控帧第二字节为 B8，与 BB 二选一 */
#define TREAD_TAIL1      0xEE
#define TREAD_TAIL2      0xFF
#define TREAD_CID_SEND   0x0A  // 上控发给下控的控制码
#define TREAD_CID_RECV   0x09  // 下控发给上控 

/* 常用功能码 FC 定义 */
#define FC_HEARTBEAT     0x01  // 心跳
#define FC_CONTROL       0x02  // 启动/停止控制
#define FC_GEAR          0x03  // 档位/速度调节
#define FC_FACTORY       0xFF  // 出厂测试

/* FC_CONTROL 子功能码（与下控协议表一致） */
#define FC_CTRL_SFC_RUN_START   0x00  /* 启动运行 */
#define FC_CTRL_SFC_STOP        0x01  /* 普通停机 */
#define FC_CTRL_SFC_SAFETY_ESTOP 0x02 /* 安全锁急停（协议：FC=02 SFC=02，数据 8 字节 0） */

/* 接收帧结构体 */
typedef struct {
    uint8_t fc;
    uint8_t sfc;
    uint8_t data[8];
} AeroRecvFrame_t;

/* API 接口 */
/**
 * @brief 发送 17 字节标准协议帧
 * @param fc  功能码
 * @param sfc 子功能码
 * @param pData 8字节数据位指针 (若为NULL则自动补0)
 */
void AeroLink_Send(uint8_t fc, uint8_t sfc, uint8_t *pData);
void AeroLink_Handler(void); // 核心接收轮询
extern void AeroLink_OnFrameReceived(AeroRecvFrame_t *pFrame);// 业务回调：当接收到一个合法包时由底层自动调用

#endif

