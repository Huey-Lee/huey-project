#ifndef __PLAT_KEYLINK_RX_H__
#define __PLAT_KEYLINK_RX_H__

#include "main.h"

/** 1ms：从 uart2_rx_buf 解析 Keylink */
void KeylinkRx_Handler_1ms(void);
/** 约每 500ms：向按键板发一帧显示板链路心跳（0x22） */
void KeylinkRx_OnTimer100ms(void);
/** 上电后立即发一帧链路心跳（可选，便于示波器抓包） */
void KeylinkRx_SendLinkPing(void);
/** 将当前目标速（0.1 km/h）同步给按键板，帧类型 0x30 */
void KeylinkRx_SendTargetSpeed0p1(uint8_t speed_0p1);
#ifndef KEYLINK_RX_WATCHDOG_MAX
/** 约 3s（×100ms）未收到按键板 Keylink 有效帧则报 E16 */
#define KEYLINK_RX_WATCHDOG_MAX 30u
#endif

/** 解析到一帧完整 Keylink 时调用（喂狗） */
void KeylinkRx_OnValidFrame(void);
/** 每 100ms 调用一次；monitor_active=0 时计数清零。返回 1 表示已超过 KEYLINK_RX_WATCHDOG_MAX */
uint8_t KeylinkRx_WatchdogExpired100ms(uint8_t monitor_active);
void KeylinkRx_ResetWatchdog(void);

#endif
