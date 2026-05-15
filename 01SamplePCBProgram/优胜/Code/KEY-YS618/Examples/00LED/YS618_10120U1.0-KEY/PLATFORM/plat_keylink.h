#ifndef __PLAT_KEYLINK_H
#define __PLAT_KEYLINK_H

#include "main.h"
#include "plat_touchkey.h"

/* 按键板 -> 显示板：固定 5 字节帧 [0x55][TYPE][DATA][XOR][0xAA]，XOR = TYPE ^ DATA */
#ifndef PLAT_USE_KEYLINK
#define PLAT_USE_KEYLINK   1
#endif

#define KEYLK_SOF           0x55u
#define KEYLK_EOF           0xAAu
#define KEYLK_TYPE_KEYDOWN  0x10u
#define KEYLK_TYPE_KEYUP    0x11u
#define KEYLK_TYPE_HEARTBEAT 0x20u
/** 显示板 → 按键板链路在线指示（UART1 RX PA00 接收） */
#define KEYLK_TYPE_DISP_LINK 0x22u
/** 与显示板扩展约定：实时速度，DATA=0.1km/h（如 60=6.0km/h） */
#define KEYLK_TYPE_SPEED0P1  0x30u

/* DATA：按键 0x01~0x0F；心跳 0x00~0xFF 心率 BPM */
#define KEYLK_DATA_START     0x01u
#define KEYLK_DATA_STOP      0x02u
#define KEYLK_DATA_SPD_UP    0x03u
#define KEYLK_DATA_SPD_DOWN  0x04u
#define KEYLK_DATA_MODE      0x05u
#define KEYLK_DATA_PROG      0x06u
#define KEYLK_DATA_INC_UP    0x07u
#define KEYLK_DATA_INC_DOWN  0x08u
#define KEYLK_DATA_HOTKEY_4K  0x09u
#define KEYLK_DATA_HOTKEY_6K  0x0Au
#define KEYLK_DATA_HOTKEY_8K  0x0Bu
#define KEYLK_DATA_HOTKEY_10K 0x0Cu
#define KEYLK_DATA_HOTKEY_2K  0x0Du
#define KEYLK_DATA_HOTKEY_5K  0x0Eu
#define KEYLK_DATA_HOTKEY_12K 0x0Fu

void Keylink_OnKeypadEvent(KeyID_t id, KeyEvt_t evt);
/** 是否有按键处于按下（Keylink 已发 KEYDOWN 未发 KEYUP），供 HR 等逻辑选用 */
uint8_t Keylink_IsPanelKeyDown(void);
void Keylink_SendHeartbeat(uint8_t bpm);
void Keylink_SendSpeed0p1(uint8_t speed_0p1);
void Keylink_SimKeyTap(KeyID_t id);

/** 1ms：解析显示板下发的链路帧（须将显示板 PB02 TX 接至本板 PA00 RX） */
void Keylink_DisplayPoll_1ms(void);
/** 2s 内收到过 0x22 链路帧则为 1 */
uint8_t Keylink_DisplayLinkOk(void);

#endif
