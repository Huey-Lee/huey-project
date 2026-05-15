#ifndef __PLAT_BLE_H
#define __PLAT_BLE_H

#include "main.h"
#include "plat_touchkey.h"

/** 《跑步机或走步机_主控与蓝牙通讯协议规范 V2.3》UART2 PB02/PB03 */

void Ble_Init(void);
void Ble_ProcessRx(void);
void Ble_OnTimer100ms(void);
void Ble_OnPanelKey(KeyID_t id, KeyEvt_t evt);

/** 下位/显示侧数据变更时触发蓝牙运动数据上报 */
void Ble_NotifyMetricsFromDisplay(void);

#endif
