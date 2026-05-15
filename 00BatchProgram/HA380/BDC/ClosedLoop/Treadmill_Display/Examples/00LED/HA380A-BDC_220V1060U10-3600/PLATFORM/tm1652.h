#ifndef __TM1652_H
#define __TM1652_H

#include "main.h"

/**
 * @file tm1652.h
 * @brief TM1652 异步单线 LED 驱动 - 顶尖性能版
 */

#ifndef __BSP_TM1652_H
#define __BSP_TM1652_H

#include "main.h"

/* --- 硬件连接配置 --- */
// 假设使用 PA05 作为数据线
#define TM1652_DAT_LOW()     PA05_SETLOW()
#define TM1652_DAT_HIGH()    PA05_SETHIGH()

/* --- TM1652 寄存器地址 --- */
#define TM1652_ADDR_DISP     0x00    // 显示数据起始地址 (00H-0EH)
#define TM1652_CMD_CTRL      0x18    // 系统控制命令地址

/* --- 显存大小 (TM1652 支持 7~8 Grid) --- */
#define TM1652_GRID_MAX      7

/* --- API 接口 --- */
void BSP_TM1652_Init(void);
void BSP_TM1652_Refresh(void); 
void BSP_TM1652_SetBrightness(uint8_t level); // 0(关)-8(最亮)

/* 缓存操作接口 (供 ui_display.c 调用) */
void TM1652_WriteBuf(uint8_t grid, uint8_t seg_data);
void TM1652_ClearBuf(void);
void TM1652_FillBuf(uint8_t data);

#endif

#endif

