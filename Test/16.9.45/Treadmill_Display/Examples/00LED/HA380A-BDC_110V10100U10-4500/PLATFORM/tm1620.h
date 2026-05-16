#ifndef __TM1620_H
#define __TM1620_H

#include "main.h"


// 显存大小：6 Grid * 2 (部分芯片为12字节)
#define TM1620_BUF_SIZE    12 

/* 核心 API */
void TM1620_Init(void);
void TM1620_Refresh(void); 

/* 设置亮度等级（level 0:关闭, 1:最暗, 8:最亮） */
void TM1620_SetBrightness(uint8_t level);

/* 快捷操作宏/接口 */
void TM1620_WriteBuf(uint8_t grid, uint8_t seg_data);
void TM1620_ClearBuf(void);
void TM1620_FillBuf(uint8_t data);
#endif
