#ifndef __UI_DISPLAY_H
#define __UI_DISPLAY_H

#include "main.h"

#define UI_GRID_MAX        6    

typedef enum {
    ZERO_SHOW = 0,               // 显示 0005
    ZERO_HIDE = 1                // 消隐    5 (高档显示标准)
} ZeroMode_t;

typedef enum {
    DISP_OFF = 0,     // 常灭
    DISP_ON,          // 常亮
    DISP_BLINK        // 闪烁
} DispMode_t;

/* --- API 接口 --- */
void UI_Engine_Init(void);
void UI_Engine_Refresh(void);    // 100ms 周期调用

// 业务调用
void UI_SetNumber(uint8_t start, uint16_t val, uint8_t len, uint8_t dot_pos, ZeroMode_t mode);
void UI_SetBitMode(uint8_t grid, uint8_t bit_index, DispMode_t mode); // 操控任意一位
void UI_SetRaw(uint8_t pos, uint8_t raw);
void UI_SetBlinkTime(uint16_t ms); 
void UI_SetGridBlinkMask(uint8_t logic_grid, _Bool enable);
void UI_ClearAll(void);						   // 全局清屏
void UI_MarkDirty(void); 
#endif
