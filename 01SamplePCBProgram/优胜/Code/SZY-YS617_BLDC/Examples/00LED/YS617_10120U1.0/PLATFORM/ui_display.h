#ifndef __UI_DISPLAY_H
#define __UI_DISPLAY_H

#include "main.h"

#define UI_GRID_MAX        6

typedef enum {
    ZERO_SHOW = 0,
    ZERO_HIDE = 1
} ZeroMode_t;

typedef enum {
    DISP_OFF = 0,
    DISP_ON,
    DISP_BLINK
} DispMode_t;

void UI_Engine_Init(void);
void UI_Engine_Refresh(void);    /* 20ms ÖÜÆÚ£¨¼û sys_scheduler Task_20ms£© */

void UI_SetNumber(uint8_t start, uint16_t val, uint8_t len, uint8_t dot_pos, ZeroMode_t mode);
void UI_SetBitMode(uint8_t grid, uint8_t bit_index, DispMode_t mode);
void UI_SetRaw(uint8_t pos, uint8_t raw);
void UI_SetBlinkTime(uint16_t ms);
void UI_SetGridBlinkMask(uint8_t logic_grid, _Bool enable);
void UI_ClearAll(void);
void UI_MarkDirty(void);
#endif
