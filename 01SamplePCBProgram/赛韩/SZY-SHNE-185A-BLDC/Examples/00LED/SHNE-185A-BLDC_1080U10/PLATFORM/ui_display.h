#ifndef __UI_DISPLAY_H
#define __UI_DISPLAY_H

#include "main.h"
#include "led_driver.h"

/* UI: set LED_DRIVER_TYPE in led_driver.h. Per-driver: UI_GRID_MAP, UI_SEG_D_TO_SEG[8].
 * TM1640: Dn -> chip SEG(n+1); array value = canonical seg a..g (0..6), 8 = NC. DP -> UI_SEG_DP_HW. */

#if (LED_DRIVER_TYPE == LED_DRIVER_TM1620)

#define UI_GRID_MAX     6
#define UI_GRID_MAP     {0, 1, 4, 2, 3, 5}
#define UI_SEG_D_TO_SEG   {0, 1, 2, 3, 4, 5, 6, 7}
#define UI_SEG_DP_HW      7
#define UI_GRID_BLINK_SEG_MASK  0x7Fu

#define UI_IND_GRID         4
#define UI_IND_TIME_BIT     0
#define UI_IND_SPEED_BIT    1
#define UI_IND_DIS_BIT      2
#define UI_IND_CAL_BIT      3
#define UI_IND_COLON_L_BIT  4
#define UI_IND_COLON_H_BIT  5

#elif (LED_DRIVER_TYPE == LED_DRIVER_TM1640)

/* 逻辑格 0..9：速度(0-2)、时间(3-6)、路程/卡(7-9)，与板 GR2..G13 对应。
 * 格 10：G10；格 11：G5；格 12：G14 橙条；格 13：G1 蓝条。TM1640 缓冲下标 = GRn−1。 */
#define UI_GRID_MAX     14
#define UI_GRID_MAP     {1, 2, 3, 5, 6, 7, 8, 10, 11, 12, 9, 4, 13, 0}

/* D0..D7 对应板 S1..S8；数组值=标准段 a..g(0..6)，7=本线不驱段。S1=dp→UI_SEG_DP_HW */
#define UI_SEG_D_TO_SEG   {7, 3, 4, 2, 6, 1, 5, 0}
#define UI_SEG_DP_HW      0
#define UI_GRID_BLINK_SEG_MASK  0xFFu

#elif (LED_DRIVER_TYPE == LED_DRIVER_TM1652)

#define UI_GRID_MAX     5
#define UI_GRID_MAP     {0, 1, 2, 3, 4}
#define UI_SEG_D_TO_SEG   {0, 1, 2, 3, 4, 5, 6, 7}
#define UI_SEG_DP_HW      7
#define UI_GRID_BLINK_SEG_MASK  0x7Fu

#define UI_IND_GRID         4
#define UI_IND_TIME_BIT     0
#define UI_IND_SPEED_BIT    1
#define UI_IND_DIS_BIT      2
#define UI_IND_CAL_BIT      3
#define UI_IND_COLON_L_BIT  4
#define UI_IND_COLON_H_BIT  5

#else
  #error "Unknown LED_DRIVER_TYPE: set in led_driver.h"
#endif


typedef enum {
    ZERO_SHOW = 0,
    ZERO_HIDE = 1
} ZeroMode_t;

typedef enum {
    DISP_OFF   = 0,
    DISP_ON,
    DISP_BLINK
} DispMode_t;


void UI_Engine_Init(void);
void UI_Engine_Refresh(void);
void UI_SetNumber(uint8_t start, uint16_t val, uint8_t len,
                  uint8_t dot_pos, ZeroMode_t mode);
void UI_SetBitMode(uint8_t grid, uint8_t bit_index, DispMode_t mode);
/* OR D0..D7 after 7seg pack（colon 等可选用） */
void UI_SetGridHwOverlay(uint8_t logic_grid, uint8_t d_mask, DispMode_t mode);
void UI_SetRaw(uint8_t pos, uint8_t raw);
void UI_SetBlinkTime(uint16_t ms);
void UI_SetGridBlinkMask(uint8_t logic_grid, _Bool enable);
/* 按格写入闪烁遮罩（0x7F=只闪 a~g，DP 位另用 SetBitMode DISP_BLINK）；seg_mask=0 清除该格闪烁 */
void UI_SetGridBlinkMaskValue(uint8_t logic_grid, uint8_t seg_mask);
void UI_ClearAll(void);
void UI_MarkDirty(void);

#endif
