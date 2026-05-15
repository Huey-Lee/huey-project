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

#define UI_GRID_MAX     12
#define UI_GRID_MAP     {7, 8, 9, 5, 6, 10, 14, 13, 4, 15, 12, 11}

/* TM1640: D0=d,D1=NC in 7seg map,D2=c,...; DP wired to SEG2=D1 -> UI_SEG_DP_HW 1 */
#define UI_SEG_D_TO_SEG   {3, 8, 2, 6, 5, 4, 0, 1}
#define UI_SEG_DP_HW      1
#define UI_GRID_BLINK_SEG_MASK  0xFFu
#define TM1640_D_TO_SEG   UI_SEG_D_TO_SEG

#define UI_IND_GRID         11
#define UI_IND_TIME_BIT     0
#define UI_IND_SPEED_BIT    1
#define UI_IND_DIS_BIT      2
#define UI_IND_CAL_BIT      3
#define UI_IND_COLON_L_BIT  4
#define UI_IND_COLON_H_BIT  5

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
/* OR D0..D7 after 7seg pack (colon on NC line) */
void UI_SetGridHwOverlay(uint8_t logic_grid, uint8_t d_mask, DispMode_t mode);
void UI_SetRaw(uint8_t pos, uint8_t raw);
void UI_SetBlinkTime(uint16_t ms);
void UI_SetGridBlinkMask(uint8_t logic_grid, _Bool enable);
void UI_ClearAll(void);
void UI_MarkDirty(void);

#endif
