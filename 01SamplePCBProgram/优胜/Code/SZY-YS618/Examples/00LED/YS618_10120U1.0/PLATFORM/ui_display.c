#include "ui_display.h"
#include "plat_segment.h"
#include "tm1620.h"
//#include "tm1652.h"
//#include "tm1640.h"

/* 逻辑格子下标映射到 TM1620 物理 GRID。
 * 0..3：四位数码管；4：指示灯/冒号；5：扩展。
 */
static const uint8_t g_GridMap[UI_GRID_MAX] = {3, 4, 0, 1, 2, 5};

static uint8_t  g_ShadowRAM[UI_GRID_MAX] = {0}; /* 显存 */
static uint8_t  g_BlinkMask[UI_GRID_MAX] = {0}; /* 闪烁掩码 */
static uint16_t g_BlinkThreshold = 25;          /* Task_20ms：25x20ms=500ms 半周期 */
static _Bool    g_IsDirty = 1;


void UI_Engine_Init(void) {
    TM1620_Init();
//	TM1652_Init();
//	TM1640_Init();
    UI_ClearAll();
}

/* 刷新显示；调度周期 20ms，与 Task_20ms 一致 */
void UI_Engine_Refresh(void) {
    static uint16_t tick = 0;
    static _Bool blink_phase = 0; /* 0：灭相，1：亮相 */

    if (++tick >= g_BlinkThreshold) {
        tick = 0;
        blink_phase = !blink_phase;
        g_IsDirty = 1;
    }

    if (!g_IsDirty) return;

    for (uint8_t logic_i = 0; logic_i < UI_GRID_MAX; logic_i++) {
        uint8_t phys_i = g_GridMap[logic_i];
        uint8_t send_data = g_ShadowRAM[logic_i];

        if (blink_phase == 0)
            send_data &= ~g_BlinkMask[logic_i];

        TM1620_WriteBuf(phys_i, send_data);
    }

    TM1620_Refresh();
    g_IsDirty = 0;
}


void UI_SetBitMode(uint8_t logic_grid, uint8_t bit_index, DispMode_t mode) {
    if (logic_grid >= UI_GRID_MAX || bit_index > 7) return;
    uint8_t mask = (1 << bit_index);

    if (mode == DISP_ON) {
        g_ShadowRAM[logic_grid] |= mask;
        g_BlinkMask[logic_grid] &= ~mask;
    } else if (mode == DISP_OFF) {
        g_ShadowRAM[logic_grid] &= ~mask;
        g_BlinkMask[logic_grid] &= ~mask;
    } else if (mode == DISP_BLINK) {
        g_ShadowRAM[logic_grid] |= mask;
        g_BlinkMask[logic_grid] |= mask;
    }
    g_IsDirty = 1;
}


/* 写数字：start 起始格；len 位数；dot_pos 小数位（0 无小数点） */
void UI_SetNumber(uint8_t start, uint16_t val, uint8_t len, uint8_t dot_pos, ZeroMode_t mode) {
    uint16_t t_val = val;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t l_pos = start + (len - 1 - i);
        if (l_pos >= UI_GRID_MAX) continue;
        uint8_t code = Disp_GetDigitCode(t_val % 10);
        if (dot_pos == (i + 1)) code = Disp_AddDot(code);

        if (mode == ZERO_HIDE && t_val == 0 && i > 0 && i >= (dot_pos ? dot_pos : 1)) code = 0;
        UI_SetRaw(l_pos, code);
        t_val /= 10;
    }
}


void UI_SetRaw(uint8_t logic_pos, uint8_t raw_data) {
    if (logic_pos < UI_GRID_MAX && g_ShadowRAM[logic_pos] != raw_data) {
        g_ShadowRAM[logic_pos] = raw_data;
        g_IsDirty = 1;
    }
}

void UI_SetBlinkTime(uint16_t ms) {
    /* UI_Engine_Refresh 每 20ms 调用：半周期毫秒数 / 20 = 阈值 */
    g_BlinkThreshold = (ms < 20) ? 1 : (ms / 20);
}

/* 整格 SEG1-8 参与闪烁（原 0x7F 不含 bit7，M 界面编辑时第 8 段不闪） */
void UI_SetGridBlinkMask(uint8_t logic_grid, _Bool enable) {
    if (logic_grid >= UI_GRID_MAX) return;
    if (enable)
        g_BlinkMask[logic_grid] |= 0xFFu;
    else
        g_BlinkMask[logic_grid] = 0;
    g_IsDirty = 1;
}

void UI_MarkDirty(void) {
    g_IsDirty = 1;
}

void UI_ClearAll(void) {
    for (uint8_t i = 0; i < UI_GRID_MAX; i++) {
        g_ShadowRAM[i] = 0;
        g_BlinkMask[i] = 0;
    }
    g_IsDirty = 1;
}
