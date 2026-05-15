#include "ui_display.h"
#include "plat_segment.h"
#include "led_driver.h"

/* Pack canonical a..g via UI_SEG_D_TO_SEG -> driver D0..D7 */
static const uint8_t g_SegDToSeg[8] = UI_SEG_D_TO_SEG;

static uint8_t prv_code_to_canonical(uint8_t code)
{
    uint8_t c = 0;
    if (code & _SEG_A) c |= (uint8_t)(1u << 0);
    if (code & _SEG_B) c |= (uint8_t)(1u << 1);
    if (code & _SEG_C) c |= (uint8_t)(1u << 2);
    if (code & _SEG_D) c |= (uint8_t)(1u << 3);
    if (code & _SEG_E) c |= (uint8_t)(1u << 4);
    if (code & _SEG_F) c |= (uint8_t)(1u << 5);
    if (code & _SEG_G) c |= (uint8_t)(1u << 6);
    return c;
}

static uint8_t prv_seg_pack_to_hw(uint8_t code)
{
    uint8_t can = prv_code_to_canonical((uint8_t)(code & 0x7Fu));
    uint8_t out = 0;
    for (uint8_t hw = 0; hw < 8u; hw++) {
        uint8_t seg = g_SegDToSeg[hw];
        if (seg < 7u && (can & (uint8_t)(1u << seg)) != 0)
            out |= (uint8_t)(1u << hw);
    }
    if ((code & _SEG_DP) != 0)
        out |= (uint8_t)(1u << UI_SEG_DP_HW);
    return out;
}

static const uint8_t g_GridMap[UI_GRID_MAX] = UI_GRID_MAP;

static uint8_t  g_ShadowRAM[UI_GRID_MAX] = {0};
static uint8_t  g_BlinkMask[UI_GRID_MAX] = {0};
static uint8_t  g_HwOr[UI_GRID_MAX] = {0};
static uint8_t  g_HwOrBlink[UI_GRID_MAX] = {0};
static uint16_t g_BlinkThreshold = 10; /* blink half-period ~= threshold * refresh period */
static _Bool    g_IsDirty = 1;


void UI_Engine_Init(void)
{
    LED_Init();
    UI_ClearAll();
}

void UI_Engine_Refresh(void)
{
    static uint16_t tick = 0;
    static _Bool blink_phase = 0;

    if (++tick >= g_BlinkThreshold) {
        tick = 0;
        blink_phase = !blink_phase;
        g_IsDirty = 1;
    }

    if (!g_IsDirty) return;

    for (uint8_t i = 0; i < UI_GRID_MAX; i++) {
        uint8_t phys = g_GridMap[i];
        uint8_t data = g_ShadowRAM[i];
        if (blink_phase == 0)
            data &= ~g_BlinkMask[i];
        data = prv_seg_pack_to_hw(data);
        uint8_t orv = g_HwOr[i];
        if (blink_phase == 0)
            orv &= (uint8_t)~g_HwOrBlink[i];
        data |= orv;
        LED_WriteBuf(phys, data);
    }

    LED_Refresh();
    g_IsDirty = 0;
}


void UI_SetBitMode(uint8_t logic_grid, uint8_t bit_index, DispMode_t mode)
{
    if (logic_grid >= UI_GRID_MAX || bit_index > 7) return;
    uint8_t mask = (uint8_t)(1 << bit_index);

    if (mode == DISP_ON) {
        g_ShadowRAM[logic_grid] |=  mask;
        g_BlinkMask[logic_grid] &= ~mask;
    } else if (mode == DISP_OFF) {
        g_ShadowRAM[logic_grid] &= ~mask;
        g_BlinkMask[logic_grid] &= ~mask;
    } else {
        g_ShadowRAM[logic_grid] |= mask;
        g_BlinkMask[logic_grid] |= mask;
    }
    g_IsDirty = 1;
}

/* OR raw driver segment bits after prv_seg_pack_to_hw */
void UI_SetGridHwOverlay(uint8_t logic_grid, uint8_t d_mask, DispMode_t mode)
{
    if (logic_grid >= UI_GRID_MAX) return;
    d_mask &= 0xFFu;
    if (mode == DISP_ON) {
        g_HwOr[logic_grid] |= d_mask;
        g_HwOrBlink[logic_grid] &= (uint8_t)~d_mask;
    } else if (mode == DISP_OFF) {
        g_HwOr[logic_grid] &= (uint8_t)~d_mask;
        g_HwOrBlink[logic_grid] &= (uint8_t)~d_mask;
    } else {
        g_HwOr[logic_grid] |= d_mask;
        g_HwOrBlink[logic_grid] |= d_mask;
    }
    g_IsDirty = 1;
}


void UI_SetNumber(uint8_t start, uint16_t val, uint8_t len,
                  uint8_t dot_pos, ZeroMode_t mode)
{
    uint16_t t_val = val;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t pos  = start + (len - 1 - i);
        if (pos >= UI_GRID_MAX) continue;
        uint8_t code = Disp_GetDigitCode(t_val % 10);
        if (dot_pos == (i + 1)) code = Disp_AddDot(code);
        if (mode == ZERO_HIDE && t_val == 0 && i > 0 && i >= (dot_pos ? dot_pos : 1))
            code = 0;
        UI_SetRaw(pos, code);
        t_val /= 10;
    }
}


void UI_SetRaw(uint8_t logic_pos, uint8_t raw_data)
{
    if (logic_pos < UI_GRID_MAX && g_ShadowRAM[logic_pos] != raw_data) {
        g_ShadowRAM[logic_pos] = raw_data;
        g_IsDirty = 1;
    }
}

void UI_SetBlinkTime(uint16_t ms)
{
    g_BlinkThreshold = (ms < 100) ? 1 : (ms / 100);
}

void UI_SetGridBlinkMask(uint8_t logic_grid, _Bool enable)
{
    if (logic_grid >= UI_GRID_MAX) return;
    if (enable)
        g_BlinkMask[logic_grid] |= UI_GRID_BLINK_SEG_MASK;
    else
        g_BlinkMask[logic_grid] &= (uint8_t)~UI_GRID_BLINK_SEG_MASK;
    g_IsDirty = 1;
}

void UI_MarkDirty(void)
{
    g_IsDirty = 1;
}

void UI_ClearAll(void)
{
    for (uint8_t i = 0; i < UI_GRID_MAX; i++) {
        g_ShadowRAM[i] = 0;
        g_BlinkMask[i] = 0;
        g_HwOr[i] = 0;
        g_HwOrBlink[i] = 0;
    }
    g_IsDirty = 1;
}
