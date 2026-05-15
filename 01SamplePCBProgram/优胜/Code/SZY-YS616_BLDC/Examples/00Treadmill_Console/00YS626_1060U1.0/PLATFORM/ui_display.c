#include "ui_display.h"
#include "plat_segment.h"
#include "tm1620.h"
//#include "tm1652.h"
//#include "tm1640.h"

/* 
 * 硬件修正映射表
 * 逻辑位置: 0, 1, 2, 3, 4, 5
 * 物理位置: 0, 1, 4, 2, 3, 5
 */
static const uint8_t g_GridMap[UI_GRID_MAX] = {0, 1, 4, 2, 3, 5};

static uint8_t  g_ShadowRAM[UI_GRID_MAX] = {0}; // 显存数据
static uint8_t  g_BlinkMask[UI_GRID_MAX] = {0}; // 闪烁掩码
static uint16_t g_BlinkThreshold = 10;          // 闪烁反转，默认 500ms (5 * 100ms)
static _Bool    g_IsDirty = 1;


void UI_Engine_Init(void) {
    TM1620_Init();
//	TM1652_Init();
//	TM1640_Init();
    UI_ClearAll();
}

/**
 * @brief 核心物理同步引擎 (100ms 驱动)
 */
void UI_Engine_Refresh(void) {
    static uint16_t tick = 0;
    static _Bool blink_phase = 0; // 0: 灭相位, 1: 亮相位
    
    // 处理动态闪烁时钟
    if (++tick >= g_BlinkThreshold) {
        tick = 0;
        blink_phase = !blink_phase;
        g_IsDirty = 1; // 状态改变，强制刷新
    }

    if (!g_IsDirty) return;

    // 映射与合成
    for (uint8_t logic_i = 0; logic_i < UI_GRID_MAX; logic_i++) {
        uint8_t phys_i = g_GridMap[logic_i]; // 查映射表，解决画错线
        uint8_t send_data = g_ShadowRAM[logic_i];

        // 如果该位在闪烁掩码中，且处于“灭”相位，则剔除对应段
        if (blink_phase == 0) {
            send_data &= ~g_BlinkMask[logic_i];
        }

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


/**
 * @brief  万能数字渲染引擎 (World-Class Number Rendering Engine)
 * @note   该函数实现了将逻辑数值转换为数码管硬件段码的全过程，包含自动对齐、动态小数点注入及智能前导零消隐技术。
 * 
 * @param  start   : 起始逻辑格位索引 (从左往右，0 为第一格)
 * @param  val     : 待显示的原始数值 (例如显示 12.5 则传入 125)
 * @param  len     : 占用的数码管总格数 (定义了排版宽度)
 * @param  dot_pos : 小数点位置 (从右往左数，1:个位后, 2:十位后, 0:无小数点)
 * @param  mode    : 消隐模式 (ZERO_HIDE: 开启智能消隐, ZERO_SHOW: 全显示)
 */
void UI_SetNumber(uint8_t start, uint16_t val, uint8_t len, uint8_t dot_pos, ZeroMode_t mode) {
    uint16_t t_val = val;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t l_pos = start + (len - 1 - i);
        if (l_pos >= UI_GRID_MAX) continue;
        uint8_t code = Disp_GetDigitCode(t_val % 10);
        if (dot_pos == (i + 1)) code = Disp_AddDot(code);
		
        // 前导零消隐算法
        if (mode == ZERO_HIDE && t_val == 0 && i > 0 && i >= (dot_pos?dot_pos:1)) code = 0;
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
	g_BlinkThreshold = (ms < 100) ? 1 : (ms / 100); 
}

/**
 * @brief 设置特定格子的数字位(SEG1-7)是否参与闪烁
 */
void UI_SetGridBlinkMask(uint8_t logic_grid, _Bool enable) {
    if (logic_grid >= UI_GRID_MAX) return;
    if (enable) 
        g_BlinkMask[logic_grid] |= 0x7F; // 仅闪烁段码 A-G
    else        
        g_BlinkMask[logic_grid] &= 0x80; // 保留 bit7(点/图标) 的状态
    g_IsDirty = 1;
}

/**
 * @brief 强制标记显示为“脏”状态
 * @note  当逻辑层需要强制刷新硬件（例如从常亮切换到闪烁模式）时调用
 */
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
