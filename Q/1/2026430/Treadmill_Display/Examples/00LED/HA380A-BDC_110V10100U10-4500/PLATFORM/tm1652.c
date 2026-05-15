#include "tm1652.h"


static uint8_t g_DispBuf[TM1652_GRID_MAX];
static uint8_t g_CtrlReg = 0x00; // 内部控制寄存器状态

/**
 * @brief 微秒级延时 (针对 TM1652 的 19.2Kbps 速率, 每位约 52us)
 * @note 实际使用中，TM1652 兼容性很强，此处根据主频调优
 */
static void TM1652_DelayBit(void) {
    uint32_t i = 180; // 48MHz 下约 50us，匹配芯片采样率
    while(i--);
}

/**
 * @brief 发送一字节 (模拟标准 UART 帧: 1起始位 + 8数据位 + 1停止位)
 */
static void TM1652_SendByte(uint8_t b) {
    __disable_irq(); // 顶尖级：发送期间锁中断，防止时序偏移

    // 1. 起始位 (Low)
    TM1652_DAT_LOW();
    TM1652_DelayBit();

    // 2. 数据位 (LSB First)
    for (uint8_t i = 0; i < 8; i++) {
        if (b & 0x01) TM1652_DAT_HIGH();
        else          TM1652_DAT_LOW();
        b >>= 1;
        TM1652_DelayBit();
    }

    // 3. 停止位 (High)
    TM1652_DAT_HIGH();
    TM1652_DelayBit();

    __enable_irq(); 
}

/**
 * @brief 写入指定地址的数据
 */
static void TM1652_WriteData(uint8_t addr, uint8_t data) {
    // TM1652 协议：先发地址码 110xxxxx，再紧跟数据码
    TM1652_SendByte(0xC0 | addr); 
    TM1652_SendByte(data);
}

/* --- API 实现 --- */

void BSP_TM1652_Init(void) {
    // 1. GPIO 初始化 (推挽输出)
    TM1652_DAT_HIGH(); 
    TM1652_ClearBuf();
    
    // 2. 初始亮度设为 4 级
    BSP_TM1652_SetBrightness(4); 
    BSP_TM1652_Refresh();
}

/**
 * @brief 调节亮度 (0:关, 1-8级)
 * TM1652 指令格式: 11011 [B2][B1][B0]
 */
void BSP_TM1652_SetBrightness(uint8_t level) {
    if (level == 0) {
        g_CtrlReg = 0x00; // 关显示
    } else {
        if (level > 8) level = 8;
        // 亮度映射：0x18地址写 11011xxx
        // 0级亮:0x00, 1级亮:0x01 ... 7级亮:0x07
        g_CtrlReg = (level - 1) & 0x07;
    }
    // 立即更新控制指令包 (地址 0x18)
    TM1652_WriteData(TM1652_CMD_CTRL, g_CtrlReg);
}

/**
 * @brief 物理刷新：将缓冲区数据一次性同步到芯片
 * @note 顶尖建议每 50ms-100ms 调用一次
 */
void BSP_TM1652_Refresh(void) {
    // 1. 推送所有 Grid 显存
    for (uint8_t i = 0; i < TM1652_GRID_MAX; i++) {
        TM1652_WriteData(i, g_DispBuf[i]);
    }
    // 2. 每一轮刷新后同步亮度状态，防止干扰导致寄存器重置
    TM1652_WriteData(TM1652_CMD_CTRL, g_CtrlReg);
}

void TM1652_WriteBuf(uint8_t grid, uint8_t seg_data) {
    if(grid < TM1652_GRID_MAX) g_DispBuf[grid] = seg_data;
}

void TM1652_ClearBuf(void) {
    for(uint8_t i=0; i<TM1652_GRID_MAX; i++) g_DispBuf[i] = 0x00;
}

void TM1652_FillBuf(uint8_t data) {
    for(uint8_t i=0; i<TM1652_GRID_MAX; i++) g_DispBuf[i] = data;
}
