#include "tm1620.h"
#include "bsp_gpio.h"


static uint8_t g_DispBuf[TM1620_BUF_SIZE];
// TM1620 亮度指令: 0x88(min) ~ 0x8F(max), 0x80(off)
static uint8_t g_CtrlCmd = 0x8A; 

// 极速字节写入 (48MHz下无需nop，依靠指令周期即可稳定)
static void TM1620_SendByte(uint8_t data) {
    for(uint8_t i=0; i<8; i++) {
        PA03_SETLOW(); // CLK
        if(data & 0x01) 
		PA04_SETHIGH(); 
		else 
			PA04_SETLOW(); // DIN
        data >>= 1;
        PA03_SETHIGH();
    }
}

static void TM1620_Cmd(uint8_t cmd) {
    PB01_SETLOW(); TM1620_SendByte(cmd); PB01_SETHIGH();
}

void TM1620_Init(void) {
    TM1620_ClearBuf();
    TM1620_Cmd(0x40); 		 // 自动地址增加模式
    TM1620_Refresh(); 		 // 先刷全灭，再开显示
    TM1620_SetBrightness(8); // 初始化为中等亮度
}


void TM1620_SetBrightness(uint8_t level) {
    if (level == 0) {
        g_CtrlCmd = 0x80; // 彻底关闭显示
    } else {
        if (level > 8) level = 8; // 防呆校验
        // 映射逻辑: 1->0x88, 2->0x89 ... 8->0x8F
        g_CtrlCmd = 0x88 + (level - 1); 
    }
    
    // 1. 立即生效 (写入控制寄存器)
    TM1620_Cmd(g_CtrlCmd);
}

void TM1620_Refresh(void) {
    TM1620_Cmd(0x40); 
    PB01_SETLOW();
    TM1620_SendByte(0xC0); // 起始地址
    for(uint8_t i=0; i<TM1620_BUF_SIZE; i++) {
        TM1620_SendByte(g_DispBuf[i]);
    }
    PB01_SETHIGH();
    TM1620_Cmd(g_CtrlCmd);
}

void TM1620_WriteBuf(uint8_t grid, uint8_t seg_data) {
    if(grid < 6) g_DispBuf[grid * 2] = seg_data;
}

void TM1620_ClearBuf(void) {
    for(uint8_t i=0; i<TM1620_BUF_SIZE; i++) g_DispBuf[i] = 0;
}

void TM1620_FillBuf(uint8_t data) {
    for(uint8_t i=0; i<TM1620_BUF_SIZE; i++) g_DispBuf[i] = data;
}
