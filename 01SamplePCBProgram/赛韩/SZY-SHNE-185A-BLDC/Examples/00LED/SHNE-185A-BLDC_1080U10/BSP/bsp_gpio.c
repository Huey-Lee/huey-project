#include "bsp_gpio.h"


void BSP_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 开时钟
    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_GPIOB_CLK_ENABLE();

    // CLK (PA03) + DIN (PA04) → 输出（软件模拟上拉）
    GPIO_InitStruct.Pins = TM1620_CLK_PIN | TM1620_DIN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(CW_GPIOA, &GPIO_InitStruct);

    // STB (PB01) → 输出
    GPIO_InitStruct.Pins = TM1620_STB_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(TM1620_STB_PORT, &GPIO_InitStruct);
	
	// 配置 433 接收口 (PB00)
    GPIO_InitStruct.Pins = RF_REC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT_PULLUP; // 上拉输入：抗干扰、信号稳定
    GPIO_Init(RF_REC_PORT, &GPIO_InitStruct);

    /* 安全锁 PB04：片内上拉，拔锁悬空/高为「断开」 */
    GPIO_InitStruct.Pins = SAFETY_LOCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_Init(SAFETY_LOCK_PORT, &GPIO_InitStruct);

    // 默认拉高（软件上拉效果）
    STB_HIGH();
    CLK_HIGH();
    DIN_HIGH();
}
