#include "bsp_pwm.h"

/**
 * @brief 初始化 PA06 为 GTIM1_CH1 PWM 输出
 */
void BSP_PWM_Init(void) {
    GTIM_InitTypeDef GTIM_InitStruct = {0};
    GTIM_OCModeCfgTypeDef GTIM_OCModeCfgStruct = {0};

    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_GTIM1_CLK_ENABLE();
    
    // PA06 复用配置为 GTIM1_CH1
    PA06_DIGTAL_ENABLE();
    PA06_DIR_OUTPUT();
    PA06_PUSHPULL_ENABLE();
    PA06_AFx_GTIM1CH1(); 

    GTIM_InitStruct.Prescaler = 48 - 1;   // 48MHz/48 = 1MHz 基频
    GTIM_InitStruct.ReloadValue = 370;    // 1MHz / 370 ≈ 2.7KHz
    GTIM_InitStruct.ARRBuffState = GTIM_ARR_BUFF_EN;
    GTIM_TimeBaseInit(CW_GTIM1, &GTIM_InitStruct);
    
    GTIM_OCModeCfgStruct.OCMode = GTIM_OC_MODE_PWM1;
    GTIM_OCModeCfgStruct.OCPolarity = GTIM_OC_POLAR_NONINVERT;
    GTIM_OC1ModeCfg(CW_GTIM1, &GTIM_OCModeCfgStruct); // 通道 1
    
    GTIM_Cmd(CW_GTIM1, DISABLE); 
}

void BSP_PWM_Set(uint16_t freq, uint8_t volume) {
    if (freq == 0 || volume == 0) { BSP_PWM_Stop(); return;}
	
    // 限制最大占空比为 50%，防止烧毁蜂鸣器且声音最响
    if (volume > 100) volume = 100;
    
    uint32_t reload = 1000000 / freq; 
    uint32_t pulse = (reload * volume) / 200; 
    
    GTIM_SetReloadValue(CW_GTIM1, reload);
    GTIM_SetCompare1(CW_GTIM1, pulse); // 设置通道 1 比较值
    GTIM_OC1Cmd(CW_GTIM1, ENABLE);
    GTIM_Cmd(CW_GTIM1, ENABLE);
}

void BSP_PWM_Stop(void) {
    GTIM_Cmd(CW_GTIM1, DISABLE);
    GTIM_OC1Cmd(CW_GTIM1, DISABLE);
}
