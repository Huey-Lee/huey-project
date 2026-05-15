#include "bsp_timer.h"
#include "sys_scheduler.h"
#include "plat_rf.h"
#include "plat_beep.h"


void BSP_Timer_Init(void) 
{
    BTIM_TimeBaseInitTypeDef BTIM_InitStruct = {0};
    __SYSCTRL_BTIM123_CLK_ENABLE();

    // BTIM1: 1ms 系统
    BTIM_InitStruct.BTIM_Mode = BTIM_MODE_TIMER;
    BTIM_InitStruct.BTIM_Prescaler = 48 - 1; 		// 48MHz / 48 = 1MHz (1us/tick)
    BTIM_InitStruct.BTIM_Period = 1000 - 1;         // 1000us = 1ms
    BTIM_TimeBaseInit(CW_BTIM1, &BTIM_InitStruct);
    
    // BTIM2: 60us 433MHz 高频采样
    BTIM_InitStruct.BTIM_Prescaler = 48 - 1; 
    BTIM_InitStruct.BTIM_Period = 60 - 1;           // 60us
    BTIM_TimeBaseInit(CW_BTIM2, &BTIM_InitStruct);

		NVIC_SetPriority(BTIM1_IRQn, 1); // 调度优先级 1
		NVIC_SetPriority(BTIM2_IRQn, 0); // 采样优先级 0 (最高)
  
    NVIC_EnableIRQ(BTIM1_IRQn);
    NVIC_EnableIRQ(BTIM2_IRQn);
	
    BTIM_ITConfig(CW_BTIM1, BTIM_IT_UPDATE, ENABLE);
    BTIM_ITConfig(CW_BTIM2, BTIM_IT_UPDATE, ENABLE);
    
    BTIM_Cmd(CW_BTIM1, ENABLE);
    BTIM_Cmd(CW_BTIM2, ENABLE);
}	//__disable_irq();   __enable_irq()

// 1ms 中断：蜂鸣与调度计数（蜂鸣须在硬实时 1ms 内推进，勿放协作 Task：运行态 UART 发送会拖长主循环，导致 RunMe 堆积、
// Beep_Handler 连续执行把「短嘀」计秒扣光，听感发闷、响不全；设置界面无下发速度故较轻、不易暴露。)
void BTIM1_IRQHandler(void) 
{
    if (BTIM_GetITStatus(CW_BTIM1, BTIM_IT_UPDATE)) 
		{
        Beep_Handler_1ms();
        Scheduler_Update(); // 调用调度器更新
        BTIM_ClearITPendingBit(CW_BTIM1, BTIM_IT_UPDATE);
    }
}

// 60us 中断：处理射频采样（最高优先级，不进调度器直接执行）
void BTIM2_IRQHandler(void) 
{
    if (BTIM_GetITStatus(CW_BTIM2, BTIM_IT_UPDATE)) 
		{
				RF433_Handler_60us();
        BTIM_ClearITPendingBit(CW_BTIM2, BTIM_IT_UPDATE);
    }
}
