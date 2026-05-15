#include "bsp_timer.h"
#include "sys_scheduler.h"
#include "plat_rf.h"


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

// 1ms 中断：驱动调度器计数
void BTIM1_IRQHandler(void) 
{
    if (BTIM_GetITStatus(CW_BTIM1, BTIM_IT_UPDATE)) 
	{
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
//		RF433M_RecevieDecode();  
        BTIM_ClearITPendingBit(CW_BTIM2, BTIM_IT_UPDATE);
    }
}
