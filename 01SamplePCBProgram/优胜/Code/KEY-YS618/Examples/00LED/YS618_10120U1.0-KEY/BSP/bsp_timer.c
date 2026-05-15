#include "plat_hr_tm998.h"
#include "bsp_timer.h"
#include "sys_scheduler.h"

void BSP_Timer_Init(void)
{
    BTIM_TimeBaseInitTypeDef BTIM_InitStruct = {0};
    __SYSCTRL_BTIM123_CLK_ENABLE();

    BTIM_InitStruct.BTIM_Mode = BTIM_MODE_TIMER;
    BTIM_InitStruct.BTIM_Prescaler = 48 - 1;
    BTIM_InitStruct.BTIM_Period = 1000 - 1;
    BTIM_TimeBaseInit(CW_BTIM1, &BTIM_InitStruct);

    NVIC_SetPriority(BTIM1_IRQn, 1);
    NVIC_EnableIRQ(BTIM1_IRQn);

    BTIM_ITConfig(CW_BTIM1, BTIM_IT_UPDATE, ENABLE);
    BTIM_Cmd(CW_BTIM1, ENABLE);
}

void BTIM1_IRQHandler(void)
{
    if (BTIM_GetITStatus(CW_BTIM1, BTIM_IT_UPDATE)) {
        HrTm998_MsTick_1ms();
        Scheduler_Update();
        BTIM_ClearITPendingBit(CW_BTIM1, BTIM_IT_UPDATE);
    }
}

void BTIM2_IRQHandler(void)
{
}
