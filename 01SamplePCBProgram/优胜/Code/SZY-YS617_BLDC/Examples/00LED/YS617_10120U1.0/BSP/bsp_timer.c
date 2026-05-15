#include "bsp_timer.h"
#include "sys_scheduler.h"
#include "plat_keyfun.h"
#if PLAT_USE_RF433
#include "plat_rf.h"
#endif

void BSP_Timer_Init(void)
{
    BTIM_TimeBaseInitTypeDef BTIM_InitStruct = {0};
    __SYSCTRL_BTIM123_CLK_ENABLE();

    BTIM_InitStruct.BTIM_Mode = BTIM_MODE_TIMER;
    BTIM_InitStruct.BTIM_Prescaler = 48 - 1;
    BTIM_InitStruct.BTIM_Period = 1000 - 1;
    BTIM_TimeBaseInit(CW_BTIM1, &BTIM_InitStruct);

#if PLAT_USE_RF433
    BTIM_InitStruct.BTIM_Prescaler = 48 - 1;
    BTIM_InitStruct.BTIM_Period = 60 - 1;
    BTIM_TimeBaseInit(CW_BTIM2, &BTIM_InitStruct);
#endif

    NVIC_SetPriority(BTIM1_IRQn, 1);
#if PLAT_USE_RF433
    NVIC_SetPriority(BTIM2_IRQn, 0);
#endif

    NVIC_EnableIRQ(BTIM1_IRQn);
#if PLAT_USE_RF433
    NVIC_EnableIRQ(BTIM2_IRQn);
#endif

    BTIM_ITConfig(CW_BTIM1, BTIM_IT_UPDATE, ENABLE);
#if PLAT_USE_RF433
    BTIM_ITConfig(CW_BTIM2, BTIM_IT_UPDATE, ENABLE);
#endif

    BTIM_Cmd(CW_BTIM1, ENABLE);
#if PLAT_USE_RF433
    BTIM_Cmd(CW_BTIM2, ENABLE);
#endif
}

void BTIM1_IRQHandler(void)
{
    if (BTIM_GetITStatus(CW_BTIM1, BTIM_IT_UPDATE)) {
        Scheduler_Update();
        BTIM_ClearITPendingBit(CW_BTIM1, BTIM_IT_UPDATE);
    }
}

#if PLAT_USE_RF433
void BTIM2_IRQHandler(void)
{
    if (BTIM_GetITStatus(CW_BTIM2, BTIM_IT_UPDATE)) {
        RF433_Handler_60us();
        BTIM_ClearITPendingBit(CW_BTIM2, BTIM_IT_UPDATE);
    }
}
#else
void BTIM2_IRQHandler(void)
{
}
#endif
