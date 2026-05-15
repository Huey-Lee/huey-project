/**
 * @file main.c
 * @author WHXY
 * @brief 跑步机仪表主程序入口与系统初始化
 * @version 0.1
 * @date 2024-08-07
 *
 * @copyright Copyright (c) 2021
 */
/******************************************************************************
 * 头文件包含
 ******************************************************************************/
#include "../inc/main.h"
#include "sys_scheduler.h"
#include "bsp_timer.h"
#include "bsp_uart.h"
#include "bsp_gpio.h"
#include "led_driver.h"
#include "plat_beep.h"
#include "ui_display.h"
#include "plat_segment.h"
#include "plat_rf.h"
#include "plat_aerolink.h"
#include "plat_rf_store.h"
#include "treadmills.h"

/******************************************************************************
 * 本地函数声明（static）
 ******************************************************************************/
void SYSCTRL_Configuration(void);

/*****************************************************************************
 * 函数实现：全局（extern）与本地（static）
 ******************************************************************************/
int32_t main(void)
{
    SYSCTRL_Configuration();
    RFStore_Init();
    BSP_Timer_Init();
    BSP_Uart_Init();
    BSP_GPIO_Init();
    LED_Init();
    Beep_Init();
    UI_Engine_Init();
    Treadmill_Init();

    while (1) {
        Scheduler_Dispatch();
    }
}

void SYSCTRL_Configuration(void)
{
    SYSCTRL_HSI_Enable(SYSCTRL_HSIOSC_DIV1); /* 48MHz */
    REGBITS_SET(CW_SYSCTRL->AHBEN, (0x5A5A0000 | bv1)); /* flash */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
