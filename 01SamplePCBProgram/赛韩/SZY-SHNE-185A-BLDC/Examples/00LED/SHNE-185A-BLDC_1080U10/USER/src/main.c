/**
 * @file main.c
 * @author HUEY
 * @brief 跑步机仪表主程序入口与系统初始化
 * @version 0.1
 * @date 2026-04-16
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
#include "treadmills.h"
#include "plat_keyfun.h"
#if PLAT_USE_TOUCHKEY
#include "plat_touchkey.h"
#endif

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
    BSP_Timer_Init();
    BSP_Uart_Init();
    BSP_GPIO_Init();
#if PLAT_USE_TOUCHKEY
    Touchkey_Init();
#endif
    LED_Init();

#if 0   /* 全亮测试：TM1640 见下裸写；改 1 编译烧录，测完改回 0 */
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
    /* TM1640 显存 16×8 位裸写 0xFF：不经 UI 段码打包，GRID×SEG 全开。*/
    LED_FillBuf(0xFF);
    LED_Refresh();
#else
    UI_Engine_Init();
    for (uint8_t i = 0; i < UI_GRID_MAX; i++) UI_SetRaw(i, 0xFF);
    UI_Engine_Refresh();
#endif
    while (1);
#endif

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
