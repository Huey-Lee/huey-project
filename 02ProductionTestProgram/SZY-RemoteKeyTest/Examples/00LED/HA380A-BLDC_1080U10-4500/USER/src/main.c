/**
 * @file main.c
 * @brief HA380A 演示：433 遥控 + TM1640 显示一位数字（无跑机逻辑）
 */
#include "../inc/main.h"
#include "sys_scheduler.h"
#include "bsp_timer.h"
#include "bsp_uart.h"
#include "bsp_gpio.h"
#include "plat_beep.h"
#include "ui_display.h"
#include "app_remote_demo.h"

void SYSCTRL_Configuration(void);

int32_t main(void)
{
    SYSCTRL_Configuration();
    BSP_Timer_Init();
    BSP_Uart_Init();
    BSP_GPIO_Init();
    Beep_Init();
    UI_Engine_Init();
    App_RemoteDemo_Init();

    while (1) {
        Scheduler_Dispatch();
    }
}

void SYSCTRL_Configuration(void)
{
    SYSCTRL_HSI_Enable(SYSCTRL_HSIOSC_DIV1);
    REGBITS_SET(CW_SYSCTRL->AHBEN, (0x5A5A0000 | bv1));
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
