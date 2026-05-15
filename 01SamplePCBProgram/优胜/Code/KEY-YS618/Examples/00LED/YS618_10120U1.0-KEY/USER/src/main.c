/**
 * @file main.c
 * @brief 按键板：触控按键 + 状态机 + AeroLink（无显示、无蜂鸣）
 */
#include "../inc/main.h"
#include "sys_scheduler.h"
#include "bsp_timer.h"
#include "bsp_uart.h"
#include "bsp_gpio.h"
#include "plat_touchkey.h"
#include "plat_gpio_keys.h"
#include "plat_ble.h"
#include "treadmills.h"
#include "plat_hr_tm998.h"

void SYSCTRL_Configuration(void);

int32_t main(void)
{
    SYSCTRL_Configuration();
    BSP_Timer_Init();
    BSP_Uart_Init();
    BSP_Uart2_Init();
    BSP_GPIO_Init();
#if PLAT_USE_TOUCHKEY
    Touchkey_Init();
#endif
#if PLAT_USE_HR_TM998
    HrTm998_Init();
#endif
#if PLAT_USE_GPIO_KEYS
    GpioKeys_Init();
#endif
    Ble_Init();
    Treadmill_Init();

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
