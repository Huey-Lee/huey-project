/*
 * 跑步机下位机主循环（CW32L010）：由 HA380A-BDC_110VC02 迁移整理。
 * 头文件路径：MOTOR_BDC 与 USER/inc。
 */

#include "main.h"
#include "cw32l010_systick.h"
#include "bdc_hal.h"
#include "bdc_board.h"
#include "uart_frame.h"
#include "user_usart.h"
#include "user_timer.h"
#include "user_adc.h"
#include "point_fun.h"
#include "user_gpio.h"
#include "motor_drive.h"
#include "motor.h"

u8  waitforamoment   = 0;
static u16 wait_time_start = 0;

static void wait_for_motor(void)
{
    if (waitforamoment == 1) {
        wait_time_start++;
        delay10us(1);
        if (wait_time_start > 20) {
            wait_time_start = 0;
            waitforamoment  = 0;
            ctr.status      = STATUS_TO_RUN;
        }
    }
}

u32 base_time       = 0;
u8  uart_time_error = 0;

static void _500ms_time_handle(void)
{
    base_time++;

    if (base_time % 2u == 0u) {
        BDC_LED_ON();
    } else {
        BDC_LED_OFF();
    }

    if (base_time >= 60000u &&
        (motor.status == NULL || motor.status == STATUS_MT_START)) {
        base_time = 0;
        uart_time_error++;
    } else if (base_time >= 20000u &&
               (motor.status != NULL && motor.status != STATUS_MT_START)) {
        base_time = 0;
        uart_time_error++;
    }

    if (uart_time_error > 5) {
        uart_time_error = 6;
        motor_soft_fault_set(ERROR_01);
    }
}

static void App_ClkInit(void)
{
    SYSCTRL_HSI_Enable(SYSCTRL_HSIOSC_DIV1);
    REGBITS_SET(CW_SYSCTRL->AHBEN, (0x5A5A0000u | (1u << 0)));
    SystemCoreClock = 48000000u;
}

extern tim_t tim_handle;

int32_t main(void)
{
    App_ClkInit();
    bdc_hal_init();
    user_usart_init();
    user_timer_init();
    user_adc_init();
    user_gpio_init();
    MAIN_INIT(100);

    while (1) {
        _500ms_time_handle();

        u16 spd_period = tim6_change_speed_time;

        if (motor.status == STATUS_MT_START) {
            spd_period = MOTOR_SPEED_PERIOD_MT_START_ISR;
        }
        if (spd_period < 1u) {
            spd_period = 1u;
        }

        if (tim_handle.tim6_cur >= spd_period) {
            tim_handle.tim6_cur = 0;
            motor_speed();
            error_chick();
        }

        uart_frame_loop();
        ctr_proc_loop();
        wait_for_motor();
        communication_checkout();
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    while (1) {
    }
}
#endif
