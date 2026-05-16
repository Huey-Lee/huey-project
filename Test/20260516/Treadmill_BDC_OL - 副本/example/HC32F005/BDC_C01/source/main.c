/*
 * Function: MCU 主入口与主循环：外设初始化并调度电机、通信与保护。
 * Method:   顺序初始化时钟、GPIO、UART、定时器、ADC、板级 GPIO 后调用 ctr_init；while(1) 轮询通信超时、速度斜坡、UART 帧解析、控制器状态机与上发。
 * Name:     Huey
 * Date:     May 16, 2026 18:00
 */

#include "ddl.h"
#include "uart.h"
#include "bt.h"
#include "gpio.h"
#include "user_usart.h"
#include "user_timer.h"
#include "user_adc.h"
#include "user_gpio.h"
#include "uart_frame.h"
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

void _500ms_time_handle(void)
{
    base_time++;

    if (base_time % 2 == 0) port_on;
    else                    port_off;

    if (base_time >= 60000u &&
        (motor.status == MOTOR_CTRL_STATUS_CLEARED || motor.status == STATUS_MT_START)) {
        base_time = 0;
        uart_time_error++;
    } else if (base_time >= 20000u &&
               (motor.status != MOTOR_CTRL_STATUS_CLEARED && motor.status != STATUS_MT_START)) {
        base_time = 0;
        uart_time_error++;
    }

    if (uart_time_error > 5) {
        uart_time_error = 6;
        motor_soft_fault_set(ERROR_01);
    }
}// 不是的继电器不启动的时候接电机量M+M-是0V,不接是320V

static void GpioInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    Gpio_SetAnalogMode(GpioPort2, GpioPin4);
    Gpio_SetAnalogMode(GpioPort2, GpioPin6);
    Gpio_SetAnalogMode(GpioPort3, GpioPin2);

    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enPu  = GpioPuDisable;
    stcGpioCfg.enPd  = GpioPdEnable;
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &stcGpioCfg);
}

extern tim_t tim_handle;
extern volatile uint16_t ZHANKONBI;

int32_t main(void)
{
    App_ClkInit();
    GpioInit();
    user_usart_init();
    user_timer_init();
    user_adc_init();
    user_gpio_init();
    ctr_init();

    while (1) {
        _500ms_time_handle();

        u16 spd_period = tim6_change_speed_time;

        if (motor.status == STATUS_MT_START)
            spd_period = MOTOR_SPEED_PERIOD_MT_START_ISR;

        if (spd_period < 1u)
            spd_period = 1u;

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
