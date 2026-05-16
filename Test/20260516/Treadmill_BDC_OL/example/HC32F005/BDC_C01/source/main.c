/******************************************************************************
* Copyright (C) 2017, Xiaohua Semiconductor Co.,Ltd All rights reserved.
* (版权声明保留，略)
******************************************************************************/
/* main.c  MCU 入口：外设初始化、主循环、通信与电机调度 */

#include "ddl.h"
#include "uart.h"
#include "bt.h"
#include "gpio.h"
#include "user_usart.h"
#include "user_timer.h"
#include "user_adc.h"
#include "point_fun.h"
#include "user_gpio.h"
#include "uart_frame.h"
#include "motor_drive.h"
#include "motor.h"

/* ── 等待继电器稳定后再启动电机 ──────────────────────────────────────────
 * STATUS_RUN 检测通过后，上控设置 waitforamoment=1。
 * wait_for_motor() 在主循环中计数约 20 次（约数百微秒），
 * 确保 ADC 采样稳定后，才允许进入 STATUS_TO_RUN（合继电器+开 PWM）。
 * 注意：这里的延时很短，主延时（500ms）在 STATUS_TO_RUN 内部完成。*/
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
            ctr.status      = STATUS_TO_RUN;  /* 进入合继电器流程 */
        }
    }
}

/* ── 主循环帧率计数器与通信超时检测 ────────────────────────────────────────
 * 注意：此函数在每次主循环迭代时调用，并非固定 500ms 间隔。
 * base_time 的增速取决于主循环实际执行速度（含各子函数耗时）。
 * 60000/20000 阈值需与实际循环速度匹配，否则通信超时误报。
 *
 * 通信超时规则：
 *   - 电机静止或起步阶段：base_time 达到 60000 次后累计 1 次超时
 *   - 电机运行阶段：base_time 达到 20000 次后累计 1 次超时
 *   - 超时累计 > 5 次：上报 E01（通信中断），触发受控减速停机
 * base_time 在收到任意有效 UART 帧时被清零（uart_frame_loop → cmd_proc 内隐含重置）。*/
u32 base_time       = 0;
u8  uart_time_error = 0;

void _500ms_time_handle(void)
{
    base_time++;

    /* 心跳 LED：每 2 次主循环翻转一次（用于观察主循环是否卡死） */
    if (base_time % 2 == 0) port_on;
    else                    port_off;

    /* 通信超时检测：超时阈值按电机运行状态区分 */
    if (base_time >= 60000u &&
        (motor.status == MOTOR_CTRL_STATUS_CLEARED || motor.status == STATUS_MT_START)) {
        base_time = 0;
        uart_time_error++;
    } else if (base_time >= 20000u &&
               (motor.status != MOTOR_CTRL_STATUS_CLEARED && motor.status != STATUS_MT_START)) {
        base_time = 0;
        uart_time_error++;
    }

    /* 超时 5 次触发 E01：执行受控减速停机（不立即断电，保证用户安全） */
    if (uart_time_error > 5) {
        uart_time_error = 6;           /* 上限钳位，防止重复触发 */
        motor_soft_fault_set(ERROR_01);
    }
}

/* ── GPIO 初始化 ──────────────────────────────────────────────────────────
 * P24/P26/P32：ADC 模拟输入（电流 CH0、母线电压 CH1/CH2）
 * STK_LED：状态指示灯（心跳/故障闪烁） */
static void GpioInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    Gpio_SetAnalogMode(GpioPort2, GpioPin4);  /* CH0：电流采样 */
    Gpio_SetAnalogMode(GpioPort2, GpioPin6);  /* CH1：M− 母线电压 */
    Gpio_SetAnalogMode(GpioPort3, GpioPin2);  /* CH2：M+ 母线电压 */

    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enPu  = GpioPuDisable;
    stcGpioCfg.enPd  = GpioPdEnable;
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &stcGpioCfg);
}

extern tim_t tim_handle;
extern volatile uint16_t ZHANKONBI;

int32_t main(void)
{
    /* ── 外设初始化 ──────────────────────────────────────────────────────── */
    App_ClkInit();      /* 系统时钟初始化（24 MHz PCLK） */
    GpioInit();         /* ADC 引脚 + LED 引脚 */
    user_usart_init();  /* UART1：与上控 2400 baud 通信 */
    user_timer_init();  /* TIM0 继电器 / TIM2 ADC / TIM6 PWM（PER 见 motor.h）*/
    user_adc_init();    /* ADC 扫描模式初始化（CH1/CH2 母线电压） */
    user_gpio_init();   /* 其他 GPIO（继电器控制等） */
    MAIN_INIT(100);     /* 控制器 + 电机参数默认值初始化（ctr_init + 通信初始化） */

    /* ── 主循环 ──────────────────────────────────────────────────────────── */
    while (1) {
        /* 1. 心跳 + 通信超时检测（每次循环执行） */
        _500ms_time_handle();

        /* 2. 速度斜坡 + 保护检测
         *    tim6_cur：UDF ISR 计数（≈ MOTOR_TIM6_ISR_HZ_APPROX）；MT_START 周期≈100 ms；斜坡 tim6_change_speed_time */
        u16 spd_period = tim6_change_speed_time;

        /* MT_START：约 100 ms；RUN/STOP：tim6_change_speed_time（按 MOTOR_TIM6_ISR_HZ_APPROX 自动换算墙钟） */
        if (motor.status == STATUS_MT_START)
            spd_period = MOTOR_SPEED_PERIOD_MT_START_ISR;

        if (spd_period < 1u)
            spd_period = 1u;

        if (tim_handle.tim6_cur >= spd_period) {
            tim_handle.tim6_cur = 0;
            motor_speed();   /* 速度斜坡步进、状态切换（START/PID/STOP） */
            error_chick();   /* 过流、过压、过速保护检测 */
        }

        /* 3. UART 帧解析（每次循环从队列取一个字节，状态机解帧） */
        uart_frame_loop();

        /* 4. 主控状态机（继电器开合、PWM 启停、故障处理） */
        ctr_proc_loop();

        /* 5. 等待继电器稳定后跳转到 STATUS_TO_RUN */
        wait_for_motor();

        /* 6. 主动上报 ACK 或故障码给上控（约每轮循环检查一次） */
        communication_checkout();
    }
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
