#include "common.h"
#include "my_gpio.h"
#include "my_pwm.h"
#include "my_flash.h"
#include "aip1628.h"
#include "led_disp.h"
#include "indecate.h"
#include "beep.h"
#include "uart_frame.h"
#include "treadmills.h"
#include "param.h"
#include "keypad.h"
#include "keypad_fun.h"
#include "7_segment.h"
#include "decode.h"

static uint8_t version_shown = 0; // 记录版本号是否已经显示

//时钟初始化配置
void App_ClkInit(void)
{
    stc_sysctrl_clk_cfg_t stcCfg;

    ///< 开启FLASH外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralFlash, TRUE);

    ///<========================== 时钟初始化配置 ===================================
    ///< 因要使用的时钟源HCLK小于24M：此处设置FLASH 读等待周期为0 cycle(默认值也为0 cycle)
    Flash_WaitCycle(FlashWaitCycle0);

    ///< 时钟初始化前，优先设置要使用的时钟源：此处设置RCH为4MHz
    Sysctrl_SetRCHTrim(SysctrlRchFreq4MHz);

    ///< 选择内部RCH作为HCLK时钟源;
    stcCfg.enClkSrc    = SysctrlClkRCH;
    ///< HCLK SYSCLK/1
    stcCfg.enHClkDiv   = SysctrlHclkDiv1;
    ///< PCLK 为HCLK/1
    stcCfg.enPClkDiv   = SysctrlPclkDiv1;
    ///< 系统时钟初始化
    Sysctrl_ClkInit(&stcCfg);
}


//将时钟从RCH4MHz切换至RCH24MHz,
void App_Rch4MHzTo24MHz(void)
{
///<============== 将时钟从RCH4MHz切换至RCH24MHz ==============================
    ///< RCH时钟不同频率的切换，需要先将时钟切换到RCL，设置好频率后再切回RCH
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
    Sysctrl_SysClkSwitch(SysctrlClkRCL);

    ///< 加载目标频率的RCH的TRIM值
    Sysctrl_SetRCHTrim(SysctrlRchFreq24MHz);
    ///< 使能RCH（默认打开，此处可不需要再次打开）
    //Sysctrl_ClkSourceEnable(SysctrlClkRCH, TRUE);
    ///< 时钟切换到RCH
    Sysctrl_SysClkSwitch(SysctrlClkRCH);
    ///< 关闭RCL时钟
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, FALSE);

    ///< 禁止HCLK从PA01输出
    Gpio_SfHClkOutputCfg(GpioSfHclkOutDisable, GpioSfHclkOutDiv1);
}

_Bool poweron_send_flag = 0,led1_flag;
extern u8 uart_time_error;
uint8_t main_tesk=0;
u16 no_key_cnt = 0; //无按键计时
u8 down_control=0;

static u8 w_cnt = 0;		//写入
static u8 send_cnt = 0; //发送次数
static _Bool poweron_flag = 1;
static u8 poweron_cnt = 0;


/* 上电自检/显示版本流程 */ 
void poweron_proc(void)
{
    if(poweron_flag == 0) return;

    key_mask = 0x00; // 屏蔽按键
    poweron_cnt++;

    switch(poweron_cnt)
    {
			case 1: // 显示上控版本号 U1.0
            set_bit_seg(0, SEG_U);
            set_bit_seg(1, NUM_1);
            set_bit_seg(2, NUM_0);
            set_bit_seg(3, SEG_ALL_OFF);
            set_dp1();
            break;

        case 3: // 显示下控版本号 Cxx
            set_bit_seg(0, SEG_C);
            set_bit_seg(1, seg_num[down_control / 10]);
            set_bit_seg(2, seg_num[down_control % 10]);
            set_bit_seg(3, SEG_ALL_OFF);
            set_dp1();
            break;

        case 5: // 全亮
//            aip1628_all_on();
            break;

        case 7: // 结束上电流程
            treadmills.status = STATUS_POWERON;
            poweron_flag = 0;
            poweron_cnt = 0;
            key_mask = 0xFF; // 恢复按键
						version_shown = 1; // 标记版本号已显示
            break;
    }
}

/* 10ms周期任务 */
void task_10ms(void)
{
    KeyPad_Scan();
    beep_sound_proc();
    boot_time();
}

/* 100ms周期任务 */
void task_100ms(void)
{
    seg_disp_proc();
    treadmills_disp_loop();
    treadmills_proc_loop();
}

/* 250ms周期任务 */
void task_250ms(void) 
{
    if (treadmills.status == STATUS_ERROR) return;

    // 1. 启停闭环（最高优先级）
    if (treadmills.status == STATUS_STOP) {
        if (treadmills.lower_status != 10) {
            uart_frame_tx(0x02, 0x01, 0x00); 
        } else {
            treadmills.status = STATUS_WAIT;
        }
        return; // 只要在启停逻辑，不要去跑下面的速度同步，防止干扰
    }
    
    if (treadmills.status == STATUS_RUN) {
        if (treadmills.lower_status != 9) {
            uart_frame_tx(0x02, 0x00, 0x00); 
        } 
		}
		
		if(treadmills.status == STATUS_RUNNING) {
        if (treadmills.param.speed != treadmills.param.fbk_speed) {
            uart_frame_tx_d1(0x03, 0x00, treadmills.param.speed);
        }
        return;
    }
}


void task_300ms(void)
{	
	if(poweron_send_flag == 1)
	{
		send_cnt++;
		if(send_cnt >= 3)
		{
			send_cnt = 0;
			poweron_send_flag = 0;
		}
	}
}

/* 500ms周期任务 */
void task_500ms(void)
{
//	poweron_proc();
	if (treadmills.status != STATUS_ERROR) {
		uart_frame_tx(0x01, 0x00, 0x00); /* 心跳：周期性查询下位机状态 */
	}
	
//	if(treadmills.status != STATUS_RUNNING)
//	{
//		no_key_cnt++;
//		if((no_key_cnt >= 1200)&&(PAUSE_FLAG == 0)) // 10分钟待机
//		{
//			treadmills.status = STATUS_SLEEP_MODE;
//		}
//		if(no_key_cnt >= 3600&&(PAUSE_FLAG == 1))		//  10分钟休眠
//		{
//			treadmills.param.speed=0;
//			treadmills.param.second=0;
//			treadmills.param.distance=0;
//			treadmills.param.calorie=0; 
//			treadmills.status = STATUS_SLEEP_MODE;
//			no_key_cnt = 0;
//		}
//	}
}

/* 1000ms周期任务 */
void task_1000ms(void)
{
   treadmills_param_calc();
	
    /* 通信超时：约 3s 无接收则报通信故障（与心跳配合） */
	if (treadmills.status != STATUS_ERROR) {
		if (uart_time_error < 3) {
			uart_time_error++;
		} else {
			/* error_code 为下位机位图；通信异常须用 error_sub_code bit0 → E14（串口丢失） */
			treadmills.error_code = 0;
			treadmills.error_sub_code = (u8)(1u << 0);
			treadmills.status = STATUS_ERROR;
		}
	}

  if(write_flag)
  {
		if(++w_cnt >= 2)
		{
			save_param();
			w_cnt = 0;
			write_flag = 0;
		}
	}
	else
	{
		w_cnt = 0;
	}
}

/* 主函数 */
int32_t main(void)
{
    App_ClkInit();
    App_Rch4MHzTo24MHz();
    App_PcaInit();
    GpioInit();
    flash_init();
    uart0_init();
    led_display_init();
    aip1628_init();
    uart_frame_init();
	RF433M_Init(); 
    treadmills_init();
    param_init();
    read_beep_status();

    poweron_send_flag = 1;
	treadmills.status = STATUS_POWERON;
    BEEP_SWITCH_ON_1_CNT();
	

    while(1)
    {
        uart_frame_loop(); // 串口任务

        if(ms10_flag)   { ms10_flag = 0; task_10ms(); }
        if(ms100_flag)  { ms100_flag = 0; task_100ms(); }
        if(ms250_flag)  { ms250_flag = 0; task_250ms(); }
		if(ms300_flag)  { ms300_flag = 0; task_300ms(); }
        if(ms500_flag)  { ms500_flag = 0; task_500ms(); }
        if(ms1000_flag) { ms1000_flag = 0; task_1000ms(); }

        main_tesk++;
        if(main_tesk >= 10)
        {
            main_tesk = 0;
            disp_update();
        }
    }
}

