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
#include "decode.h"
#include "7_segment.h"
#include "programe.h"

void App_ClkInit(void);
void App_Rch4MHzTo24MHz(void);
void bilingbuling(void);

void delay_main(void)
{
	uint16_t main_i=10000;
	while(main_i--);
}

// Boot: delay then ensure motor stop (BLDC)
static void send_default_param(void)
{
   u8 i;
   for (i = 0; i < 100; i++) {
     delay_main();
   }
   uart_frame_tx(CMD_STATUS, STATUS_MOTOR_STOP);
}

_Bool poweron_send_flag = 0,led1_flag;
extern u8 uart_time_error;
extern u8 STATUS_URGENT_STOP;
u8 main_tesk=0;
u8 down_control=0;
u16 no_key_cnt = 0; //无按键计时

int32_t main(void)
{	
  static u8 w_cnt = 0;		//写入
  static u8 send_cnt = 0; //发送次数
	static _Bool poweron_flag = 1;
	static u8 poweron_cnt = 0;
	
	App_ClkInit();
	App_Rch4MHzTo24MHz();//初始化时钟
	App_PcaInit();//打开定时器和PWM
	GpioInit();
	flash_init();
	uart0_init();
	led_display_init();
	aip1628_init();
	uart_frame_init();
	
  treadmills_init();
  programe_init();
  param_init();
	
	 //上电全亮
  set_seg_val(8888);
  set_dp0();
  set_dp1();       //小数点
  set_dp2();
  set_dp3();
  indecate.led_speed=LED_ON;
  indecate.led_distance=LED_ON;
  indecate.led_calorie=LED_ON;
  indecate.led_time=LED_ON;
	indecate.led_auto=LED_ON;
  indecate.led_updp=LED_ON;
	indecate.led_downdp=LED_ON;
	
  read_beep_status();
	
	poweron_send_flag = 1;
	BEEP_SWITCH_ON_1_CNT();  
	
  while (1)
	{		
	 uart_frame_loop();	
		
   if(ms10_flag)
   {
		 
   	ms10_flag=0;
		KeyPad_Scan();
    TouchKey_Scan();
   	beep_sound_proc();
   }

   //100ms 周期处理程序
   if(ms100_flag)
   {
     ms100_flag=0;
     seg_disp_proc();
     update_display_mode();
     treadmills_disp_loop();
     treadmills_proc_loop();
   }
	
	//250ms 周期处理程序
   if(ms250_flag)
   {
     ms250_flag=0;  
     led1_flag=!led1_flag;
		 LED1(led1_flag);//指示灯闪烁
     communication_checkout();
 
     if(poweron_send_flag == 1)
     {
       send_cnt++;
       if( send_cnt >= 3)
       {
         send_default_param();
         send_cnt = 0;
         poweron_send_flag = 0;              
       }          
     }  
   }

   //500ms 周期处理程序
   if(ms500_flag)
   {
     ms500_flag=0; 
		if(poweron_flag == 1)
    {
			key_mask = 0x00; 
      poweron_cnt++;
      if(poweron_cnt == 2)   				//全亮2秒后显示版本号
      {
				set_bit_seg(0,SEG_U);				//显示U
        set_bit_seg(1,NUM_1);			  //版本号高位
        set_bit_seg(2,NUM_3);			  //版本号低位
        set_bit_seg(3,SEG_ALL_OFF); //强制关闭显示
        indecate.led_speed=LED_OFF;
        indecate.led_distance=LED_OFF;
        indecate.led_calorie=LED_OFF;
        indecate.led_time=LED_OFF;
        indecate.led_updp=LED_OFF;
        set_dp1();      
      }            
					
			if(poweron_cnt == 4)   											  //全亮2秒后显示版本号
      {    
				set_bit_seg(0,SEG_C);								
        set_bit_seg(1,seg_num[down_control / 10]);	//版本号高位
        set_bit_seg(2,seg_num[down_control % 10]);	//版本号低位
        set_bit_seg(3,SEG_ALL_OFF);
        indecate.led_speed=LED_OFF;
        indecate.led_distance=LED_OFF;
        indecate.led_calorie=LED_OFF;
        indecate.led_time=LED_OFF;
        indecate.led_updp=LED_OFF;
        indecate.led_updp=LED_OFF;        
        set_dp1();   
			}
			if(poweron_cnt == 6)   //全亮2秒后显示版本号
			{
				treadmills.status=STATUS_POWERON; 
				poweron_flag = 0;
				poweron_cnt = 0;
			}
		}
    
    if(treadmills.status==STATUS_SET_PARAM || treadmills.status==STATUS_SET_PROGRAM)
    {
      no_key_cnt++;
      if(no_key_cnt == 20)   //待机时间
      {										
        set_seg_mode(SEG_MODE_NORMAL);
        treadmills.status = STATUS_POWERON;
        no_key_cnt=0;
      }
    }
	}

  //1秒周期 处理程序
  if(ms1000_flag)
  {
    ms1000_flag=0;
    treadmills_param_calc();
		uart_time_error++;
#if TM_ERROR_REPORT_ENABLE
		if(uart_time_error>2&&poweron_flag==0)
		{
			treadmills.error_code=ERROR_07;
			treadmills.status=STATUS_ERROR;
		}
#endif
		if(STATUS_URGENT_STOP!=0)
		{
			STATUS_URGENT_STOP++;
			if(STATUS_URGENT_STOP>=4)
			{
				STATUS_URGENT_STOP=0;
				uart_frame_tx(CMD_STOP_BURST,10);
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
	
	main_tesk++;
	if(main_tesk==10)
	{
    main_tesk=0;
    disp_update();
  }
 }
}

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



