/*
 * main.c
 *
 *  Created on: 2019年11月28日
 *      Author: Administrator
 */
//////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////
#include "common.h"
#include "aip1628.h"
#include "led_disp.h"
#include "indecate.h"
#include "keypad.h"
#include "keypad_fun.h"
#include "pwm_simple.h"
#include "beep.h"
#include "uart.h"
#include "uart_frame.h"
#include "treadmills.h"
#include "param.h"
#include "decode.h"
#include "7_segment.h"


u8 down_control=0;
u16 no_key_cnt = 0; //无按键计时

static bit poweron_send_flag = 0;

extern UINT8  ms10_flag;
extern UINT8  ms100_flag;
extern UINT8  ms250_flag;
extern UINT8  ms500_flag;
extern UINT8  ms1000_flag;
extern void Timer0_Init(void);

void MODIFY_HIRC_166(void)
{
  unsigned char data hircmap0,hircmap1;
  unsigned int trimvalue16bit;
	/* Check if power on reset, modify HIRC */
	if (PCON&SET_BIT4)
	{
    CKDIV = 0x00;
    set_IAPEN;
    IAPAL = 0x30;
    IAPAH = 0x00;
    IAPCN = READ_UID;
    set_IAPGO;
    hircmap0 = IAPFD;
    IAPAL = 0x31;
    IAPAH = 0x00;
    set_IAPGO;
    hircmap1 = IAPFD;
    clr_IAPEN;
    trimvalue16bit = ((hircmap0<<1)+(hircmap1&0x01));
    trimvalue16bit = trimvalue16bit - 15;
    hircmap1 = trimvalue16bit&0x01;
    hircmap0 = trimvalue16bit>>1;
    TA=0XAA;
    TA=0X55;
    RCTRIM0 = hircmap0;
    TA=0XAA;
    TA=0X55;
    RCTRIM1 = hircmap1;


    //CKDIV = 0x50;					//HIRC devider 160
    //set_CLOEN;						//Check HIRC out wavefrom to confirm the HIRC modified
    /* Clear power on flag */
    PCON &= CLR_BIT4;
	}
}


void send_default_param(void)
{
  u8 i=0;
  
  treadmills.param.speed_step = DEFAULT_SPEED_STEP;//50;
  treadmills.param.speed_max = DEFAULT_SPEED_MAX;//600;
  treadmills.param.voltage_max = DEFAULT_VOLTAGE_MAX;//90;
  treadmills.param.voltage_min = DEFAULT_VOLTAGE_MIN;//15;
  treadmills.param.over_current_max = DEFAULT_OVER_CURRENT_MAX;//120;

  treadmills.param.kiv[0] = DEFAULT_SET_KIV_1KM_VALUE;//20;
  treadmills.param.kiv[1] = DEFAULT_SET_KIV_2KM_VALUE;//20;
  treadmills.param.kiv[2] = DEFAULT_SET_KIV_3KM_VALUE;//15;
  treadmills.param.kiv[3] = DEFAULT_SET_KIV_4KM_VALUE;//10;
  treadmills.param.kiv[4] = DEFAULT_SET_KIV_5KM_VALUE;//8;
  treadmills.param.kiv[5] = DEFAULT_SET_KIV_6KM_VALUE;//8;
  treadmills.param.kiv[6] = DEFAULT_SET_KIV_7KM_VALUE;//6;
  treadmills.param.kiv[7] = DEFAULT_SET_KIV_8KM_VALUE;//6;
  treadmills.param.kiv[8] = DEFAULT_SET_KIV_9KM_VALUE;//6;
  treadmills.param.kiv[9] = DEFAULT_SET_KIV_10KM_VALUE;//6;
  treadmills.param.kiv[10] = DEFAULT_SET_KIV_11KM_VALUE;//6;
  treadmills.param.kiv[11] = DEFAULT_SET_KIV_12KM_VALUE;//6;
  
  //****************上电就发送*************  
  for(i = 0; i< 100;i++)
  {
    NOP_15();
  }

  uart1_frame_tx(CMD_TREADMILLS_SPEED_MAX,treadmills.param.speed_max/10);  //后面的设置参数发送的时候会把前面的一同发送给下控
  uart1_frame_tx(CMD_VOLTAGE_MAX,treadmills.param.voltage_max);
  uart1_frame_tx(CMD_VOLTAGE_MIN,treadmills.param.voltage_min); 
  uart1_frame_tx(CMD_OVER_CURRENT_MAX,treadmills.param.over_current_max);  
	uart1_frame_tx(CMD_OVER_VALTAGE_MAX,240);
  uart1_frame_tx(CMD_STA_LEVEL,1);	//value chose betwen 1~5 , the higher the level , the faster the starting speed
	uart1_frame_tx(CMD_END_LEVEL,13);	//VALUE CHOSE BETWEN 13~25 , the higher the level , the faster the ending   speed
	uart1_frame_tx(CMD_Accelerated_LEVEL,5);	//value chose betwen 1~6 , the higher the value , the faster the acceleration
	uart1_frame_tx(CMD_deceleration_LEVEL,5);//VALUE CHOSE BETWEN 1~6 , the higher the value , the faster the deceleration

  for( i = 16; i < 28; i++)        //16 = CMD_KIV_1KM， 27 = CMD_KIV_12KM;
  {
    uart1_frame_tx(i ,treadmills.param.kiv[i-16]); 
  }
}

extern u8 uart_time_error;
bit poweron_flag = 1;	
/************************************************************************************************************
*    Main function
************************************************************************************************************/
void main (void)
{
  u8 i = 0;
  static u8 send_cnt = 0;
	static u8 poweron_cnt = 0;
  
	NOP_15();
	MODIFY_HIRC_166();
	Set_All_GPIO_Quasi_Mode;

	/*
	 * 准双向:     PxM1=0 PxM2=0
	 * 推挽:       PxM1=0 PxM2=1
	 * 高阻抗输入:  PxM1=1 PxM2=0
	 * 开漏:       PxM1=1 PxM2=1
	 *
	 */
	P0M1 =(1<<0) |(1<<1) |(0<<2) |(1<<3) |(1<<4) |(0<<5) |(0<<6) |(0<<7);
	P0M2 =(0<<0) |(0<<1) |(0<<2) |(0<<3) |(0<<4) |(1<<5) |(0<<6) |(0<<7);

	P1M1 =(1<<0) |(0<<1) |(0<<2) |(0<<3) |(0<<4) |(0<<5) |(0<<6) |(1<<7);
	P1M2 =(0<<0) |(0<<1) |(0<<2) |(0<<3) |(1<<4) |(0<<5) |(0<<6) |(0<<7);

	P3M1 =(1<<0);
	P3M2 =(0<<0);
  
	key_mask=0x00;//禁止所有按键
 
  RF433M_Init();          //433模块引脚初始化 
	led_display_init();
	aip1628_init();
	pwm_init();
	uart_init();
	Timer0_Init();
  uart_frame_init();

  treadmills_init();
  param_init();
  ee_param_init();
  
//  treadmills.status=STATUS_POWERON;  
  poweron_send_flag = 1;
	set_EA;       //开总中断

	beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);


  while(1)
  {
    uart1_frame_loop();

    //10ms 周期处理程序
    if(ms10_flag)
    {
      ms10_flag=0;
      KeyPad_Scan();
      beep_sound_proc();
      boot_time();
    }
    
     //100ms 周期处理程序
      if(ms100_flag)
      {
        ms100_flag=0;
        seg_disp_proc();
        treadmills_disp_loop();
        treadmills_proc_loop();
      }
      
      //250ms 周期处理程序
      if(ms250_flag)
      {
        ms250_flag=0;  
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
            if(poweron_cnt == 1)   											//全亮2秒后显示版本号
            {
              set_bit_seg(0,SEG_U);								//显示U
              set_bit_seg(1,NUM_1);			//版本号高位
              set_bit_seg(2,NUM_0);			//版本号低位
              set_bit_seg(3,SEG_ALL_OFF);								//强制关闭显示
              set_dp1();       					 //decimal point     
            }            
					
						if(poweron_cnt == 3)   											//全亮2秒后显示版本号
            {
              set_bit_seg(0,SEG_C);								
              set_bit_seg(1,seg_num[down_control / 10]);			//版本号高位
              set_bit_seg(2,seg_num[down_control % 10]);			//版本号低位
              set_bit_seg(3,SEG_ALL_OFF);								//强制关闭显示
              set_dp1();       					 //decimal point
            }
            
            if(poweron_cnt == 5)   //全亮2秒后显示版本号
            {
							 //上电全亮
							 treadmills.status=STATUS_FULL_DISPLAY;             
            }
						if(poweron_cnt == 7)   //全亮2秒后显示版本号
            {
							treadmills.status=STATUS_POWERON; 
              poweron_flag = 0;
							poweron_cnt = 0;
						}
          }
          if(poweron_flag == 0)
            {
              key_mask=0xff;
            }
             
          if(treadmills.status!=STATUS_RUNNING)
             {
								no_key_cnt++;
                if(no_key_cnt == 1200)   //待机时间
                  {										
										set_seg_mode(SEG_MODE_NORMAL);
                    treadmills.status = STATUS_SLEEP_MODE;
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
				if(uart_time_error>5&&poweron_flag==0)
					{
						treadmills.error_code=ERROR_06;
						treadmills.status=STATUS_ERROR;
					}
      }
    // led update
    disp_update();
  }
}
