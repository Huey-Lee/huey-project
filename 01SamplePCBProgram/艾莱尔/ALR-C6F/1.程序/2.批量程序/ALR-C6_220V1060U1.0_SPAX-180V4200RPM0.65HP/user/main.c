/*
 * main.c
 *
 *  Created on: 2019��11��28��
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
u16 no_key_cnt = 0; //�ް�����ʱ

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
  treadmills.param.speed_step = DEFAULT_SPEED_STEP;//10;
  treadmills.param.speed_max = DEFAULT_SPEED_MAX;//800;
  treadmills.param.voltage_max = DEFAULT_VOLTAGE_MAX;//174;
  treadmills.param.voltage_min = DEFAULT_VOLTAGE_MIN;//43;
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
	 * ׼˫��:     PxM1=0 PxM2=0
	 * ����:       PxM1=0 PxM2=1
	 * ���迹����:  PxM1=1 PxM2=0
	 * ��©:       PxM1=1 PxM2=1
	 *
	 */
	P0M1 =(1<<0) |(1<<1) |(0<<2) |(1<<3) |(1<<4) |(0<<5) |(0<<6) |(1<<7);
	P0M2 =(0<<0) |(0<<1) |(0<<2) |(0<<3) |(0<<4) |(1<<5) |(0<<6) |(0<<7);

	P1M1 =(1<<0) |(0<<1) |(0<<2) |(0<<3) |(0<<4) |(0<<5) |(0<<6) |(1<<7);
	P1M2 =(0<<0) |(0<<1) |(0<<2) |(0<<3) |(1<<4) |(0<<5) |(0<<6) |(0<<7);

	P3M1 =(1<<0);
	P3M2 =(0<<0);
  
	key_mask=0x00;//��ֹ���а���
 
  RF433M_Init();          //433ģ�����ų�ʼ�� 
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
	set_EA;       //�����ж�

	beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);


  while(1)
  {
    uart0_frame_loop();

    uart1_frame_loop();    

    //10ms ���ڴ�������
    if(ms10_flag)
    {				
      ms10_flag=0;
      KeyPad_Scan();
      beep_sound_proc();
      boot_time();
    }
    
     //100ms ���ڴ�������
      if(ms100_flag)
      {
        ms100_flag=0;
        seg_disp_proc();
        treadmills_disp_loop();
        treadmills_proc_loop();
        bt_power_send();
        aerolink_ctrl_loop();  /* AeroLink����+��ͣ����ɿ��ط� */
      }
      
      //250ms ���ڴ�������
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
      
      //500ms ���ڴ�������
      if(ms500_flag)
      {
        ms500_flag=0;
				send_motion_data_by_status();
				if(poweron_flag == 1)
          {
            key_mask = 0x00; 
            poweron_cnt++;
            if(poweron_cnt == 1)   											//ȫ��2�����ʾ�汾��
            {
              set_bit_seg(0,SEG_U);								//��ʾU
              set_bit_seg(1,NUM_1);			//�汾�Ÿ�λ
              set_bit_seg(2,NUM_0);			//�汾�ŵ�λ
              set_bit_seg(3,SEG_ALL_OFF);								//ǿ�ƹر���ʾ
              set_dp1();       					 //decimal point     
            }            
					
						if(poweron_cnt == 3)   											//ȫ��2�����ʾ�汾��
            {
              set_bit_seg(0,SEG_C);								
              set_bit_seg(1,seg_num[down_control / 10]);			//�汾�Ÿ�λ
              set_bit_seg(2,seg_num[down_control % 10]);			//�汾�ŵ�λ
              set_bit_seg(3,SEG_ALL_OFF);								//ǿ�ƹر���ʾ
              set_dp1();       					 //decimal point
            }
            if(poweron_cnt == 5)   //ȫ��2�����ʾ�汾��
            {
							 //�ϵ�ȫ��
							 treadmills.status=STATUS_FULL_DISPLAY;             
            }
						if(poweron_cnt == 7)   //ȫ��2�����ʾ�汾��
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
                if(no_key_cnt == 1200)   //����ʱ��1200
                  {										
										set_seg_mode(SEG_MODE_NORMAL);
                    treadmills.status = STATUS_SLEEP_MODE;
                    no_key_cnt=0;
                  }
              }
				}

      //1������ ��������
      if(ms1000_flag)
      {
        ms1000_flag=0;
        treadmills_param_calc();

        uart_time_error = 0;






      }
    // led update
    disp_update();
  }
}
