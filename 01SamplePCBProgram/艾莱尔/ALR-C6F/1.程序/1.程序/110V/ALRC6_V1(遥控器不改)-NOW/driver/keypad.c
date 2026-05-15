/*
 * keypad.c
 *
 *  Created on: 2018年2月4日
 *      Author: Administrator
 */
#include "keypad.h"
#include "keypad_fun.h"
#include "led_disp.h"
#include "decode.h"
#include "treadmills.h"
#include "beep.h"


static u8 trig=0;
static u8 cont=0;

static u8 cnt=0,state=0;
static u8 new ,old;

u8 FOHA_cont_cnt=0;

u8 times=0;
u8 key_valuq=0;
u8 key=0;
u8 key_long_flag=0;
u8 key_cnt=0x00;
u8 key_mask=0x00;
u8 delay_add=0x00;
u8 tl_flag=0;
u8 key_continue=0;
extern bit poweron_flag;
u16 key_delay_dede=0;
extern u16 no_key_cnt;

void KeyPad_Scan(void)
{
   /**
   * 按键输入入口,和硬件有关系
   */
  //=================================
	new=0;
  
  if(key_value!=key_valuq)
	{
		times++;
		if(times>15)
		{
			key_valuq = key_value;
			times=0;
		}
		if(key_long_flag==0)
		{
			
			if(key_continue>=7)
			{
			key_valuq = key_value;
				key_continue=0;
			}
			else if(key_continue<=7)
			key_continue++;
		}
	}
	else if(key_value == key_valuq)
	{
		times=0;
			if(key_continue<=7)
			key_continue++;
	}
  
  //客户433键值
	if(key_valuq==0X04){new |=(1<<0);}  //+			//0X04
	if(key_valuq==0X1A){new |=(1<<1);}  //ON/OFF  
	if(key_valuq==0X01){new |=(1<<2);}  //M			
	if(key_valuq==0X05){new |=(1<<3);}  //-
	
	
		if(key_delay_dede)
		{
			key_delay_dede--;
			return;
		}
	
	if(new!=0)//按键有效
	{		
   if(treadmills.status==STATUS_SLEEP_MODE)  //休眠状态下，按遥控器任何按键
    {
        treadmills.status=STATUS_POWERON; 
				beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
				key_delay_dede=20;
        return;
    }		
		no_key_cnt=0;

   if(FOHA_cont_cnt>90)	//长按延时
   {
		 key=new;
		 delay_add++;
		 if(delay_add==15)		//连击延时，越小越快，15
		 {
       key_long_flag=1;
			 delay_add=0;
			 keypad_continue_proc();
		 }
   }else
	 {
		 FOHA_cont_cnt++;
	 }
	}else//按键无效了
	{
		if(FOHA_cont_cnt==1)
		{
			FOHA_cont_cnt=0;
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		}
	}
	
	if(new !=old)//按键有变更
	{
    no_key_cnt=0;
		key=new;
		keypad_short_proc();
		old=new;
		if(FOHA_cont_cnt>90)//连击结束响
		{
			FOHA_cont_cnt=1;
		}else
		{
			FOHA_cont_cnt=0;
		}
	}	
}

