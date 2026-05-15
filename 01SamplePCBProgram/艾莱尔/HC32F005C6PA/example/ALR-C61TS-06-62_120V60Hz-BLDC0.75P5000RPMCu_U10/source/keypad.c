#include "keypad.h"
#include "keypad_fun.h"
#include "led_disp.h"
#include "decode.h"
#include "treadmills.h"
#include "beep.h" 

/**
 * 本键盘程序实现以下功能
 * 功能：
 * 1.将底层驱动和APP分离(keypad_func.c中添加具体实现的功能)，减少耦合性，方便移植。
 * 2.能够识别单键或组合键，键功能自行定义。
 * 3.能够检测当前按下键的数量。
 * 4.实现 短/长/连击功能，以消息循环自行响应按键功能。
 * 5.长按时间长短，连击速度快慢可以通过宏定义配置，短按立即触发快慢只和系统响应有关。
 * 6.具有软件设置按键的屏蔽功能，以及连击(可软件屏蔽)屏蔽功能。
 * 7.自动实现防抖动功能，硬件输入接口简单。
 *
 */

static u8 new ,old;

u8 FOHA_cont_cnt=0;

u8 key=0;
u8 key_cnt=0x00;
u8 key_mask=0x00;
u8 delay_add=0x00;
u8 tl_flag=0;
u8 protected_add=0;
extern u8 error_times;

/* 长按连发：10ms 节拍；ARM=约1.3s 后才开始连发，INTERVAL=每约250ms 发一档，减轻下控与协议压力 */
#define KEYPAD_REPEAT_ARM_TICKS   130u
#define KEYPAD_REPEAT_INTERVAL    25u

void KeyPad_Scan(void)
{
   /**
   * 按键输入入口,和硬件有关系
   */
  //=================================
	new=0;
  //客户433键值
//	if(key_value==0X04){new |=(1<<0);}  //+			//0X04
//	if(key_value==0X1A){new |=(1<<1);}  //ON/OFF
//	if(key_value==0X01){new |=(1<<2);}  //M			//0X01
//	if(key_value==0X05){new |=(1<<3);}  //-
  
  if(key_all.key0){new |=(1<<0);}
  if(key_all.key1){new |=(1<<1);}
  if(key_all.key2){new |=(1<<2);}
  if(key_all.key3){new |=(1<<3);}
  if(key_all.key4){new |=(1<<4);}
//  if(key_all.key5){new |=(1<<5);} 
  if(key_all.key6){new |=(1<<6);}

	keypad_m_key_release_poll(new);
  
  if(Reed_Val)//保险脱了 //
	{
		keypad_all_realse_proc();
		if(tl_flag==0)
		{
			tl_flag=1;
		}
	}
	else //if(key_reed==1)
	{
		if(tl_flag==1)
		{
			protected_add++;
			if(protected_add==50)
			{
				error_times=0;//报错次数清零
				treadmills.error_code=0;
				treadmills.status=STATUS_POWERON;
				protected_add=0;
				tl_flag=0;
				beep_set(1, SAFETY_LOCK_BEEP_ON_TICKS, SAFETY_LOCK_BEEP_OFF_TICKS);
			}	
		}
	}

	if(new !=old)//按键有变更
	{
    no_key_cnt=0;
		key=new;
		keypad_short_proc();
		old=new;
		delay_add = 0;
		if(FOHA_cont_cnt>100)//连击结束响
		{
			FOHA_cont_cnt=1;
		}else
		{
			FOHA_cont_cnt=0;
		}
	}
	if(new!=0)//按键有效
	{
    no_key_cnt=0;
    if(FOHA_cont_cnt > KEYPAD_REPEAT_ARM_TICKS)
    {
      key=new;
		  delay_add++;
		  if(delay_add >= KEYPAD_REPEAT_INTERVAL)
		  {
        delay_add=0;
			  keypad_continue_proc();
      }
    }
    else
    {
      FOHA_cont_cnt++;
    }
  }
  else//按键无效了
  {
    if(FOHA_cont_cnt==1)
		{
      FOHA_cont_cnt=0;
      BEEP_SWITCH_ON_1_CNT();  //2023.3.27
    }
  }
}

