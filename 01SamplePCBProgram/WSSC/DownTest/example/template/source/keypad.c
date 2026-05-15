#include "keypad.h"
#include "keypad_fun.h"
#include "led_disp.h"
#include "decode.h"
#include "treadmills.h"
#include "beep.h" 


static u8 new ,old;

u8 FOHA_cont_cnt = 0;

u8 key = 0;
u8 key_cnt = 0x00;
u8 key_mask = 0x00;
u8 delay_add = 0x00;
u8 tl_flag = 0;
u16 key_delay_dede = 0;
extern u16 no_key_cnt;

void KeyPad_Scan(void)
{
   /**
   * 按键输入入口,和硬件有关系
   */
  //=================================
	new=0;
  //客户433键值
	if(key_value==0X04){new |=(1<<0);}  //+			//0X04
	if(key_value==0X1A){new |=(1<<1);}  //ON/OFF//0X1A
	if(key_value==0X01){new |=(1<<2);}  //M			//0X01
	if(key_value==0X05){new |=(1<<3);}  //-     //0X05
  
//	if(key_value==0X2E){new |=(1<<0);}  //+		
//	if(key_value==0X3D){new |=(1<<1);}  //M	
//	if(key_value==0X4C){new |=(1<<2);}  //-		
//	if(key_value==0X5B){new |=(1<<3);}  //ON/OFF
//	if(key_value==0X79){new |=(1<<4);}  //SLEEP
	
	if(key_delay_dede)
	{
		key_delay_dede--;
		return;
	}
	
	if(new != old)//按键有变更
	{
    no_key_cnt = 0;
		key = new;
		keypad_short_proc();
		old = new;
		if(FOHA_cont_cnt > 100)//连击结束响
		{
			FOHA_cont_cnt = 1;
		}else
		{
			FOHA_cont_cnt = 0;
		}
	}
	
	if(new != 0)//按键有效
	{
		no_key_cnt = 0;
    if(FOHA_cont_cnt > 100)	//长按延时
      {
       key = new;
       delay_add++;
       if(delay_add == 4)		//连击延时
         {
            delay_add = 0;
            keypad_continue_proc();
         }
   }
			else
	 {
		 FOHA_cont_cnt++;
	 }
	}else//按键无效了
	{
		if(FOHA_cont_cnt == 1)
		{
			FOHA_cont_cnt = 0;
			BEEP_SWITCH_ON_1_CNT();  
		}
	}
}

