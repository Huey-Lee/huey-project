/*
 * keypad_fun.c
 */
#include "common.h"
#include "keypad_fun.h"
#include "beep.h"
#include "led_disp.h"
#include "uart_frame.h"
#include "treadmills.h"
#include "7_segment.h"
#include "indecate.h"
#include "treadmills.h"
#include "param.h"
#include "my_flash.h"


_Bool write_flag = 0;
_Bool PAUSE_FLAG = 0;

//按键加
static void btn_add_proc(void)
{
	if(treadmills.status==STATUS_SET_PARAM)
	{
		if(param.index==PARAM_SET_TIME)
		{
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
			if((treadmills.param.second+60)<=TIME_MAX)
			{
				treadmills.param.second+=60;
			}
			else
			{
				treadmills.param.second = TIME_MIN;
			}
			treadmills_display(param.index);
//			set_seg_mode(SEG_MODE_FLASH_SELF);
		}
		else if(param.index==PARAM_SET_DISTANCE)
		{
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);

			  treadmills.param.distance+=1000;
        if(treadmills.param.distance>DISTANCE_MAX)
        {
          treadmills.param.distance=DISTANCE_MIN;
        }
      
			treadmills_display(param.index);
//			set_seg_mode(SEG_MODE_FLASH_SELF);
		}
		else if(param.index == PARAM_SET_CALORIE)
		{
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
			//单位: 卡
			treadmills.param.calorie += 10000;
			if(treadmills.param.calorie>CALORIE_MAX)
			{
				treadmills.param.calorie=CALORIE_MIN;
			}
			treadmills_display(param.index);
//			set_seg_mode(SEG_MODE_FLASH_SELF);
		}
	}
	else if(treadmills.status == STATUS_RUNNING)
	{
		if(treadmills.param.speed < treadmills.param.speed_max)//treadmills.param.speed_max
		{
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
			treadmills.param.speed += treadmills.param.speed_step;
      if( treadmills.param.speed > treadmills.param.speed_max )
      {
        treadmills.param.speed = treadmills.param.speed_max;
      }
		}
		else{ treadmills.param.speed = treadmills.param.speed_max; }
//		uart_frame_tx_4byte(treadmills.param.speed);
//		uart_frame_tx_2(CMD_SPEED, (uint8_t)((treadmills.param.speed/10) & 0xFF), (uint8_t)(((treadmills.param.speed/10) >> 8) & 0xFF));
		uart_frame_tx_d1(0x03, 0x00, treadmills.param.speed);
		treadmills.display.index = PARAM_SET_SPEED_ADC;
		treadmills.display.mode = DISP_MODE_GEAR_HOLD;
	}
	else if(treadmills.status == STATUS_SLEEP_MODE)
	{
		beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		treadmills.status=STATUS_POWERON;
	}
}

//按键减
static void btn_sub_proc(void)
{
	if(treadmills.status == STATUS_SET_PARAM)
	{
//		beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
//		if(param.index==PARAM_SET_SPEED_ADC)
//		{
//			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
//			param.speed_adc-=1;
//			if(param.speed_adc<SPEED_ADC_MIN){param.speed_adc=SPEED_ADC_MIN;}
//			disp_speed_adc_param();
//		}
		if(param.index == PARAM_SET_TIME)
		{
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
			if(treadmills.param.second>=(TIME_MIN+60))
			{
				treadmills.param.second-=60;
			}
			else
			{
				treadmills.param.second=TIME_MAX;
			}
			treadmills_display(param.index);
//			set_seg_mode(SEG_MODE_FLASH_SELF);
		}
		else if(param.index==PARAM_SET_DISTANCE)
		{
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);

        if(treadmills.param.distance>=(DISTANCE_MIN+100))
        {
          treadmills.param.distance-=1000;
        }
        else
        {
          treadmills.param.distance=DISTANCE_MAX;
        }
      
			treadmills_display(param.index);
//			set_seg_mode(SEG_MODE_FLASH_SELF);
		}
		else if(param.index==PARAM_SET_CALORIE)
		{
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
			//单位:卡
			if(treadmills.param.calorie>=(CALORIE_MIN+1000))
			{
				treadmills.param.calorie-=10000;
			}
			else
			{
				treadmills.param.calorie=CALORIE_MAX;
			}
			treadmills_display(param.index);
//			set_seg_mode(SEG_MODE_FLASH_SELF);
		}
	}
	else if(treadmills.status == STATUS_RUNNING)
	{
		if(treadmills.param.speed > DEFAULT_SPEED_MIN)//100
		{
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
      treadmills.param.speed -= treadmills.param.speed_step;  //2.2
      if( treadmills.param.speed < DEFAULT_SPEED_MIN)
      {
        treadmills.param.speed = DEFAULT_SPEED_MIN;           //避免减了步距之后小于最小值2023.2.2
      }
		}
		else{treadmills.param.speed = DEFAULT_SPEED_MIN;}
//		uart_frame_tx_4byte(treadmills.param.speed);
//		uart_frame_tx_2(CMD_SPEED, (uint8_t)((treadmills.param.speed/10) & 0xFF), (uint8_t)(((treadmills.param.speed/10) >> 8) & 0xFF));
		uart_frame_tx_d1(0x03, 0x00, treadmills.param.speed);
		treadmills.display.index = PARAM_SET_SPEED_ADC;
		treadmills.display.mode = DISP_MODE_GEAR_HOLD;
	}
	else if(treadmills.status == STATUS_SLEEP_MODE)
	{
		beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		treadmills.status = STATUS_POWERON;
	}
}

void keypad_short_proc(void)
{
  u8 bitmap = key;  

//-------------------- +键处理 -----------------------------
  if((bitmap & BTN_ADD) &&(key_mask &BTN_ADD)) 
	{
		btn_add_proc();
	}

//-----------------------启动/暂停键处理 ----------------------------
  if((bitmap & BTN_ONOFF) &&(key_mask &BTN_ONOFF))
	{
		beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		if(treadmills.status == STATUS_WAIT)
		{
			treadmills.status = STATUS_RUN;
			key_mask = 0xff;//打开所有按键
		}
		else if(treadmills.status == STATUS_RUN)//进入运行到计时间，但未真正的开始运行的时候。
		{
      PAUSE_FLAG = 0;
			treadmills.status = STATUS_STOP_OVER;
			key_mask = 0x00;//禁止所有按键操作
//			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		}
		else if(treadmills.status == STATUS_RUNNING)//电机正在运行中
		{
			PAUSE_FLAG = 0;
			treadmills.status = STATUS_STOP;
			key_mask=0x00;//禁止所有按键操作
//			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		}
//		else if(treadmills.status == STATUS_PAUSE)
//		{
//      PAUSE_FLAG = 0;
//			treadmills.status=STATUS_RUN;     
//			key_mask =0xff;//打开所有按键
////			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
//		}
		else if(treadmills.status==STATUS_SET_PARAM)//退出设置态
		{
			treadmills.status = STATUS_RUN;
//			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
			key_mask =0xff;//打开所有按键
		}
		else if(treadmills.status == STATUS_SLEEP_MODE)
		{
			treadmills.status = STATUS_POWERON;
		}
	}

// ------------------模式按键 处理-------------------------------
  if((bitmap & BTN_MODE) &&(key_mask &BTN_MODE))
	{
    if(treadmills.status!=STATUS_RUNNING)
    {
      beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
    }
		if(treadmills.status==STATUS_WAIT)
		{
			treadmills.status = STATUS_SET_PARAM;
//		  if(param.index<4){param.index++;}//增加了程序模式
      treadmills.param.second = 1800;//秒
      key_mask=0xff;//打开所有键盘
      param.index=PARAM_SET_TIME;
			treadmills.display.index=param.index;
      treadmills_display(param.index);
      
//			treadmills.param.speed=0;
//      treadmills.param.second=0;
//			treadmills.param.distance=0;
//      treadmills.param.runstep=0;
//			treadmills.param.calorie=0;
      
//      if(param.index==PARAM_SET_TIME)
//      {
//				treadmills.param.second=1800;//秒
//      }
//			set_seg_mode(SEG_MODE_FLASH_SELF);
			return;
		}
		else if(treadmills.status == STATUS_SET_PARAM)
		{
        param.index++;

        if (param.index > 3) {  
            treadmills.status = STATUS_POWERON;
						treadmills.param.second=0;
            treadmills.param.distance = 0;
            treadmills.param.calorie = 0;
            return;
        }
			if(param.index==PARAM_SET_TIME)
			{
				treadmills.param.second = 1800;//秒
			}
			else if(param.index==PARAM_SET_DISTANCE)
			{
				  treadmills.param.distance = 1000;//米 DISTANCE_MIN
			}
			else if(param.index==PARAM_SET_CALORIE)
			{
				treadmills.param.calorie = 50000;//卡
			}

			treadmills.display.index = param.index;
			key_mask = 0xff;//打开所有键盘
			treadmills_display(param.index);
//			set_seg_mode(SEG_MODE_FLASH_SELF);
			return;
		}
		else if(treadmills.status==STATUS_SLEEP_MODE)
		{
			beep_set(0,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		}
		else if(treadmills.status==STATUS_SLEEP_MODE)
		{
			treadmills.status=STATUS_POWERON;
		}
	}
	
//----------------- -键处理 -------------------------
  if((bitmap & BTN_SUB) &&(key_mask &BTN_SUB))
	{
		btn_sub_proc();
	}

//---------------- 休眠按键 -------------------------	
//	if((bitmap & BTN_SLEEP) &&(key_mask &BTN_SLEEP))
//  {

//    if((treadmills.status == STATUS_WAIT)||(treadmills.status==STATUS_PAUSE))
//		{
//			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
//			treadmills.status=STATUS_SLEEP_MODE;
//		}
//		else if(treadmills.status==STATUS_SLEEP_MODE)
//		{
//			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
//			treadmills.status=STATUS_POWERON;
//		}
//		if(treadmills.status==STATUS_RUN)//进入运行到计时间，但未真正的开始运行的时候。
//		{
//      PAUSE_FLAG = 0;
//			treadmills.status=STATUS_STOP_OVER;
//			key_mask=0x00;//禁止所有按键操作
//			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
//		}
//		else if(treadmills.status==STATUS_RUNNING)//电机正在运行中
//		{
//      PAUSE_FLAG = 0;
//			treadmills.status=STATUS_STOP;
//			key_mask=0x00;//禁止所有按键操作
//			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
//		}
//  }
}


void keypad_long_proc(void)
{
	u8 bitmap = key;
	key=key;
}


void keypad_continue_proc(void)
{
	 u8 bitmap = key;
	//-------------------- +键处理 -----------------------------
	if((bitmap & BTN_ADD) &&(key_mask &BTN_ADD))
	{
    is_disable_beep = 1; //连击禁止蜂鸣器响
		btn_add_proc();
    is_disable_beep = 0; //恢复蜂鸣器
	}
//-------------------- -键处理 -----------------------------
	if((bitmap & BTN_SUB) &&(key_mask &BTN_SUB))
	{
    is_disable_beep = 1; //连击禁止蜂鸣器响
		btn_sub_proc();
    is_disable_beep = 0; //恢复蜂鸣器
	}
}


