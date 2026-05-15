/*
 * keypad_fun.c
 */
#include "common.h"
#include "keypad_fun.h"
#include "beep.h"
#include "led_disp.h"
#include "uart_frame.h"
#include "uart.h"
#include "treadmills.h"
#include "7_segment.h"
#include "indecate.h"
#include "treadmills.h"
#include "param.h"
#include "programe.h"
#include "ee_param.h"
#include "decode.h"

static u8 index=0;
bit PAUSE_FLAG = 0;

//按键加
static void btn_add_proc(int scale)
{
  if(scale==1)
  {
    if(treadmills.status==STATUS_RUNNING)
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
      else{treadmills.param.speed=treadmills.param.speed_max;}

      uart1_frame_tx(CMD_SPEED,treadmills.param.speed/10);
		  treadmills.display.index=0;
      treadmills.display.mode=DISP_MODE_SINGLE;
    }
  } 
}

//按键减
static void btn_sub_proc(int scale)
{
  if(scale==1)
  {
    if(treadmills.status==STATUS_RUNNING)
    {
     if(treadmills.param.speed > DEFAULT_SPEED_MIN )//100
      {
        beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
        treadmills.param.speed -= treadmills.param.speed_step;  //2.2
        if( treadmills.param.speed < DEFAULT_SPEED_MIN )
        {
          treadmills.param.speed = DEFAULT_SPEED_MIN ;           //避免减了步距之后小于最小值2023.2.2
        }
      }
      else{treadmills.param.speed = DEFAULT_SPEED_MIN ;}
					
      uart1_frame_tx(CMD_SPEED,treadmills.param.speed/10);
		  treadmills.display.index=0;
      treadmills.display.mode=DISP_MODE_SINGLE;
    }
  }
}

int speed_modes[] = {SPEED_MODE_2KM, SPEED_MODE_4KM, SPEED_MODE_6KM};  // 定义速度模式数组和索引

void keypad_short_proc(void)
{
  u8 bitmap = key;    

//-------------------- +键处理 -----------------------------
  if((bitmap & BTN_ADD) &&(key_mask &BTN_ADD)) 
	{
		btn_add_proc(1);
	}

//-----------------------启动/暂停键处理 ----------------------------
  if((bitmap & BTN_ONOFF) &&(key_mask &BTN_ONOFF))
	{
		beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		if(treadmills.status==STATUS_WAIT)
		{
			treadmills.status=STATUS_RUN;
			key_mask =0xff;//打开所有按键
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		}
		else if(treadmills.status==STATUS_RUN)//进入运行到计时间，但未真正的开始运行的时候。
		{
			treadmills.status=STATUS_STOP_OVER;
			key_mask=0x00;//禁止所有按键操作
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		}
		else if(treadmills.status==STATUS_RUNNING)//电机正在运行中
		{ 
			treadmills.status=STATUS_STOP;
			key_mask=0x00;//禁止所有按键操作
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		}
    else if(treadmills.status==STATUS_PAUSE)
		{
      PAUSE_FLAG = 0;
			treadmills.status=STATUS_RUN;     
			key_mask =0xff;//打开所有按键
			beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		}
	}

// ------------------模式按键 处理-------------------------------
  if((bitmap & BTN_MODE) &&(key_mask &BTN_MODE))
	{       
  //-----------------运行时 BTN_MODE 键处理 -------------------------
    if (treadmills.status == STATUS_RUNNING) 
    {
        int current_mode_index = 0; // 当前模式索引
        int current_speed = treadmills.param.speed;      // 获取当前速度
        beep_set(1, DEFAULT_ON_TICKS, DEFAULT_OFF_TICKS); // 响铃提示切换     

        // 根据当前速度设置新的模式
        if (current_speed < SPEED_MODE_2KM) {
            // 小于 2KM 显示 2KM 模式
            current_mode_index = 0;
        } else if (current_speed >= SPEED_MODE_2KM && current_speed < SPEED_MODE_4KM) {
            // 大于等于 2KM 且小于 4KM 显示 4KM 模式
            current_mode_index = 1;
        } else if (current_speed >= SPEED_MODE_4KM && current_speed < SPEED_MODE_6KM) {
            // 大于等于 4KM 且小于 6KM 显示 6KM 模式
            current_mode_index = 2;
        } else if (current_speed == SPEED_MODE_6KM) {
            // 等于 6KM 切换回 2KM 模式
            current_mode_index = 0;
        }

      
      treadmills.param.speed = speed_modes[current_mode_index];// 设置跑步机速度
      uart1_frame_tx(CMD_SPEED, treadmills.param.speed / 10); // 发送速度到下控

      // 更新显示模式
      treadmills.display.index = 0;
      treadmills.display.mode = DISP_MODE_SINGLE;
    }
	}
//----------------- -键处理 -------------------------
  if((bitmap & BTN_SUB) &&(key_mask &BTN_SUB))
	{
		btn_sub_proc(1);
	}
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
		btn_add_proc(1);
    is_disable_beep = 0; //恢复蜂鸣器
	}
//-------------------- -键处理 -----------------------------
	if((bitmap & BTN_SUB) &&(key_mask &BTN_SUB))
	{
    is_disable_beep = 1; //连击禁止蜂鸣器响
		btn_sub_proc(1);
    is_disable_beep = 0; //恢复蜂鸣器
	}
}



