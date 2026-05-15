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
#include "param.h"
#include "my_flash.h"
#include "programe.h"

_Bool write_flag = 0;
u16 old_bl=0;
u8 STATUS_URGENT_STOP=0;
/* 干簧管 */
void keypad_all_realse_proc(void)
{
  if(treadmills.status!=STATUS_ERROR&&treadmills.status!=IDEL)
	{
    param.en=0;
    treadmills.param.speed=0;
		param.book_en=0;
		param.index=0;
		treadmills.error_code=0;
		treadmills.status=STATUS_ERROR;
		STATUS_URGENT_STOP=1;
    set_seg_mode(SEG_MODE_NORMAL);
    beep_set(1,E07_ON_TICKS ,E07_OFF_TICKS);
  }
}

/* 按键＋进程 */
static void btn_add_proc(void)
{
  BEEP_SWITCH_ON_1_CNT();
  if(treadmills.status==STATUS_SET_PARAM)
  {
    if(param.index==PARAM_SET_TIME)
    {		
      if((treadmills.param.second+DEFAULT_TIME_SINGLE)<=(TIME_MAX))
      {
        treadmills.param.second+=DEFAULT_TIME_SINGLE;
      }
      else
      {
        treadmills.param.second=TIME_MIN;
      }
      
      treadmills_display(param.index);
    }      
    else if(param.index==PARAM_SET_DISTANCE)
    {			
      treadmills.param.distance+=DEFAULT_DISTANCE_SINGLE;
      if(treadmills.param.distance>DISTANCE_MAX)
      {
        treadmills.param.distance=DISTANCE_MIN;
      }
      
      treadmills_display(param.index);
    }
    else if(param.index==PARAM_SET_CALORIE)
    { //单位: 卡
      treadmills.param.calorie+=DEFAULT_CALORIE_SINGLE;
      if(treadmills.param.calorie>CALORIE_MAX)
      {
        treadmills.param.calorie=CALORIE_MIN;
      }
      
      treadmills_display(param.index);      
    }
    set_seg_mode(SEG_MODE_NORMAL);
    old_bl = BLINK_HOLD_TIME;
  }
  else if(treadmills.status==STATUS_RUNNING)
	{
    if(treadmills.param.speed < DEFAULT_SPEED_MAX)  //600
    {
      treadmills.param.speed += treadmills.param.speed_step; 
      uart_frame_tx(CMD_SPEED,treadmills.param.speed / 10);
    }else{
			treadmills.param.speed = DEFAULT_SPEED_MAX;
		}

    treadmills.display.index = PARAM_SET_SPEED_ADC;
    treadmills.display.mode = DISP_MODE_SINGLE;
  }
  else if(treadmills.status == STATUS_WAIT  || treadmills.status == STATUS_SET_PROGRAM  || treadmills.status == STATUS_RUN)
  {
    beep_set(0,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS); 
  }  
}

/* 按键-进程 */
static void btn_sub_proc(void)
{
  BEEP_SWITCH_ON_1_CNT();
  if(treadmills.status == STATUS_SET_PARAM)
  {
    if(param.index == PARAM_SET_TIME)
    {
      if(treadmills.param.second >= TIME_MIN+DEFAULT_TIME_SINGLE)
      {
        treadmills.param.second -= DEFAULT_TIME_SINGLE;		
      }
      else
      {
        treadmills.param.second = TIME_MAX;
      }
      
      treadmills_display(param.index);
    }
    else if(param.index == PARAM_SET_DISTANCE)
    {
      if(treadmills.param.distance >= DISTANCE_MIN + DEFAULT_DISTANCE_SINGLE)
      {
        treadmills.param.distance -= DEFAULT_DISTANCE_SINGLE;
      }
      else
      {
        treadmills.param.distance = DISTANCE_MAX;
      }    
      treadmills_display(param.index);
    }
    else if(param.index == PARAM_SET_CALORIE)
    { //单位:卡
      if(treadmills.param.calorie >= CALORIE_MIN+DEFAULT_CALORIE_SINGLE)
      {
        treadmills.param.calorie -= DEFAULT_CALORIE_SINGLE;
      }
      else
      {
        treadmills.param.calorie = CALORIE_MAX;
      }
      
      treadmills_display(param.index);
    }
    set_seg_mode(SEG_MODE_NORMAL);
    old_bl = BLINK_HOLD_TIME;
  }
	else if(treadmills.status == STATUS_RUNNING)
  {
    if(treadmills.param.speed > DEFAULT_SPEED_MIN)
    {
      treadmills.param.speed -= treadmills.param.speed_step;//50;
      uart_frame_tx(CMD_SPEED,treadmills.param.speed / 10);
    }else{
			treadmills.param.speed = DEFAULT_SPEED_MIN;
		}
    
    treadmills.display.index = PARAM_SET_SPEED_ADC;
    treadmills.display.mode = DISP_MODE_SINGLE;
  }
  else if(treadmills.status == STATUS_WAIT || treadmills.status==STATUS_SET_PROGRAM  || treadmills.status == STATUS_RUN)
  {
    beep_set(0,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS); 
  }  
}


//短按进程
void keypad_short_proc(void)
{
  u8 bitmap = key;
	if(key_cnt > 1){return;}//多键盘的限制，只能单键操作
  
  /* +键处理 */
  if((bitmap & BTN_ADD)&&(key_mask &BTN_ADD)) 
  {
    btn_add_proc();
  }
  
  /* P键处理 */
  if((bitmap & BTN_PROGRAM)&&(key_mask &BTN_PROGRAM)) 
  {
    BEEP_SWITCH_ON_1_CNT();
		if(treadmills.status == STATUS_WAIT)
		{
			treadmills.status = STATUS_SET_PROGRAM;      
      key_mask = 0xFF;
      param.index = PARAM_SET_PROGRAME;
      programe.use_prog_speed = 1;
      treadmills.display.index = param.index;
      treadmills_display(param.index);
      set_seg_mode(SEG_MODE_FLASH_SELF);          
			return;
		}
    else if(treadmills.status == STATUS_SET_PROGRAM)
    {
      programe.index++;      
      if(programe.index >= 12)
      {       
        programe.index = 0;
        treadmills.param.second = 1800;//秒
      }     
      key_mask = 0xFF;      
      param.index = PARAM_SET_PROGRAME;
      programe.use_prog_speed = 1;
      treadmills.display.index=param.index;
      treadmills_display(param.index);
      set_seg_mode(SEG_MODE_FLASH_SELF); 
      return;
    }
    else if(treadmills.status == STATUS_SET_PARAM)
    {
        // 按P键，从设置参数切换到设置程序
        treadmills.status = STATUS_SET_PROGRAM;
        key_mask = 0xFF;
        param.index = PARAM_SET_PROGRAME;
        programe.use_prog_speed = 1;
        treadmills.display.index = param.index;
        treadmills_display(param.index);
        set_seg_mode(SEG_MODE_FLASH_SELF);          
        return;
    }
    else if(treadmills.status == STATUS_RUNNING  || treadmills.status == STATUS_RUN)
    {
      beep_set(0,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);     
    }
  }
  
  /* M键处理 */
  if((bitmap & BTN_MODE)&&(key_mask &BTN_MODE))
	{
    BEEP_SWITCH_ON_1_CNT();
		if(treadmills.status == STATUS_WAIT)
		{
			treadmills.status = STATUS_SET_PARAM;
      key_mask=0xFF;//打开所有键盘
      treadmills.param.second = 1800;//秒  
      treadmills.param.distance = 0;
      treadmills.param.calorie = 0;
      programe.use_prog_speed = 0;
      param.index=PARAM_SET_TIME;
			treadmills.display.index=param.index;
      treadmills_display(param.index);
			set_seg_mode(SEG_MODE_FLASH_SELF);
			return;
		}
		else if(treadmills.status == STATUS_SET_PARAM)
		{
      param.index++;
      if (param.index >= 4) 
      {       
        treadmills.param.second = 1800;//秒  
        treadmills.param.distance = 0;
        treadmills.param.calorie = 0;
        param.index=PARAM_SET_TIME;
      }
			else if(param.index == PARAM_SET_DISTANCE)
			{
        treadmills.param.second = 0;      
        treadmills.param.distance = 1000;//米 
        treadmills.param.calorie = 0;
			}
			else if(param.index == PARAM_SET_CALORIE)
			{
        treadmills.param.second = 0;
        treadmills.param.distance = 0;
				treadmills.param.calorie = 50000;//卡
			}
			
			key_mask = 0xFF;//打开所有键盘
      treadmills.display.index = param.index;
			treadmills_display(param.index);
			set_seg_mode(SEG_MODE_FLASH_SELF);
			return;
		}
    else if(treadmills.status == STATUS_SET_PROGRAM)
    {
        /* M 进时间/距离后 param 不再是 PROGRAME，保持程式跑标志 */
        programe.use_prog_speed = 1;
        treadmills.status = STATUS_SET_PARAM;
        key_mask = 0xff;
        treadmills.param.second = 1800;
        treadmills.param.distance = 0;
        treadmills.param.calorie = 0;
        param.index=PARAM_SET_TIME;
        treadmills.display.index = param.index;
        treadmills_display(param.index);
        set_seg_mode(SEG_MODE_FLASH_SELF);
        return;
    }
    else if(treadmills.status == STATUS_RUNNING || treadmills.status == STATUS_RUN)
    {
      beep_set(0,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);           
    }
	}

  /* START键处理 */
  if((bitmap & BTN_START) &&(key_mask &BTN_START))
	{
    BEEP_SWITCH_ON_1_CNT();
		if(treadmills.status==STATUS_WAIT)
		{
			programe.run_use_tab_speed = 0;
			programe.session_is_program = 0;
			treadmills.status=STATUS_RUN;
			key_mask =0xff;      
		}
		else if(treadmills.status==STATUS_SET_PARAM)
		{
			programe.run_use_tab_speed = (programe.use_prog_speed != 0u) ? 1u : 0u;
			treadmills.status=STATUS_RUN;
			key_mask =0xff;
		}
    else if(treadmills.status==STATUS_SET_PROGRAM)
		{
			programe.run_use_tab_speed = 1;
			treadmills.status=STATUS_RUN;
			key_mask =0xff;
		}
    else if(treadmills.status==STATUS_RUNNING)
    {
      beep_set(0,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);           
    }
	}
  
  /* STOP键处理 */
  if((bitmap & BTN_STOP) &&(key_mask &BTN_STOP))
	{
    BEEP_SWITCH_ON_1_CNT();
		if(treadmills.status==STATUS_RUN)
		{
			programe.run_use_tab_speed = 0;
			treadmills_natural_stop_hold_off = 0;
			treadmills.status=STATUS_STOP_OVER;
			key_mask=0x00;
		}
		else if(treadmills.status==STATUS_RUNNING)
		{
			treadmills_natural_stop_hold_off = 0;
			treadmills.status=STATUS_STOP;
			key_mask=0x00;
		}
    else if(treadmills.status==STATUS_SET_PARAM)
		{
      programe.use_prog_speed = 0;
      programe.run_use_tab_speed = 0;
      programe.session_is_program = 0;
      treadmills.status = STATUS_POWERON;
      return;
		}
    else if(treadmills.status==STATUS_SET_PROGRAM)
		{
			programe.use_prog_speed = 0;
			programe.run_use_tab_speed = 0;
			programe.session_is_program = 0;
			treadmills.status=STATUS_POWERON;
      return;
		}   
	}
  
  /* -键处理 */
  if((bitmap & BTN_SUB) &&(key_mask &BTN_SUB))
	{
		btn_sub_proc();
	}
}

void keypad_continue_proc(void)
{
	u8 bitmap = key;
	if(key_cnt>1){return;}
	//-------------------- +键处理 -----------------------------
	if((bitmap & BTN_ADD) &&(key_mask &BTN_ADD))
	{
    BeepState = 1; //连击禁止蜂鸣器响
		btn_add_proc();
    BeepState = 0; //恢复蜂鸣器
	}
//-------------------- -键处理 -----------------------------
	if((bitmap & BTN_SUB) &&(key_mask &BTN_SUB))
	{
    BeepState = 1; //连击禁止蜂鸣器响
		btn_sub_proc();
    BeepState = 0; //恢复蜂鸣器
	}
}

/* 更新显示常亮闪烁 */
void update_display_mode(void)
{
  if (treadmills.status == STATUS_SET_PARAM)
  {
    if (old_bl > 0)
    {
      old_bl--;
      if (old_bl == 0)
      {
        set_seg_mode(SEG_MODE_FLASH_SELF);
      }
    }
  } 
}





