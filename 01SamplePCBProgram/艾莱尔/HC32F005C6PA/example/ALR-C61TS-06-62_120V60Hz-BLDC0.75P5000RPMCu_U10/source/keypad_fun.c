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
#include "programe.h"

_Bool write_flag = 0;
u16 old_bl=0;
u8 STATUS_URGENT_STOP=0;
_Bool PAUSE_FLAG = 0;

/* M 键：响应一次后需松键（蓝牙/RX 连发看成长按）；松键由 keypad_m_key_release_poll 清 latch */
static u8 s_mode_m_key_latched;

void keypad_m_key_release_poll(u8 raw_keys)
{
	if ((raw_keys & BTN_MODE) == 0u) {
		s_mode_m_key_latched = 0u;
	}
}

/* 干簧管 / 安全锁释放 */
void keypad_all_realse_proc(void)
{
#if TM_SAFETY_LOCK_ENABLE
	if (treadmills.status != STATUS_ERROR && treadmills.status != IDEL) {
		param.en = 0;
		treadmills.param.speed = 0;
		/*		param.book_en=0; */
		param.index                = 0;
		treadmills.error_code      = ERROR_DISPLAY_SAFETY_LOCK;
		treadmills.status          = STATUS_ERROR;
		STATUS_URGENT_STOP         = 1;
		/*		uart_frame_tx(CMD_STATUS_URGENT_STOP,1);//发送紧急停止 */
		/*		uart_frame_tx(CMD_STATUS_URGENT_STOP,1); */
		set_seg_mode(SEG_MODE_NORMAL);
		beep_set(1, SAFETY_LOCK_BEEP_ON_TICKS, SAFETY_LOCK_BEEP_OFF_TICKS);
	}
#endif
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
      treadmills.param.speed += treadmills.param.speed_step;//50; //2023.3.27
      if( treadmills.param.speed > DEFAULT_SPEED_MAX)
      {
				treadmills.param.speed = DEFAULT_SPEED_MAX;
      }
    }
		else{treadmills.param.speed = DEFAULT_SPEED_MAX;}
		
		uart_frame_tx(CMD_SPEED,treadmills.param.speed/10);
    treadmills.display.index = PARAM_SET_SPEED_ADC;
    treadmills.display.mode = DISP_MODE_SINGLE;
  }
  else if(treadmills.status==STATUS_WAIT  || treadmills.status==STATUS_SET_PROGRAM)
  {
    beep_set(0,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS); 
  }  
}

/* 按键-进程 */
static void btn_sub_proc(void)
{
  BEEP_SWITCH_ON_1_CNT();

  if(treadmills.status==STATUS_SET_PARAM)
  {
    if(param.index==PARAM_SET_TIME)
    {
      if(treadmills.param.second>=TIME_MIN+DEFAULT_TIME_SINGLE)
      {
        treadmills.param.second-=DEFAULT_TIME_SINGLE;		
      }
      else
      {
        treadmills.param.second=TIME_MAX;
      }
      
      treadmills_display(param.index);
    }
    else if(param.index==PARAM_SET_DISTANCE)
    {
      if(treadmills.param.distance>=DISTANCE_MIN+DEFAULT_DISTANCE_SINGLE)
      {
        treadmills.param.distance-=DEFAULT_DISTANCE_SINGLE;
      }
      else
      {
        treadmills.param.distance=DISTANCE_MAX;
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
		if(treadmills.param.speed > DEFAULT_SPEED_MIN )//100
    {
			treadmills.param.speed -= treadmills.param.speed_step; // 50
			if(treadmills.param.speed < DEFAULT_SPEED_MIN)
			{   
				treadmills.param.speed = DEFAULT_SPEED_MIN;      
			}
		}
		else{treadmills.param.speed = DEFAULT_SPEED_MIN;}
			
		uart_frame_tx(CMD_SPEED,treadmills.param.speed/10);
		treadmills.display.index = PARAM_SET_SPEED_ADC;
		treadmills.display.mode = DISP_MODE_SINGLE;
  }

  else if(treadmills.status==STATUS_WAIT || treadmills.status==STATUS_SET_PROGRAM)
  {
    beep_set(0,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS); 
  }  
}

u8 ss_buf[] = {0x01};  // 定义速度模式数组和索引

//短按进程
void keypad_short_proc(void)
{
  u8 bitmap = key;
	if(key_cnt>1){return;}//多键盘的限制，只能单键操作
  
  /* +键处理 */
  if((bitmap & BTN_ADD) &&(key_mask &BTN_ADD)) 
  {
    btn_add_proc();
  }
  
  /* P键处理 */
  if((bitmap & BTN_PROGRAM) &&(key_mask &BTN_PROGRAM)) 
  {
    BEEP_SWITCH_ON_1_CNT();
		if(treadmills.status==STATUS_WAIT)
		{
			treadmills.status=STATUS_SET_PROGRAM;      
      key_mask = 0xFF;
      param.index=PARAM_SET_PROGRAME;
      treadmills.display.index=param.index;
      treadmills_display(param.index);
      set_seg_mode(SEG_MODE_FLASH_SELF);          
			return;
		}
    else if(treadmills.status==STATUS_SET_PROGRAM)
    {
      programe.index++;      
      if(programe.index >= 12)
      {       
        programe.index = 0;
        treadmills.param.second=1800;//秒
      }     
      key_mask = 0xFF;      
      param.index=PARAM_SET_PROGRAME;
      treadmills.display.index=param.index;
      treadmills_display(param.index);
      set_seg_mode(SEG_MODE_FLASH_SELF); 
      return;
    }
    else if(treadmills.status==STATUS_SET_PARAM)
    {
        // 按P键，从设置参数切换到设置程序
        treadmills.status=STATUS_SET_PROGRAM;
        key_mask = 0xFF;
        param.index=PARAM_SET_PROGRAME;
        treadmills.display.index=param.index;
        treadmills_display(param.index);
        set_seg_mode(SEG_MODE_FLASH_SELF);          
        return;
    }
    else if(treadmills.status==STATUS_RUNNING)
    {
      beep_set(0,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);     
    }
  }
  
  /* M键处理：运行中每“按一次”只切一档，须松键后再按（避免长按/连发多档） */
  if ((bitmap & BTN_MODE) && (key_mask & BTN_MODE)) {
		if (treadmills.status == STATUS_RUNNING) {
			if (s_mode_m_key_latched != 0u) {
				/* 已切过档，等松键 */
			} else {
				s_mode_m_key_latched = 1u;
				BEEP_SWITCH_ON_1_CNT();
				if (treadmills.mode_speed == 0) {
					treadmills.mode_speed     = 320;
					treadmills.param.speed    = treadmills.mode_speed;
					uart_frame_tx(CMD_SPEED, treadmills.param.speed / 10);
				} else if (treadmills.mode_speed == 320) {
					treadmills.mode_speed     = 640;
					treadmills.param.speed    = treadmills.mode_speed;
					uart_frame_tx(CMD_SPEED, treadmills.param.speed / 10);
				} else if (treadmills.mode_speed == 640) {
					treadmills.mode_speed     = 960;
					treadmills.param.speed    = treadmills.mode_speed;
					uart_frame_tx(CMD_SPEED, treadmills.param.speed / 10);
				} else if (treadmills.mode_speed == 960) {
					treadmills.mode_speed     = 320;
					treadmills.param.speed    = treadmills.mode_speed;
					uart_frame_tx(CMD_SPEED, treadmills.param.speed / 10);
				}
				treadmills.display.index = PARAM_SET_SPEED_ADC;
				treadmills.display.mode  = DISP_MODE_SINGLE;
			}
		} else {
			BEEP_SWITCH_ON_1_CNT();
		}
	}

  /* START键处理 */
  if((bitmap & BTN_START) &&(key_mask &BTN_START))
	{
    BEEP_SWITCH_ON_1_CNT();
		key_mask =0xff;    
		
		if(treadmills.status==STATUS_WAIT)
		{
			treadmills.status=STATUS_RUN;  
		}
		else if(treadmills.status==STATUS_SET_PARAM)
		{
			treadmills.status=STATUS_RUN;
		}
		else if(treadmills.status==STATUS_PAUSE)
		{
      PAUSE_FLAG = 0;
			treadmills.status=STATUS_RUN;     
		}
    else if(treadmills.status==STATUS_SET_PROGRAM)
		{
			treadmills.status=STATUS_RUN;
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
			treadmills.status=STATUS_STOP_OVER;
			key_mask=0x00;
		}
		else if(treadmills.status==STATUS_RUNNING)
		{
			treadmills.status=STATUS_STOP;
			key_mask=0x00;
		}
    else if(treadmills.status==STATUS_SET_PARAM)
		{
      treadmills.status = STATUS_POWERON;
      return;
		}
    else if(treadmills.status==STATUS_SET_PROGRAM)
		{
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

/*    */
void keypad_long_proc(void)
{

}

/*   */
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





