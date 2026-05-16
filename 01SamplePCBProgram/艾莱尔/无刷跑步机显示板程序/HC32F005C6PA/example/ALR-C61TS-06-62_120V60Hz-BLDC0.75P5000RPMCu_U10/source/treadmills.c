#include "common.h"
#include "treadmills.h"
#include "keypad.h"
#include "led_disp.h"
#include "keypad_fun.h"
#include "uart_frame.h"
#include "indecate.h"
#include "7_segment.h"
#include "beep.h"
#include "param.h"
#include "programe.h"

void treadmills_display (u8 index);
treadmills_t treadmills;

extern u8 tl_flag;
extern u8 set_param_en;
extern s8 param_speed;
u8 error_times=0;

void treadmills_init(void)
{
	treadmills.ack=0;
	treadmills.mode_speed = 0u;
	treadmills.param.second=0; 
	treadmills.param.speed=DEFAULT_SPEED_MIN;  
	treadmills.param.speed_step = DEFAULT_SPEED_STEP;
  treadmills.param.beep_status = BEEP_SWITCH_ON;  //0表示蜂鸣器开，1表示关 
  treadmills_natural_stop_hold_off = 0;
}

static const u8 run_disp_tab[]={NUM_3,NUM_2,NUM_1};

/* 上电序列里「全亮后」的 OFF 界面，供停止/倒计时结束立即切换，避免中间档 0.6 停留 */
void treadmills_ui_apply_off_segments(void)
{
	indecate.led_speed    = LED_OFF;
	indecate.led_distance = LED_OFF;
	indecate.led_calorie  = LED_OFF;
	indecate.led_auto     = LED_OFF;
	indecate.led_time     = LED_OFF;
	indecate.led_updp     = LED_OFF;
	indecate.led_downdp   = LED_OFF;
	set_bit_seg(0, SEG_ALL_OFF);
	set_bit_seg(1, SEG_O);
	set_bit_seg(2, SEG_F);
	set_bit_seg(3, SEG_F);
}

u8 treadmills_natural_stop_hold_off = 0;

/* Shared by treadmills_proc_loop and lower-UART stop callbacks */
static u8 proc_loop_cnt;
static u8 proc_loop_timeout;
static u8 proc_loop_val;
/*
 * 程式第一档：下位机对已运行态的 CMD_SPEED（仅 FC_GEAR）响应与 3 分钟段切换一致；
 * 起步若直接 Send_Run(目标速) 可能仍跑 0.6，故先 Send_Run(MIN) 再延时补 CMD_SPEED。
 * treadmills_proc_loop 周期 100ms，5tick ≈ 500ms。
 */
#define PROG_START_CMD_SPEED_DELAY_TICKS (5u)
static u8 s_prog_startup_cmd_speed_delay;

void treadmills_poweron_lower_stop_tx(void)
{
	uart_frame_tx(CMD_STATUS, STATUS_MOTOR_STOP);
	uart_frame_tx(CMD_STATUS, STATUS_MOTOR_STOP);
}

void treadmills_on_enter_poweron(void)
{
	proc_loop_cnt     = 0u;
	proc_loop_val     = 0u;
	proc_loop_timeout = 0u;
}

/* Lower status 10 (idle): standby WAIT + default speed */
void treadmills_on_lower_standby_after_stop(void)
{
	if (treadmills.status != STATUS_STOP_WAIT && treadmills.status != STATUS_STOP_OVER) {
		return;
	}
	treadmills.param.speed    = DEFAULT_SPEED_MIN;
	treadmills.mode_speed     = 0u;
	treadmills.param.second   = 0;
	treadmills.param.distance = 0;
	treadmills.param.calorie  = 0;
	treadmills.display.mode   = IDEL;
	treadmills.status         = STATUS_WAIT;
	key_mask                  = 0xFF;
	treadmills_natural_stop_hold_off = 0;
	param.book_en             = 0;
	param.index               = 0;
	proc_loop_cnt             = 0;
	proc_loop_timeout         = 0;
	proc_loop_val             = 0;
	s_prog_startup_cmd_speed_delay = 0;
	beep_force_stop();
	treadmills_ui_apply_off_segments();
}

/* Lower status 7 (wait start): same route as timeout stop -> POWERON (OFF then WAIT) */
void treadmills_on_lower_poweron_after_stop(void)
{
	if (treadmills.status != STATUS_STOP_WAIT && treadmills.status != STATUS_STOP_OVER) {
		return;
	}
	treadmills.display.mode   = IDEL;
	treadmills.param.speed    = 0;
	treadmills.mode_speed     = 0u;
	treadmills.param.second   = 0;
	treadmills.param.distance = 0;
	treadmills.param.calorie  = 0;
	treadmills_ui_apply_off_segments();
	treadmills.status         = STATUS_POWERON;
	key_mask                  = 0xFF;
	treadmills_natural_stop_hold_off = 0;
	param.book_en             = 0;
	param.index               = 0;
	proc_loop_cnt             = 0;
	proc_loop_timeout         = 0;
	proc_loop_val             = 0;
	s_prog_startup_cmd_speed_delay = 0;
	beep_force_stop();
	uart_frame_tx(CMD_STOP_BURST, 5);
	uart_frame_tx(CMD_STOP_BURST, 5);
}
 
//参数计算设置  1000ms
void treadmills_param_calc(void)
{
  static float distance = 0;
	if(treadmills.status != STATUS_RUNNING){distance = 0;return;}
  if(param.index != PARAM_SET_PROGRAME && programe.use_prog_speed == 0u &&
      programe.session_is_program == 0u)//非程式模式
	{
    if(param.index == PARAM_SET_TIME)
    {
        if(treadmills.param.second){treadmills.param.second--;}
        else
        {
          treadmills.display.mode = IDEL;
          treadmills_ui_apply_off_segments();
          treadmills_natural_stop_hold_off = 1;
          treadmills.status = STATUS_STOP;
          key_mask = 0x00;
          if(treadmills.param.beep_status == BEEP_SWITCH_ON )  
          {
            beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
          }
        }
    }
    else
    {
      treadmills.param.second++;
      //增加定时功能---必须和键盘统一
      if(treadmills.param.second >= 6000)
      {
        treadmills.display.mode = IDEL;
        treadmills_ui_apply_off_segments();
        treadmills_natural_stop_hold_off = 1;
        treadmills.status = STATUS_STOP;
        key_mask = 0x00;
        if( treadmills.param.beep_status == BEEP_SWITCH_ON )  //2023.3.27
        {
          beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
        }
      }
    }

    if(param.index==PARAM_SET_DISTANCE)
    {
      distance += ((float)(treadmills.param.speed) / 1.6) / 360.0;//米/秒  单位：米
      treadmills.param.distance -= ((float)(treadmills.param.speed) / 1.6) / 360.0;//米/秒  单位：米
      if(treadmills.param.distance < 10)
      {
        treadmills.display.mode = IDEL;
        treadmills_ui_apply_off_segments();
        treadmills_natural_stop_hold_off = 1;
        treadmills.status = STATUS_STOP;
        key_mask = 0x00;
        
        if( treadmills.param.beep_status == BEEP_SWITCH_ON )  //2023.3.27
        {
          beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
        }
      }
    }
    else
    {
      /*
       * distance = V*t = (((float)treadmills.param.speed/100.0)*1000.0)/(3600.0)
       */
      treadmills.param.distance += ((float)(treadmills.param.speed) / 1.6) / 360.0;//米/秒  单位：米
      distance = treadmills.param.distance;
    }

    if(param.index==PARAM_SET_CALORIE)
    {
      if(treadmills.param.calorie>=(((float)(treadmills.param.speed) /1.6 )/360.0)*70)//80千卡/千米 =0.080K/M  单位:卡
      {
        treadmills.param.calorie -= (((float)(treadmills.param.speed) /1.6 )/360.0)*70;
      }
      else
      {
        treadmills.param.calorie=0;
        treadmills.display.mode = IDEL;
        treadmills_ui_apply_off_segments();
        treadmills_natural_stop_hold_off = 1;
        treadmills.status = STATUS_STOP;
        key_mask =0x00;
        
        if( treadmills.param.beep_status == BEEP_SWITCH_ON )  //2023.3.27
        {
          beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
        }
      }
    }
    else
    {
      treadmills.param.calorie = distance * 70;//80千卡/千米 =0.080K/M  单位:卡
    }  
  }
  else// 程序模式
	{
		//时间倒计时
		if(treadmills.param.second){treadmills.param.second--;}
		else
		{
			treadmills.display.mode = IDEL;
			treadmills_ui_apply_off_segments();
			treadmills_natural_stop_hold_off = 1;
			treadmills.status = STATUS_STOP;
      key_mask =0x00;
			beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
		}
    
    programe.interval_time++;
    if(programe.interval_time >= PROGRAME_INTERVAL_TIME)
    {
      programe.interval_time = 0;
      if(programe.item < (SPEED_NUM_PERPROGRAME - 1))
      {
        programe.item ++;
        treadmills.param.speed =
            uart_prog_kmh_to_internal(prog_tab[programe.index].speed[programe.item]);
        {
          u8 sd = (u8)(treadmills.param.speed / 10u);
          uart_frame_tx(CMD_SPEED, sd);
          uart_frame_tx(CMD_SPEED, sd);
        }
				beep_set(2,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
      }
    }
		/*
		 * distance = V*t = (((float)treadmills.param.speed/100.0)*1000.0)/(3600.0)
		 */
		treadmills.param.distance += ((float)(treadmills.param.speed) / 1.6)/360.0;//米/秒  单位：米
		distance = treadmills.param.distance;

		treadmills.param.calorie = distance * 70;  //80千卡/千米 = 0.080K/M  单位:卡
		if(treadmills.param.calorie >= CALORIE_MAX ){treadmills.param.calorie = CALORIE_MAX -1;}
	}
	if (treadmills.status == STATUS_RUNNING) {
		treadmills_display(treadmills.display.index);
	}
}
//时基 100ms
void treadmills_proc_loop(void)
{
	switch(treadmills.status)
	{
    case STATUS_POWERON:
      proc_loop_cnt++;   
      if(proc_loop_cnt>1)//10
      {
        proc_loop_cnt=0;
        programe_init();  
        indecate.led_speed=LED_OFF;
        indecate.led_distance=LED_OFF;
        indecate.led_calorie=LED_OFF;
        indecate.led_auto=LED_OFF;
        indecate.led_time=LED_OFF;
        indecate.led_updp=LED_OFF;
        indecate.led_downdp=LED_OFF;
        
        set_bit_seg(0,SEG_ALL_OFF);//上电全亮后显示OFF
        set_bit_seg(1,SEG_O);
        set_bit_seg(2,SEG_F);
        set_bit_seg(3,SEG_F);
        
        treadmills.param.speed=0;
        treadmills.param.second=0;
        treadmills.param.distance = 0;
        treadmills.param.calorie = 0;

        treadmills.status=STATUS_WAIT;
        key_mask= 0xFF;
        set_seg_mode(SEG_MODE_NORMAL);
				uart_frame_tx(CMD_STOP_BURST,5);
				uart_frame_tx(CMD_STOP_BURST,5);
      }
      break;
    case STATUS_WAIT:
      break;
    case STATUS_SET_PROGRAM:
      break;
    case STATUS_SET_PARAM:
      break;
    case STATUS_RUN:		    
      indecate.led_updp=LED_OFF;
      indecate.led_downdp=LED_OFF;
      set_seg_mode(SEG_MODE_NORMAL);
      if(proc_loop_cnt==0)
      {
        if (param.index == PARAM_SET_PROGRAME || programe.use_prog_speed != 0u ||
            programe.run_use_tab_speed != 0u) {
          programe.interval_time = 0;
          programe.item          = 0;
        }
        set_bit_seg(0, SEG_ALL_OFF);
        set_bit_seg(1, SEG_ALL_OFF);
        set_bit_seg(2, SEG_ALL_OFF);
        set_bit_seg(3, run_disp_tab[proc_loop_val]);
        proc_loop_cnt=1;
        proc_loop_val=1;
      }
      else
      {
        proc_loop_cnt++;
         if(proc_loop_cnt>10)
        {
          proc_loop_cnt=1;       
          BEEP_SWITCH_ON_1_CNT();  //2023.3.27

          if(proc_loop_val<=2)
          {
            set_bit_seg(0, SEG_ALL_OFF);
            set_bit_seg(1, SEG_ALL_OFF);
            set_bit_seg(2, SEG_ALL_OFF);
            set_bit_seg(3, run_disp_tab[proc_loop_val]);
            proc_loop_val++;
          }
          else
          {
            proc_loop_val=0;
            {
              u8  use_tab = (param.index == PARAM_SET_PROGRAME || programe.use_prog_speed != 0u ||
                             programe.run_use_tab_speed != 0u);
              u16 start_spd;

              if (use_tab != 0u) {
                programe.item          = 0;
                programe.interval_time = 0;
                start_spd =
                    uart_prog_kmh_to_internal(prog_tab[programe.index].speed[programe.item]);
                treadmills.param.second = programe.total_time;
                programe.session_is_program = 1;
              } else {
                start_spd                 = DEFAULT_SPEED_MIN;
                programe.session_is_program = 0;
              }
              programe.run_use_tab_speed = 0;

              /* 显示目标档；下发先 0.6 进 RUN，延时后与段切换相同走 CMD_SPEED */
              treadmills.param.speed = start_spd;
              if (use_tab != 0u) {
                uart_frame_motor_run_internal_spd(DEFAULT_SPEED_MIN);
                uart_frame_motor_run_internal_spd(DEFAULT_SPEED_MIN);
                s_prog_startup_cmd_speed_delay = PROG_START_CMD_SPEED_DELAY_TICKS;
              } else {
                s_prog_startup_cmd_speed_delay = 0;
                uart_frame_motor_run_internal_spd(start_spd);
                uart_frame_motor_run_internal_spd(start_spd);
              }
            }
            
            treadmills.status = STATUS_RUNNING;
            treadmills.display.mode = DISP_MODE_AUTO; 
            treadmills.display.index = PARAM_SET_SPEED_ADC;//速度档
            treadmills_display(treadmills.display.index);
          }
        }
      }
		break;
	case STATUS_RUNNING:
		if (s_prog_startup_cmd_speed_delay > 0u) {
			s_prog_startup_cmd_speed_delay--;
			if (s_prog_startup_cmd_speed_delay == 0u) {
				u8 sd = (u8)(treadmills.param.speed / 10u);
				uart_frame_tx(CMD_SPEED, sd);
				uart_frame_tx(CMD_SPEED, sd);
			}
		}
		break;
	case STATUS_STOP:
		indecate.led_auto=LED_OFF;
		if (treadmills_natural_stop_hold_off == 0u) {
			treadmills.display.index = PARAM_SET_SPEED_ADC;
			treadmills.display.mode  = IDEL;
			treadmills_display(PARAM_SET_SPEED_ADC);
		}
		treadmills.param.second=0;
		treadmills.param.distance=0;
		treadmills.param.calorie=0;
		if (param.index == PARAM_SET_PROGRAME || programe.use_prog_speed != 0u ||
		    programe.session_is_program != 0u) {
			programe.interval_time = 0;
			programe.item = 0;
		}

		uart_frame_tx(CMD_STATUS,STATUS_MOTOR_STOP);
		uart_frame_tx(CMD_STATUS,STATUS_MOTOR_STOP);

		s_prog_startup_cmd_speed_delay = 0;
    param.book_en=0;
    param.index=0;
    programe.use_prog_speed = 0;
    programe.session_is_program = 0;
    programe.run_use_tab_speed = 0;
    proc_loop_cnt=0;
    proc_loop_timeout=0;
    treadmills.status=STATUS_STOP_WAIT;

		break;
	case STATUS_STOP_WAIT:
		proc_loop_timeout++;
		if(proc_loop_timeout>STOP_WAIT_TIMEOUT)//超时退出 5*5=25   //100
		{
			proc_loop_timeout=0;
			treadmills.status=STATUS_STOP_OVER;
		}
		break;
	case STATUS_STOP_OVER:
	uart_frame_tx(CMD_STOP_BURST,5);
	uart_frame_tx(CMD_STOP_BURST,5);
		proc_loop_cnt++;
		if(proc_loop_cnt>1)//5=约1秒
		{
			treadmills.display.mode=IDEL;
			treadmills.param.speed=0;
			treadmills.param.second=0;
			treadmills.param.distance=0;
			treadmills.param.calorie=0;
			treadmills_ui_apply_off_segments();
			treadmills.status=STATUS_POWERON;	
      
			key_mask = 0xFF;
			treadmills_natural_stop_hold_off = 0;
			param.book_en=0;
			param.index=0;
			proc_loop_cnt=0;
			proc_loop_val=0;
			s_prog_startup_cmd_speed_delay = 0;
			beep_force_stop();
			treadmills_on_enter_poweron();
		}
		break;
	case STATUS_PAUSE:
		treadmills.status=STATUS_POWERON;
		proc_loop_cnt=0;
		proc_loop_val=0;
		s_prog_startup_cmd_speed_delay = 0;
		beep_force_stop();
		break;
	case STATUS_ERROR:
		proc_loop_val=0;
		proc_loop_cnt=0;
		s_prog_startup_cmd_speed_delay = 0;
		key_mask=0x00;//关闭所有的按键功能
		set_bit_seg(0, SEG_ALL_OFF);
		set_bit_seg(1, SEG_E);
		set_bit_seg(2, seg_num[(treadmills.error_code / 10u) % 10u]);
		set_bit_seg(3, seg_num[treadmills.error_code % 10u]);

		indecate.led_speed=LED_OFF;
		indecate.led_distance=LED_OFF;
		indecate.led_calorie=LED_OFF;
		indecate.led_auto=LED_OFF;
		indecate.led_time=LED_OFF;
		indecate.led_updp=LED_OFF;
		indecate.led_downdp=LED_OFF;
	  treadmills.status=IDEL;
	  treadmills.display.mode=IDEL;
	  treadmills.status=IDEL;

		if(tl_flag==0)
		{
			tl_flag=2;
			beep_set(20, ERROR_ON_TICKS, ERROR_OFF_TICKS);//改这里，这里增加蜂鸣器时间
		}
		
    if(error_times==0)
    {
      uart_frame_tx(CMD_STATUS,STATUS_MOTOR_STOP);
      uart_frame_tx(CMD_STATUS,STATUS_MOTOR_STOP);
    }    
    error_times++;
    if(error_times>50)
    {
      treadmills.status=IDEL;
      error_times=50;
    }
      break;
    default:
      proc_loop_cnt=0;
      break;
  }
}

//显示
void treadmills_display (u8 index)
{
	u8 high,low;
  u8 n=0;
  u16 SPD_VAL;
	u32 DIS_VAL;
	if(index == PARAM_SET_SPEED_ADC)//0显示速度 km/hour
  {
    indecate.led_speed=LED_ON;
    indecate.led_distance=LED_OFF;
    indecate.led_calorie=LED_OFF;
    //indecate.led_auto=LED_OFF;
    indecate.led_time=LED_OFF;
    indecate.led_updp=LED_OFF;
    indecate.led_downdp=LED_OFF;

    SPD_VAL  = (treadmills.param.speed) / 16; 
    
    //直接显示实际速度    
    high = SPD_VAL/10;
    low = SPD_VAL%10;
    
    if(SPD_VAL < 1000)
    {
      set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示
      set_bit_seg(3,SEG_ALL_OFF);//强制关闭显示
      set_seg_two_spd(high,low);
      set_dp2();   //加小数点

    }
    else
    {
      set_seg_two_spd(high,low);
      set_dp2();   //加小数点
//      set_bit_seg(3,SEG_ALL_OFF);//强制关闭显示
    }
  } 
  
  else if(index == PARAM_SET_TIME)//时间:mm:ss
  {
    indecate.led_speed=LED_OFF;
    indecate.led_distance=LED_OFF;
    indecate.led_calorie=LED_OFF;
//    indecate.led_auto=LED_OFF;
    indecate.led_time=LED_ON;
    indecate.led_updp=LED_ON;
    indecate.led_downdp=LED_ON;
        
    high=(treadmills.param.second/60)%100;
    low=treadmills.param.second%60;
    
    if(treadmills.param.second<600)
    {
      set_seg_two(high,low);
      set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示
    }
    else
    {
      set_seg_two(high,low);
    }
    
    if((param.index==PARAM_SET_PROGRAME || programe.use_prog_speed != 0u ||
        programe.session_is_program != 0u)&&treadmills.status!=STATUS_RUNNING)
		{
				high=((programe.total_time/60)%100);
				low=programe.total_time%60;
		}
  }
  
  else if(index == PARAM_SET_DISTANCE)//路程 km
  {
    indecate.led_speed=LED_OFF;
    indecate.led_distance=LED_ON;
    indecate.led_calorie=LED_OFF;
    indecate.led_time=LED_OFF;
    indecate.led_updp=LED_OFF;
    indecate.led_downdp=LED_ON;
          
    //实际数据
    DIS_VAL = (treadmills.param.distance);  
    
    high=((u32)DIS_VAL/1000);   // 1000
    low=((u32)DIS_VAL/10)%100;  // /10

    if(DIS_VAL<=9999)
    {
      set_seg_two(high,low);
      set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示
      set_dp1();       //小数点
    }
    else
    {
      set_seg_two(high,low);
      set_dp1();       //小数点
    }
    clr_dp2();
  }
  else if(index == PARAM_SET_CALORIE )//卡洛里 kcal
  {
    indecate.led_speed=LED_OFF;
    indecate.led_distance=LED_OFF;
    indecate.led_calorie=LED_ON;
    indecate.led_time=LED_OFF;
    indecate.led_updp=LED_OFF;
    indecate.led_downdp=LED_OFF;

    if(treadmills.param.calorie<=99999)//=99.99千卡
    {
      high=(treadmills.param.calorie/1000);
      low=(treadmills.param.calorie%1000)/10;
      set_seg_two_sc(high,low);
    }
    else if((treadmills.param.calorie>=100000)&&(treadmills.param.calorie<=999000))
    {
      high=(treadmills.param.calorie/1000)/10;
      low=((treadmills.param.calorie/100)%100);
      set_seg_two(high,low);
    }        
    set_dp2();
    
    if(treadmills.param.calorie<=99999)//=99.99千卡
    {
      if(treadmills.param.calorie<=9999)
      {
        set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示 
        set_bit_seg(1,SEG_ALL_OFF);//强制关闭显示 
      }
      set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示    
    }
  }
  else if(index==PARAM_SET_PROGRAME)//PROGRAM 程序
  {
    indecate.led_speed=LED_OFF;
    indecate.led_distance=LED_OFF;
    indecate.led_calorie=LED_OFF;
    indecate.led_time=LED_OFF;
    indecate.led_updp=LED_OFF;
    indecate.led_downdp=LED_OFF;
    n = programe.index + 1;
		if(n > 12) n = 1;
		set_bit_seg(0,SEG_P);
		set_bit_seg(1,seg_num[(n)/10]);
		set_bit_seg(2,seg_num[(n)%10]);
		set_bit_seg(3,SEG_ALL_OFF);		  
	}
}

//时基 100ms
void treadmills_disp_loop(void)
{
	static u8 cnt=0;
	switch(treadmills.display.mode)
	{
    case DISP_MODE_AUTO:
      cnt=0;
      treadmills.display.index=0;
      treadmills_display(treadmills.display.index);
      treadmills.display.mode=DISP_MODE_TO_AUTO;
      break;
    case DISP_MODE_TO_AUTO:
      cnt++;
      if(cnt>50)
      {
        cnt=0;
        treadmills.display.index++;  
        if(treadmills.display.index>=4){treadmills.display.index=0;}
        treadmills_display(treadmills.display.index);
      }
      break;
    case DISP_MODE_SINGLE:
      treadmills.display.index=0;
      treadmills_display(treadmills.display.index);
      treadmills.display.mode=DISP_MODE_TO_AUTO;
      break;
    default:
      break;
	}
}


void communication_checkout(void)
{
	static u8 timeout=0;
	static u8 sm=0;
	static u8 error_cnt=0;
	switch(sm)
	{
    case 0:
      treadmills.ack=0;
      uart_frame_tx(CMD_ACK,treadmills.ack);
      timeout=0;
      sm=1;
      break;
    case 1:
      timeout++;
      if(treadmills.ack==0)
      {
        uart_frame_tx(CMD_ACK,treadmills.ack);
        if(timeout>1)
        {
          timeout=0;
          error_cnt++;
          if(error_cnt>10)
          {
#if TM_ERROR_REPORT_ENABLE
            treadmills.error_code = ERROR_DISPLAY_COMM_LINK;
            treadmills.status=STATUS_ERROR;
            sm=2;
#else
            error_cnt=0;
            sm=0;
#endif
          }
          else
          {
            sm=0;
          }
        }
      }
      else
      {
        error_cnt=0;
        sm=0;
      }
      break;
    case 2:
      break;
    default:
      break;
	}
}

