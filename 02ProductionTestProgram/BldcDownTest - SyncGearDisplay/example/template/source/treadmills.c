/*
 * treadmills.c
 *
 *  Created on: 2019年12月12日
 *      Author: Administrator
 */
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


void treadmills_display (u8 index);

treadmills_t treadmills;

extern u8 tl_flag;
extern u8 set_param_en;
extern s8 param_speed;
u8 error_times=0;

void treadmills_init(void)
{
	treadmills.ack=0;
	treadmills.param.second = 0;
	treadmills.param.speed = 0;
	treadmills.param.distance = 0;
	treadmills.param.calorie = 0;
	treadmills.error_cnt = 0;
	treadmills.error_code = 0;
	treadmills.param.speed_step = DEFAULT_SPEED_STEP;
	treadmills.param.speed_max = DEFAULT_SPEED_MAX;
//  Bluetooth = 0;   //上电开启蓝牙
	treadmills.param.beep_status = BEEP_SWITCH_ON;  //0表示蜂鸣器开，1表示关 
	treadmills.display.gear_shown = 0;
}

static const u8 boot_disp_tab[]=
{
#if(0)
		SEG_UP_,
		SEG_DWN_,
		SEG__
#else
		NUM_0,
		NUM_0,
		NUM_0,
#endif
};


static const u8 run_disp_tab[]={NUM_3,NUM_2,NUM_1};

u8 booktime_en=0;
u8 booktime_cnt=0;
//时基 10ms
void boot_time(void)
{
	if(treadmills.status != STATUS_ERROR&&treadmills.status != IDEL)
	{
		if(booktime_en == 0){booktime_cnt = 0;return;}
		booktime_cnt++;
		if(booktime_cnt > STOP_DISPLAY_INTERVAL)//18
		{
			booktime_cnt = 0;
			if(treadmills.param.speed >= DEFAULT_SPEED_SINGLE)
			{
				treadmills.param.speed -= DEFAULT_SPEED_SINGLE;
			}
			else
			{
				treadmills.param.speed = 0;
			}
			treadmills.display.index = PARAM_SET_SPEED_ADC;//速度档
			treadmills_display(treadmills.display.index);

			if(treadmills.param.speed == 0)
			{
				treadmills.status = STATUS_POWERON;
				booktime_cnt = 0;
				booktime_en = 0;
			}
		}
	}
}
 

//时基 100ms
void treadmills_proc_loop(void)
{
	static u8 cnt=0,timeout=0;
    static u8 val=0;
    static u8 proc_prev_status = 0;

    if ((treadmills.status == STATUS_RUN) && (proc_prev_status != STATUS_RUN)) {
		cnt = 0;
		val = 0;
	}
	proc_prev_status = treadmills.status;

	switch(treadmills.status)
	{
	case STATUS_POWERON:
		cnt++;
	
		if(cnt>1)//10
			{        
        	cnt = 0;

          indecate.led_speed = LED_OFF;
          indecate.led_distance = LED_OFF;
          indecate.led_calorie = LED_OFF;
//          indecate.led_auto=LED_OFF;
          indecate.led_time = LED_ON;
          indecate.led_updp = LED_ON;
          indecate.led_downdp = LED_ON;

        	set_bit_seg(0,boot_disp_tab[val]);
        	set_bit_seg(1,boot_disp_tab[val]);
        	set_bit_seg(2,boot_disp_tab[val]);
        	set_bit_seg(3,boot_disp_tab[val]);

        	treadmills.display.index = PARAM_SET_TIME;//时间档位
//        	treadmills.param.second=0;
        	treadmills_display(treadmills.display.index);
        	treadmills.status = STATUS_WAIT;			
			
//			if(treadmills.lower_status == 0)
//			{
				key_mask = BTN_ONOFF |BTN_MODE; // |BTN_SLEEP;
//			}	
        }
		break;
	case STATUS_WAIT:
//			treadmills.display.mode = DISP_MODE_TO_AUTO;		
		break;
  case STATUS_SET_PARAM:
    break;
	case STATUS_RUN:		
    
    indecate.led_speed=LED_OFF;
    indecate.led_distance=LED_OFF;
    indecate.led_calorie=LED_OFF;
//    indecate.led_auto=LED_OFF;
    indecate.led_time=LED_OFF;
    indecate.led_updp=LED_OFF;
    indecate.led_downdp=LED_OFF;
		if(cnt==0)
		{
      BEEP_SWITCH_ON_1_CNT();  //2023.3.27
      set_bit_seg(0,SEG_ALL_OFF);
      set_bit_seg(1,run_disp_tab[val]);
      set_bit_seg(2,SEG_ALL_OFF);
      set_bit_seg(3,SEG_ALL_OFF);
      cnt=1;
      val=1;
		}
		else
		{
			cnt++;
			 if(cnt>10)
			{
        cnt=1;       
        BEEP_SWITCH_ON_1_CNT();  //2023.3.27

        if(val<=2)
        {
          set_bit_seg(0,SEG_ALL_OFF);
          set_bit_seg(1,run_disp_tab[val]);
          set_bit_seg(2,SEG_ALL_OFF);
          set_bit_seg(3,SEG_ALL_OFF);
          val++;
        }
		else
		{
			val=0;
			/* hold max gear from lower (do not overwrite with DEFAULT_SPEED_MIN) */
			uart_frame_tx(0x02, 0x00, 0x00);
			uart_frame_tx(0x02, 0x00, 0x00);
			uart_frame_tx_d1(0x03, 0x00, treadmills.param.speed);
			if(treadmills.lower_status != 9)
			{
				uart_frame_tx(0x02, 0x00, 0x00);
			}

			treadmills.status = STATUS_RUNNING;
			treadmills.display.gear_shown = (u16)DEFAULT_SPEED_MIN; /* 显示从 1.0 档起爬升 */
			treadmills.display.index = PARAM_SET_SPEED_ADC;
			treadmills_display(treadmills.display.index);  
			treadmills.display.mode = DISP_MODE_GEAR_HOLD;	
	
		}
			}
		}       
		indecate.led_updp=LED_OFF;
		indecate.led_downdp=LED_OFF;
		break;
	case STATUS_RUNNING:
		 if (treadmills.param.speed != treadmills.param.fbk_speed) {
            uart_frame_tx_d1(0x03, 0x00, treadmills.param.speed);
        }
		break;
	case STATUS_STOP:
		treadmills.display.mode = IDEL;
//		treadmills.param.speed = 0;
//		uart_frame_tx_4byte(0);
//		uart_frame_tx(CMD_STATUS,STATUS_MOTOR_STOP);
//		uart_frame_tx(CMD_STATUS,STATUS_MOTOR_STOP);
		uart_frame_tx(0x02, 0x01, 0x00); // 停止
		uart_frame_tx(0x02, 0x01, 0x00);
		booktime_cnt = 0;
		booktime_en = 1; 
		param.book_en = 0;
		param.index = 0;
		cnt = 0;
		timeout=0;
		treadmills.status = STATUS_STOP_WAIT;

		break;
	case STATUS_STOP_WAIT:
		timeout++;
		if(timeout > STOP_WAIT_TIMEOUT)//超时退出 5*5=25   //100
		{
			timeout=0;
			treadmills.status = STATUS_STOP_OVER;
		}
		if(treadmills.lower_status != 9)
		{
			treadmills.status = STATUS_STOP_OVER;
		}
		else
		{
			uart_frame_tx(0x02, 0x01, 0x00);
		}
		break;
	case STATUS_STOP_OVER:
			treadmills.display.mode = IDEL;
			treadmills.display.gear_shown = 0;
			treadmills.param.speed = 0;
			if(((treadmills.param.second >= 6000)||(param.index!=0))||(PAUSE_FLAG == 0))    //运行100分钟停止
			{
				treadmills.param.second = 0;
				treadmills.param.distance = 0;
				treadmills.param.calorie = 0;
				treadmills.display.index = PARAM_SET_TIME;//1 
				treadmills_display(treadmills.display.index);
				set_seg_mode(SEG_MODE_NORMAL); 
				treadmills.status = STATUS_POWERON;
			}
			else if(PAUSE_FLAG == 1)//按暂停键，则进入暂停模式
			{
				set_bit_seg(0, SEG_P);
				set_bit_seg(1, SEG_U);
				set_bit_seg(2, SEG_A);
				set_bit_seg(3, SEG_ALL_OFF);
//				treadmills.display.index = PARAM_SET_TIME;//1 
//				treadmills_display(treadmills.display.index);
				treadmills.status = STATUS_PAUSE;
			}
			booktime_en=0;
			param.book_en=0;
			param.index=0;	
			cnt=0;
			val=0;
			beep_force_stop();
		break;
	case STATUS_PAUSE:
		treadmills.display.mode = IDEL;
		indecate.led_speed=LED_OFF;
		indecate.led_distance=LED_OFF;
		indecate.led_calorie=LED_OFF;
		indecate.led_time=LED_OFF;
		indecate.led_updp=LED_OFF;
		indecate.led_downdp=LED_OFF;
    key_mask = BTN_ONOFF; // BTN_SLEEP|
    booktime_en=0;
    cnt=0;
    val=0;
		set_seg_mode(SEG_MODE_NORMAL);
    beep_force_stop();
		break;      
  case STATUS_SLEEP_MODE:	
//    treadmills.param.second=0;
//		treadmills.param.distance=0;
//		treadmills.param.calorie=0;
		treadmills.display.mode=IDEL;
		indecate.led_speed=LED_OFF;
		indecate.led_distance=LED_OFF;
		indecate.led_calorie=LED_OFF;
		indecate.led_time=LED_OFF;
		indecate.led_updp=LED_OFF;   
		indecate.led_downdp=LED_OFF;
	
    set_bit_seg(0,SEG_ALL_OFF);
    set_bit_seg(1,SEG_ALL_OFF);       
    set_bit_seg(2,SEG_ALL_OFF);
    set_bit_seg(3,SEG_ALL_OFF);
    key_mask =0xff;  
    set_seg_mode(SEG_MODE_NORMAL);
		break;     
	case STATUS_ERROR:
		val=0;
		cnt=0;
		key_mask = 0x00;																		//关闭所有的按键功能
    
	u16 err1 = treadmills.error_code;
    u8  err2 = treadmills.error_sub_code;
    u8 display_num = 0; 

    if      (err1 & (1 << 12)) display_num = 13; // 4096: 零偏异常
    else if (err1 & (1 << 11)) display_num = 12; // 2048: 电机过载
    else if (err1 & (1 << 10)) display_num = 11; // 1024: 电机缺相
    else if (err1 & (1 << 9))  display_num = 10; // 512:  MOS损坏
    else if (err1 & (1 << 8))  display_num = 9;  // 256:  速度反转
    else if (err1 & (1 << 7))  display_num = 8;  // 128:  速度过冲
    else if (err1 & (1 << 6))  display_num = 7;  // 64:   堵转故障
    else if (err1 & (1 << 5))  display_num = 6;  // 32:   软件过流
    else if (err1 & (1 << 4))  display_num = 5;  // 16:   硬件过流
    else if (err1 & (1 << 3))  display_num = 4;  // 8:    12V欠压
    else if (err1 & (1 << 2))  display_num = 3;  // 4:    12V过压
    else if (err1 & (1 << 1))  display_num = 2;  // 2:    母线欠压
    else if (err1 & (1 << 0))  display_num = 1;  // 1:    母线过压
        
		else if (err2 & (1 << 0))  display_num = 14; // 1:    串口丢失 (E14)
    else if (err2 & (1 << 1))  display_num = 15; // 2:    过温保护 (E15)

    if (display_num == 0) display_num = 99; // 未知错误导致位匹配失败，强制显示 E99 报警

    // 数码管刷新
    set_bit_seg(0, SEG_E); // 固定显示 E
    set_bit_seg(1, seg_num[display_num / 10]); // 十位
    set_bit_seg(2, seg_num[display_num % 10]); // 个位
    set_bit_seg(3, 0x00);
    set_seg_mode(SEG_MODE_NORMAL);

		indecate.led_speed = LED_OFF;
		indecate.led_distance = LED_OFF;
		indecate.led_calorie = LED_OFF;
	//		indecate.led_auto=LED_OFF;
		indecate.led_time = LED_OFF;
		indecate.led_updp = LED_OFF;
		indecate.led_downdp = LED_OFF;
		treadmills.status = IDEL;
		treadmills.display.mode = IDEL;
		treadmills.status = IDEL;
		set_seg_mode(SEG_MODE_NORMAL);
    
//	    beep_set(1,ERROR_ON_TICKS,ERROR_OFF_TICKS);
	
		if(tl_flag == 0)
		{
      tl_flag = 1;
			beep_set(10, ERROR_ON_TICKS, ERROR_OFF_TICKS);//改这里，这里增加蜂鸣器时间
		}
		
		if(error_times == 0)
		{
			treadmills.param.speed = 0;
			uart_frame_tx(0x02, 0x01, 0x00);
	//		uart_frame_tx_4byte(0);
	//		uart_frame_tx(CMD_STATUS,STATUS_MOTOR_STOP);
	//		uart_frame_tx(CMD_STATUS,STATUS_MOTOR_STOP);
		}
		
		error_times++;
		if(error_times > 50)
		{
			treadmills.status = IDEL;
			error_times = 50;
		}
//	else
//	{
//	beep_set(1,ERROR_ON_TICKS,ERROR_OFF_TICKS);
//	}
	
		break;
	default:
		cnt=0;
		break;
	}
}


//参数计算设置  1000ms
void treadmills_param_calc(void)
{
  static float distance=0;
	if(treadmills.status != STATUS_RUNNING){distance=0;return;}

	if(param.index == PARAM_SET_TIME)
	{
		
		// if(treadmills.status!=STATUS_ERROR)
		// {
			if(treadmills.param.second){treadmills.param.second--;}
			else
			{
				treadmills.status = STATUS_STOP;
				key_mask = 0x00;
				beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
			}
		// }else
		// {
		// 	treadmills.param.second=0;
		// 	key_mask =0x00;
		// }
	}
	else
	{
		treadmills.param.second++;
		//增加定时功能---必须和键盘统一
		if(treadmills.param.second >= 6000)//
		{
			treadmills.status = STATUS_STOP;
			key_mask = 0x00;
      beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
		}
	}


	if(param.index == PARAM_SET_DISTANCE)
	{
		distance += ((float)(treadmills.param.speed)*10)/360.0;//米/秒  单位：米

		treadmills.param.distance -= ((float)(treadmills.param.speed)*10)/360.0;//米/秒  单位：米

		if(treadmills.param.distance < 10)
		{
			treadmills.status = STATUS_STOP;
			key_mask = 0x00;
      beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
		}
	}
	else
	{
		/**
		* distance=V*t
		*         =(((float)treadmills.param.speed/100.0)*1000.0)/(3600.0)
		*/
		treadmills.param.distance += ((float)(treadmills.param.speed)*10)/360.0;//米/秒  单位：米
		distance = treadmills.param.distance;
	}

	if(param.index==PARAM_SET_CALORIE)
	{
		if(treadmills.param.calorie>=(((float)(treadmills.param.speed)*10)/360.0)*112)  //  80*0.625=50，80千卡/英里
		{
			treadmills.param.calorie-=(((float)(treadmills.param.speed)*10)/360.0)*112;
		}
		else
		{
			treadmills.param.calorie = 0;
			treadmills.status = STATUS_STOP;
			key_mask = 0x00;
      beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
		}
	}
	else
	{
		treadmills.param.calorie = distance * 70;//70千卡/千米 =0.070K/M  单位:卡
	}

	treadmills_display(treadmills.display.index);
}


//显示
void treadmills_display (u8 index)
{
	u8 fitst_time=0;
  unsigned char H[]={0,1,2,3,4,5,6};
  unsigned char L[]={0,10,20,30,40,50,60,70,80,90};
	u32 high,low;
  u8 beep_status;
  float SPD_VAL;
	float DIS_VAL;
  float TIME_VAL;
	float T_VAL;
	if(index == PARAM_SET_SPEED_ADC)//显示速度 km/hour
  {
    u16 spd_for_disp;
    indecate.led_speed = LED_ON;
    indecate.led_distance = LED_OFF;
    indecate.led_calorie = LED_OFF;
    //indecate.led_auto=LED_OFF;
    indecate.led_time = LED_OFF;
    indecate.led_updp = LED_OFF;
    indecate.led_downdp = LED_OFF;
    set_seg_mode(SEG_MODE_NORMAL);

    spd_for_disp = treadmills.param.speed;
    if (treadmills.display.mode == DISP_MODE_GEAR_HOLD
        && treadmills.status == STATUS_RUNNING) {
      if (treadmills.display.gear_shown != 0U) {
        spd_for_disp = treadmills.display.gear_shown;
      }
    }

    SPD_VAL = spd_for_disp * 10;  		
    high = (u32)SPD_VAL/100;
    low = (u32)SPD_VAL%100;
		set_seg_two_sp(high,low);
    if(SPD_VAL<1000)
		{
			set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示
			set_bit_seg(1,SEG_ALL_OFF);//强制关闭显示
		}
		else
		{
			set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示
		}
		set_dp2(); 
  }  
  else if(index == PARAM_SET_TIME)//时间:mm:ss
  {
    indecate.led_speed=LED_OFF;
    indecate.led_distance=LED_OFF;
    indecate.led_calorie=LED_OFF;
    //indecate.led_auto=LED_OFF;
    indecate.led_time=LED_ON;
//    indecate.led_updp=LED_ON;
//    indecate.led_downdp=LED_ON;
    set_seg_mode(SEG_MODE_FLASH_COLON);
    
    TIME_VAL = treadmills.param.second;
    high = ((u32)TIME_VAL/60)%100;
    low = (u32)TIME_VAL%60;
    
    if(TIME_VAL < 600)
    {
      set_seg_two(high,low);
      set_bit_seg(0, SEG_ALL_OFF);//强制关闭显示
    }
    else
    {
      set_seg_two(high, low);
    }
  }
  else if(index == PARAM_SET_DISTANCE)//路程 km
  {
    indecate.led_speed=LED_OFF;
    indecate.led_distance=LED_ON;
    indecate.led_calorie=LED_OFF;
//    indecate.led_auto=LED_OFF;
    indecate.led_time=LED_OFF;
    indecate.led_updp=LED_OFF;
    indecate.led_downdp=LED_OFF;
    set_seg_mode(SEG_MODE_NORMAL);
    //公里路程数据
    DIS_VAL=(treadmills.param.distance);

//    if(DIS_VAL>DISTANCE_MAX)
//		{
//			DIS_VAL=DISTANCE_MIN;
//		}
    high = (u32)DIS_VAL /1000;                 // 整数部分
    low = ((u32)DIS_VAL/10)%100; // 小数部分，保留两位数
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
  }
  else if(index == PARAM_SET_CALORIE)//卡洛里 kcal
  {
    indecate.led_speed=LED_OFF;
    indecate.led_distance=LED_OFF;
    indecate.led_calorie=LED_ON;
    //indecate.led_auto=LED_OFF;
    indecate.led_time=LED_OFF;
    indecate.led_updp=LED_OFF;
    indecate.led_downdp=LED_OFF;
		set_seg_mode(SEG_MODE_NORMAL);		
		
		//treadmills.param.calorie//单位：卡
//		if(treadmills.param.calorie>=CALORIE_MAX)
//    {
//			treadmills.param.calorie=CALORIE_MIN;
//		}
    
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
			set_seg_two_sp(high,low);
		}
        
//    set_dp2();
    
    if(treadmills.param.calorie < 1000000)//=99.99千卡
		{
			if(treadmills.param.calorie < 100000)
			{
				if(treadmills.param.calorie < 10000)
				{
					set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示 
					set_bit_seg(1,SEG_ALL_OFF);//强制关闭显示 
					set_bit_seg(2,SEG_ALL_OFF);//强制关闭显示 
				}
				set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示 
				set_bit_seg(1,SEG_ALL_OFF);//强制关闭显示 
			}
			set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示    
		}
  }
	else if (index == PARAM_SET_TEMP)
	{
		indecate.led_speed=LED_ON;
    indecate.led_distance=LED_ON;
    indecate.led_calorie=LED_ON;
    //indecate.led_auto=LED_OFF;
    indecate.led_time=LED_ON;
    indecate.led_updp=LED_OFF;
    indecate.led_downdp=LED_OFF;
		set_seg_mode(SEG_MODE_NORMAL);
		
		T_VAL = treadmills.param.temp;
		
		set_seg_temp(T_VAL);
		
	}
}


static const u8 disp_seq[] = {0, 1, 2, 3}; 
#define SEQ_LEN (sizeof(disp_seq) / sizeof(disp_seq[0]))
	
//时基 100ms
void treadmills_disp_loop(void)
{
	static u8 cnt=0;
	switch(treadmills.display.mode)
	{
	case DISP_MODE_AUTO:
		cnt=0;
		treadmills.display.index=PARAM_SET_SPEED_ADC;		//显示速度挡
		treadmills_display(treadmills.display.index);
		treadmills.display.mode=DISP_MODE_TO_AUTO;
		break;
	case DISP_MODE_TO_AUTO:
		cnt++;
		if(cnt > 50)
		{
			cnt = 0;
			if ((treadmills.status != STATUS_SET_PARAM)
				&& (treadmills.status != STATUS_RUN)
				&& (treadmills.status != STATUS_RUNNING)) 
			{
					u8 i, next_idx = 0;
					u8 current = treadmills.display.index;
				
					for (i = 0; i < SEQ_LEN; i++) {
							if (current == disp_seq[i]) {
									next_idx = disp_seq[(i + 1) % SEQ_LEN];
									break;
							}
					}
					treadmills.display.index = next_idx;
					treadmills_display(treadmills.display.index);
			}
		}
		break;
	case DISP_MODE_GEAR_HOLD:
		treadmills.display.index = PARAM_SET_SPEED_ADC;
		if (treadmills.status == STATUS_RUNNING) {
			u16 target = treadmills.param.fbk_speed;
			if (target == 0U) {
				target = treadmills.param.speed;
			}
			if (target < (u16)DEFAULT_SPEED_MIN) {
				target = (u16)DEFAULT_SPEED_MIN;
			}
			if (treadmills.param.speed_max != 0U && target > treadmills.param.speed_max) {
				target = treadmills.param.speed_max;
			}
			if (treadmills.display.gear_shown == 0U) {
				treadmills.display.gear_shown = (u16)DEFAULT_SPEED_MIN;
			}
			if (treadmills.display.gear_shown < target) {
				treadmills.display.gear_shown += DEFAULT_SPEED_SINGLE;
				if (treadmills.display.gear_shown > target) {
					treadmills.display.gear_shown = target;
				}
			} else if (treadmills.display.gear_shown > target) {
				treadmills.display.gear_shown -= DEFAULT_SPEED_SINGLE;
				if (treadmills.display.gear_shown < target) {
					treadmills.display.gear_shown = target;
				}
			}
		}
		treadmills_display(treadmills.display.index);
		break;
	case DISP_MODE_SINGLE:
		treadmills_display(treadmills.display.index);
		//treadmills.display.mode=IDEL;
    treadmills.display.mode=DISP_MODE_AUTO; 
		break;
	default:
		break;
	}
}

