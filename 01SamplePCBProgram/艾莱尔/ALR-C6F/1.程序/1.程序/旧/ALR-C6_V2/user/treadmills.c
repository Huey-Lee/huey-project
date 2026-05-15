#include "common.h"
#include "treadmills.h"
#include "keypad.h"
#include "led_disp.h"
#include "keypad_fun.h"
#include "uart_frame.h"
#include "indecate.h"
#include "7_segment.h"
#include "beep.h"
#include "uart.h"
#include "param.h"
#include "cfg.h"
#include "programe.h"


extern u8 tl_flag;
u8 error_times=0;
treadmills_t treadmills;
//bit STATUS_PAUSE_FLAG = 0;
//u8 time_delay=0;


//跑步机初始化
void treadmills_init(void)
{
	treadmills.ack=0;
	treadmills.param.second=0;
	treadmills.param.speed=0;
  treadmills.param.distance=0;
	treadmills.param.calorie=0;
  treadmills.param.runstep=0;
	treadmills.error_cnt=0;
	treadmills.error_code=0;
//	STATUS_PAUSE_FLAG = 0;
//  Bluetooth = 0;   //上电开启蓝牙
  treadmills.param.beep_status = BEEP_SWITCH_ON;  //0表示蜂鸣器开，1表示关
}


static const u8 boot_disp_tab[]={ NUM_0, NUM_0, NUM_0 };

//倒计时3，2，1
static const u8 run_disp_tab[]={ NUM_3, NUM_2, NUM_1};//NUM_5, NUM_4, 

u8 booktime_en=0;
u8 booktime_cnt=0;

//时基 10ms
void boot_time(void)  //速度倒计时
{
	if(booktime_en==0){booktime_cnt=0;return;}
	booktime_cnt++;
	if(booktime_cnt>STOP_DISPLAY_INTERVAL)//20
	{
		booktime_cnt=0;
		if(treadmills.param.speed>=10)
		{
			treadmills.param.speed-=10;
		}
		else
		{
			treadmills.param.speed=0;
		}
		treadmills.display.index=PARAM_SET_SPEED_ADC;//速度档
		treadmills_display(treadmills.display.index);
//		bt_panel_send_speed();//实时上报当前速度

		if(treadmills.param.speed==0)
		{
      treadmills.status=STATUS_STOP_OVER;
			booktime_cnt=0;
			booktime_en=0;
		}
	}
}


//参数计算设置  1000ms
void treadmills_param_calc(void)
{
  static float distance=0;
	if(treadmills.status !=STATUS_RUNNING){distance=0;return;}
 
  //设置时间
  if(param.index==PARAM_SET_TIME)//选择了模式，需要倒计
  {
    if(treadmills.param.second){treadmills.param.second--;}
    else
    {
      treadmills.status=STATUS_STOP;
      key_mask =0x00;
      beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
    }
  }
  else
  {
    treadmills.param.second++;
    //增加定时功能---必须和键盘统一
    if(treadmills.param.second>=Overflow_time)    //运行100分钟停止,6000
    {
      treadmills.status=STATUS_STOP;     
      key_mask =0x00;
      beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
    }
  }

  //设置距离  
  if(param.index==PARAM_SET_DISTANCE)//选择了模式，需要倒计
  {
    distance+=((float)(treadmills.param.speed))/360.0;//米/秒  单位：米
    treadmills.param.distance-=((float)(treadmills.param.speed))/360.0;//米/秒  单位：米

    if(treadmills.param.distance<10)
    {
      treadmills.status=STATUS_STOP;       
      key_mask =0x00;
      beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
    }
  }
  else
  {
    /**
    * distance=V*t
    *         =(((float)treadmills.param.speed/100.0)*1000.0)/(3600.0)
    */
    treadmills.param.distance+=((float)(treadmills.param.speed))/360.0;//米/秒  单位：米
    distance=treadmills.param.distance;
  }
  
    //设置卡路里
    if(param.index==PARAM_SET_CALORIE)//选择了模式，需要倒计
    {
      if(treadmills.param.calorie>=(((float)(treadmills.param.speed))/360.0)*70)//70千卡/千米 =0.070K/M  单位:卡
      {
        treadmills.param.calorie-=(((float)(treadmills.param.speed))/360.0)*70;
      }
      else
      {
        treadmills.param.calorie=0;
        treadmills.status=STATUS_STOP;
        key_mask =0x00;
        beep_set(STOP_BEEP_CNT,STOP_BEEP_ON_TICKS,STOP_BEEP_OFF_TICKS);
      }
    }
    else
    {
      treadmills.param.calorie=distance*70;//70千卡/千米 =0.070K/M  单位:卡
    }
 
	treadmills_display(treadmills.display.index);
}

u8 status_code;
//时基 100ms
void treadmills_proc_loop(void) //处理
{
	static u8 cnt=0,timeout=0;
  static u8 val=0;
	
	switch(treadmills.status)
	{
	case STATUS_POWERON:
		cnt++;
    if(cnt>1)//10
    {
      cnt=0;

      indecate.led_speed=LED_OFF;
      indecate.led_distance=LED_OFF;
      indecate.led_calorie=LED_OFF;
      indecate.led_step=LED_OFF;
      indecate.led_time=LED_OFF;
      indecate.led_updp=LED_OFF;
      
      set_bit_seg(0,SEG_ALL_OFF);//上电全亮后显示OFF
      set_bit_seg(1,SEG_O);
      set_bit_seg(2,SEG_F);
      set_bit_seg(3,SEG_F);
      set_seg_mode(SEG_MODE_NORMAL);
			
      treadmills.status=STATUS_WAIT;

      uart_tx_frame.cmd=CMD_READ_PARAM;
      uart_tx_frame.len=1;
      uart_tx_frame.buf[0]=0x00;
      UART_Send_Buf(UART1,(UINT8*)&uart_tx_frame,sizeof(uart_tx_frame));

      key_mask=0xff;
    }

		break;
	case STATUS_WAIT:  
		break;
	case STATUS_SET_PARAM:
    break;
	case STATUS_RUN:
		if(cnt==0)
		{
      beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
      set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示
      set_bit_seg(1,SEG_ALL_OFF);//强制关闭显示   
      set_bit_seg(3,run_disp_tab[val]);
      set_bit_seg(2,SEG_ALL_OFF);//强制关闭显示  
      set_seg_mode(SEG_MODE_NORMAL);
      cnt=1;
      val=1;
		}
		else
		{
			cnt++;
			if(cnt>10)
			{
        cnt=1;
        beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
        if(val<=2)  //启动倒计时次数
        {
          set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示
          set_bit_seg(1,SEG_ALL_OFF);//强制关闭显示 
          set_bit_seg(3,run_disp_tab[val]);
          set_bit_seg(2,SEG_ALL_OFF);//强制关闭显示                  
          
          val++;
        }
				else
				{
					val=0;
				  treadmills.param.speed=TREADMILLS_SPEED_MIN;
          
					uart1_frame_tx_2(CMD_STATUS,STATUS_MOTOR_RUN,treadmills.param.speed/10);
					uart1_frame_tx_2(CMD_STATUS,STATUS_MOTOR_RUN,treadmills.param.speed/10);
				
					treadmills.status = STATUS_RUNNING;
					treadmills.display.index=PARAM_SET_SPEED_ADC;//速度档
					treadmills.display.mode=DISP_MODE_AUTO; 
					treadmills_display(treadmills.display.index);
				}
			}
		}
    indecate.led_updp=LED_OFF;
    indecate.led_downdp=LED_OFF;  
		break;
	case STATUS_RUNNING:
		break;
	case STATUS_STOP: 		
		treadmills.display.mode=IDEL;
	
		uart1_frame_tx(CMD_STATUS,STATUS_MOTOR_STOP);
		uart1_frame_tx(CMD_STATUS,STATUS_MOTOR_STOP);
    set_seg_mode(SEG_MODE_NORMAL);
		param.index=0;     //初始化参数索引
    booktime_cnt=0;
    booktime_en=1;     //速度倒计时使能
    cnt=0;
    timeout=0;
    treadmills.status=STATUS_STOP_WAIT;
		break;
	case STATUS_STOP_WAIT:
		timeout++;
		if(timeout>STOP_WAIT_TIMEOUT)//超时退出 5*5=25   //100
		{
			timeout=0;
			treadmills.status=STATUS_STOP_OVER;    
		}
		break;
	case STATUS_STOP_OVER:
    treadmills.display.mode=IDEL;
    if((treadmills.param.second>=Overflow_time)||(PAUSE_FLAG == 0))    //运行100分钟停止
    {
      treadmills.param.speed=0;
			treadmills.param.second=0;
			treadmills.param.distance=0;
			treadmills.param.calorie=0;

			treadmills.status=STATUS_POWERON;//STATUS_WAIT;	
			set_seg_mode(SEG_MODE_NORMAL);
		}
		else if(PAUSE_FLAG == 1)//按暂停键，则进入暂停模式
    {
			treadmills.param.speed=0;
//      treadmills.display.index=1;//时间档
//			treadmills_display(treadmills.display.index);
//      set_seg_mode(SEG_MODE_FLASH_SELF);
      treadmills.status=STATUS_PAUSE;
    }

		booktime_en=0;
    param.index=0;
		cnt=0;
		val=0;
		beep_force_stop();
		break;
		
	case STATUS_PAUSE:
		treadmills.status=STATUS_POWERON;//STATUS_WAIT;
    booktime_en=0;
    cnt=0;
    val=0;
    beep_force_stop();  
		break;
  case STATUS_SLEEP_MODE:
		treadmills.display.mode=IDEL;
    treadmills.param.second=0;
		treadmills.param.distance=0;
		treadmills.param.calorie=0;
    treadmills.param.runstep=0;
  
		indecate.led_speed=LED_OFF;
		indecate.led_distance=LED_OFF;
		indecate.led_calorie=LED_OFF;
		indecate.led_time=LED_OFF;
		indecate.led_updp=LED_OFF;   

    set_bit_seg(0,SEG_ALL_OFF);
    set_bit_seg(1,SEG_ALL_OFF);       
    set_bit_seg(2,SEG_ALL_OFF);
    set_bit_seg(3,SEG_ALL_OFF);
    key_mask =0xff;  
		break;  
	case STATUS_FULL_DISPLAY:         
    indecate.led_speed=LED_ON;
    indecate.led_distance=LED_ON;
    indecate.led_calorie=LED_ON;
    indecate.led_time=LED_ON;
    indecate.led_updp=LED_ON;
    indecate.led_downdp=LED_ON;
    set_dp0();
    set_dp1();
    set_dp2();
    set_dp3();
    set_seg_val(8888);  
    break;
	case STATUS_ERROR:
		key_mask=0x00;//关闭所有的按键功能
		set_bit_seg(0,SEG_ALL_OFF);
		set_bit_seg(1,SEG_E);
		set_bit_seg(2,seg_num[0]);
		set_bit_seg(3,seg_num[treadmills.error_code]);
    
		indecate.led_speed=LED_OFF;
		indecate.led_distance=LED_OFF;
		indecate.led_calorie=LED_OFF;
		indecate.led_step=LED_OFF;
		indecate.led_time=LED_OFF;
		indecate.led_updp=LED_OFF;
		indecate.led_downdp=LED_OFF;
    clr_dp1();

	  treadmills.status=IDEL;
	  treadmills.display.mode=IDEL;

  if(error_times==0)
	{
		uart1_frame_tx(CMD_STATUS,STATUS_MOTOR_STOP);
		uart1_frame_tx(CMD_STATUS,STATUS_MOTOR_STOP);
	}
	error_times++;
	if(error_times>60)
	{
		treadmills.status=IDEL;
		error_times=60;
	}
    
	if(tl_flag==0)
		{
      tl_flag=1;
			beep_set(1, ERROR_ON_TICKS, ERROR_OFF_TICKS);//报错蜂鸣器响改这里，这里增加蜂鸣器蜂鸣器是一声持续长响
		}
		
		break;
	default:
		cnt=0;
  	val=0;
		break;
	}
}


void treadmills_display(index)   //显示模式
{
  unsigned char H[]={0,1,2,3,4};
  unsigned char L[]={0,10,20,30,40,50,60,70,80,90};
  u8 n=0;
	u8 high,low;
	float DIS_VAL;
 	float TIME_VAL;
	if(index==0)//显示速度 km/hour
  {
    indecate.led_speed=LED_ON;
    indecate.led_distance=LED_OFF;
    indecate.led_calorie=LED_OFF;
    indecate.led_step=LED_OFF;
    indecate.led_time=LED_OFF;
    indecate.led_updp=LED_OFF;
    indecate.led_downdp=LED_OFF;
    
    //显示公里制速度    
    high=treadmills.param.speed/100;
    low=treadmills.param.speed%100;   
    set_seg_two_sc(high,low);
    set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示
    set_bit_seg(1,SEG_ALL_OFF);//强制关闭显示
    set_dp2();       //小数点
  }
  else if(index==1)//路程 km
  {
    indecate.led_speed=LED_OFF;
    indecate.led_distance=LED_ON;
    indecate.led_calorie=LED_OFF;
    indecate.led_step=LED_OFF;
    indecate.led_time=LED_OFF;
    indecate.led_updp=LED_OFF;
    indecate.led_downdp=LED_ON;
          
    //实际数据
    DIS_VAL=(treadmills.param.distance);  
    
    if(DIS_VAL>=99990)
    {
      DIS_VAL=0;
    }
    high=((u32)DIS_VAL/1000);
    low=((u32)DIS_VAL/10)%100;

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
  else if(index==2)//卡洛里 kcal
  {
    indecate.led_speed=LED_OFF;
    indecate.led_distance=LED_OFF;
    indecate.led_calorie=LED_ON;
    indecate.led_step=LED_OFF;
    indecate.led_time=LED_OFF;
    indecate.led_updp=LED_OFF;
    indecate.led_downdp=LED_OFF;

    //treadmills.param.calorie//单位：卡
     if(treadmills.param.calorie>=999000)
    {
      treadmills.param.calorie=0;
    }
    
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
  else if(index==3)//时间:mm:ss
  {
    indecate.led_speed=LED_OFF;
    indecate.led_distance=LED_OFF;
    indecate.led_calorie=LED_OFF;
    indecate.led_step=LED_OFF;
    indecate.led_time=LED_ON;
    indecate.led_updp=LED_ON;
    indecate.led_downdp=LED_ON;

    TIME_VAL=treadmills.param.second;
    high=((u32)TIME_VAL/60)%100;
    low=(u32)TIME_VAL%60;

    if(TIME_VAL<600)
    {
      set_seg_two(high,low);
      set_bit_seg(0,SEG_ALL_OFF);//强制关闭显示
    }
    else
    {
      set_seg_two(high,low);
    }

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

      treadmills.display.index=0;  //0--speed 1--distance 2--calorie 3--time 4--programe
      treadmills_display(treadmills.display.index);
      treadmills.display.mode=DISP_MODE_TO_AUTO;					
      break;
    case DISP_MODE_TO_AUTO:
      cnt++;        
      if(cnt>50)//50*100ms=5s
      {
        cnt=0;

					if(treadmills.status==STATUS_POWERON||treadmills.status==STATUS_WAIT
						||treadmills.status==STATUS_RUNNING||treadmills.status==STATUS_STOP
						||treadmills.status==STATUS_STOP_WAIT){
						treadmills.display.index++;
						if(treadmills.display.index>=4){treadmills.display.index=0;}
						treadmills_display(treadmills.display.index);
					}
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
      uart1_frame_tx(CMD_ACK,treadmills.ack);
      timeout=0;
      sm=1;
      break;
    case 1:
      timeout++;
      if(treadmills.ack==0)
      {
        uart1_frame_tx(CMD_ACK,treadmills.ack);
        if(timeout>1)
        {
          timeout=0;
          error_cnt++;
          if(error_cnt>10)
          {
            treadmills.error_code=ERROR_06;
            treadmills.status=STATUS_ERROR;
            sm=2;
            beep_set(1,ERROR_ON_TICKS,ERROR_OFF_TICKS);
          }
          else{sm=0;}
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

