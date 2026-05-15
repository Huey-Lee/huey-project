/*
 * motor.c
 *
 *  Created on: 2019年12月11日
 *      Author: Administrator
 */
#include "motor.h"
#include "MY_TIM.h"
#include "pid.h"
#include "MY_ADC.h"
#include "uart_frame.h"
extern void ptfm_bus_init(void);
extern void ptfm_slew_reset(void);
extern adc_t adc_handle;
extern pid_t mt_pid;
extern u8 waitforamoment;
ctr_t ctr;
motor_t motor;
speed_param_t  speed_param[13]=
{
	//山东小电机--注意此表格的电压不再做对应关系，这里利用公式求解的方法，计算得到相应的电压值
  //	float voltage;
  //	u16 pwm_max;
  //	u16 pwm_min;
  //	float kiv;   放大倍数
  //	u8 adjust_max_voltage;
  //	u8 adjust_cont_voltage;
  //	float load_base_curren;
  {GET_SPEED_VOLTAGE(100,0.14,18),  1000,1000, 3, 30,4,23}, //0--- 18V
  {GET_SPEED_VOLTAGE(100,0.14,18),  1000,1000, 3, 30,4,23}, //1--- 18V
  {GET_SPEED_VOLTAGE(200,0.14,18),  1000,1000, 3, 30,4,23}, //2--- 32V					可以
  {GET_SPEED_VOLTAGE(300,0.14,18),  1000,1000, 3, 30,4,30}, //3--- 46V
  {GET_SPEED_VOLTAGE(400,0.14,18),  1000,1000, 3, 25,5,30}, //4--- 60V
  {GET_SPEED_VOLTAGE(500,0.14,18),  1000,1000, 3, 25,6,30}, //5--- 74V
  {GET_SPEED_VOLTAGE(600,0.14,18),  1000,1000, 2, 25,7,30}, //6--- 88V
  {GET_SPEED_VOLTAGE(700,0.14,18),  1000,1000, 2, 20,7,40}, //7--- 102V
  {GET_SPEED_VOLTAGE(800,0.14,18),  1000,1000, 2, 20,7,40}, //8--- 116V
  {GET_SPEED_VOLTAGE(900,0.14,18),  1000,1000, 2, 20,3,30}, //9--- 130V
  {GET_SPEED_VOLTAGE(1000,0.14,18), 1000,1000, 2, 20,8,33}, //10---144V
  {GET_SPEED_VOLTAGE(1100,0.14,18), 1000,1000, 2, 20,8,35}, //11---158V
	{GET_SPEED_VOLTAGE(1200,0.14,18), 1000,1000, 2, 20,8,35}, //12---172V
};
void ctr_init(void)
{
	ctr.status = STATUS_POWERON;
	
	motor.stop_time_late = 0;
	motor.start_valtage = 10;
	motor.adjust_sta_voltage = 10;
	motor.start_add_val = 2;
	motor.sta_speed_scale = 50;
	motor.END_speed_scale = end_set_val;
	motor.accelerated = speed_step_PID;
	motor.deceleration = speed_step_END;
	motor.adjust_ove_curren = 400;
	motor.adjust_max_voltage = 90;
	motor.adjust_min_voltage = 20;
	motor.adjust_speed_max   = 1200;
	motor.voltage_param = ((motor.adjust_max_voltage - motor.adjust_min_voltage)*1.0)/(motor.adjust_speed_max - 100);//0.1463636363636364
}
extern u16 ZHANKONBI;
void ctr_proc_loop(void)
{
	static u8 interrupt_flag=0,er02_code=0,er08_code=0;
//	uint16_t cut_adc;			//ad差值
	uint16_t val_up,val_down;
	/*
	 * 解决：有可能继电器合上了，但是由于和MOS管PWM调速上有时间偏差，导致没有办法等到STATUS_TO_STOP
	 * 从而只能让上控显示板超时退出的问题。
	 */
	switch(ctr.status)
	{
	case STATUS_POWERON:
		MY_ADC_INIT();
		for(u8 i=0;i<100;i++)
		{
			if(adc_handle.irq_flag==0 && interrupt_flag==0)
			{
				Adc_Start();
			}
			if(interrupt_flag!=adc_handle.irq_flag)//采集到数据
			{
				if(adc_handle.irq_flag>0)						 //采集数据
				{
					interrupt_flag =adc_handle.irq_flag;
					Adc_Start();
				}
				if(adc_handle.irq_flag == 0 && interrupt_flag > 0)//采集完成
				{
					interrupt_flag=0;
					val_up = CALC_ADC_2_VOLTAGE(motor.valtage_up);
					val_down = CALC_ADC_2_VOLTAGE(motor.valtage_down);
					if((val_up>(val_down + 70)) || (val_down > (val_up + 70)))
					{
						er02_code++;
					}
					if(val_up<10 && val_down < 10)
					{
						er08_code++;
					}
				}
			}
		}
		if(er02_code > 5)
		{
			er02_code = 0;
      ctr.error_code=ERROR_06;
      ctr.status=STATUS_ERROR;
		}
		else if(er08_code > 5)
		{
			er08_code = 0;
      ctr.error_code=ERROR_05;		//电机异常
      ctr.status=STATUS_ERROR;
		}
		else
		{
			ptfm_bus_init();
			ctr.status = NULL;
		}
	//上电处理
	break;
	case STATUS_RUN:
		MY_ADC_INIT();
		for(u8 i=0;i<100;i++)
		{
			if(adc_handle.irq_flag==0 && interrupt_flag==0)
			{
				Adc_Start();
			}
			if(interrupt_flag!=adc_handle.irq_flag)
			{
				if(adc_handle.irq_flag>0)
				{
					interrupt_flag =adc_handle.irq_flag;
					Adc_Start();
				}
				if(adc_handle.irq_flag == 0 && interrupt_flag > 0)
				{
					interrupt_flag=0;
					val_up = CALC_ADC_2_VOLTAGE(motor.valtage_up);
					val_down = CALC_ADC_2_VOLTAGE(motor.valtage_down);
					if((val_up>(val_down + 70)) || (val_down > (val_up + 70)))
					{
						er02_code++;
					}
					if(val_up<10 && val_down < 10)
					{
						er08_code++;
					}
				}
			}
		}
		if(er02_code > 5)
		{
			er02_code = 0;
      ctr.error_code=ERROR_06;								
      ctr.status=STATUS_ERROR;
		}
		else if(er08_code > 5)
		{
			er08_code = 0;
      ctr.error_code=ERROR_05;							//
      ctr.status=STATUS_ERROR;
		}
		else
		{
			ctr.status = NULL;
			waitforamoment = 1;
		}
	break;
	case STATUS_TO_RUN:
		motor.adjust_set_voltage = speed_param[0].voltage;
		motor.adjust_now_voltage=13;
		ZHANKONBI = MT_START_PWM;
		if(M0P_ADTIM6->GCONR_f.START == 0 || M0P_TIM0->CR_f.CTEN == 0)
		{
			motor.status=STATUS_MT_START;
			ptfm_slew_reset();
			clear_adcbuf();
			motor.cur_speed_scale = motor.sta_speed_scale;
			motor.adjust_sta_voltage = motor.start_valtage;
			motor.start_tim = 0;
			tim0run();
			delay1ms(500);
			tim6run();
		}

		
		ctr.status = NULL;
	break;
	case STATUS_STOP:

		motor.status=STATUS_MT_STOP;
	  ctr.status = NULL;
	break;
	case STATUS_TO_STOP:
		tim0stop();
		tim6stop();
//				printf("ctr:%d\r\n",ctr.status);
		uart_frame_tx(CMD_STOP_OVER,1);
		ctr.status = NULL;
	break;
	case STATUS_ERROR:
			motor.set_speed_scale=MT_STOP_SPEED;//设置停止速度
			motor.status = STATUS_MT_STOP;
		switch(ctr.error_code)
		{
//    case ERROR_01://er01 没有通讯，上下板之间没有建立通讯

//			break;
//		case ERROR_MT_OPEN://er02 电机断线

//			break;
//		case ERROR_MT_STALL://er03 电机堵转（电机在一段时间内停止转动）

//			break;
//		case ERROR_RELAY://er04 主继电器粘连

//			break;
		case ERROR_03://er05 电流超过保护值（电机负载过大）
			tim0stop();
			tim6stop();

			break;
//		case ERROR_OVER_SPEED://er06 速度超过设定值，飞车等

//			break;
//    case ERROR_MOS_SHORT://er08 功率管短路

//			break;
		default:
			break;
		}
		if(ctr.error_code)
		{
			uart_frame_tx(CMD_ERROR,ctr.error_code);
		}
		motor.set_speed_scale=MT_STOP_SPEED;//设置停止速度
		motor.status = STATUS_MT_STOP;
	break;
	default:
	break;
	}
}
void motor_speed(void)
{
	u16 val;
	int16_t cut_adc;
	u16 now_valtage=0;
	switch(motor.status)
	{
		case(STATUS_MT_START):
		
		
		if(motor.cur_speed_scale < motor.set_speed_scale)
		{
			motor.cur_speed_scale+=motor.accelerated;
		}
		if(motor.cur_speed_scale > motor.set_speed_scale)
		{
			motor.cur_speed_scale-=motor.accelerated;
		}
		val = (u16)GET_SPEED_VOLTAGE(motor.cur_speed_scale,motor.voltage_param,motor.stop_valtage);
		motor.adjust_now_voltage = (u16)(val*8.13);
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
		cut_adc =	motor.valtage_up - motor.valtage_down;
		if(cut_adc<0)
			cut_adc=0;
		now_valtage = (u8)(0.126*cut_adc);

		
		break;
		case(STATUS_MT_PID):
		if(motor.cur_speed_scale < motor.set_speed_scale)
		{
			motor.cur_speed_scale+=motor.accelerated;
		}
		if(motor.cur_speed_scale > motor.set_speed_scale)
		{
			motor.cur_speed_scale-=motor.accelerated;
		}
		val = (u16)GET_SPEED_VOLTAGE(motor.cur_speed_scale,motor.voltage_param,motor.stop_valtage);
		motor.adjust_now_voltage = (u16)(val*8.13);
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
		/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
		break;
		case(STATUS_MT_STOP):
		if(motor.cur_speed_scale < motor.set_speed_scale)
		{
			motor.cur_speed_scale+=motor.deceleration;
		}
		if(motor.cur_speed_scale > motor.set_speed_scale)
		{
			motor.cur_speed_scale-=motor.deceleration;
		}
		
		if(motor.cur_speed_scale<MT_STOP_SPEED)
		{
			motor.stop_time_late++;
		}
		
		if(motor.stop_valtage>2)
			motor.stop_valtage = motor.stop_valtage - 0.5;
		else
			motor.stop_valtage=2;
		
		val = (u16)GET_SPEED_VOLTAGE(motor.cur_speed_scale,motor.voltage_param,motor.stop_valtage);
		motor.adjust_now_voltage = (u16)(val*8.13);
			
		if(motor.kiv>0.1)
			motor.kiv = motor.kiv-0.1;
		else
			motor.kiv = 0.1;
		break;
	}
	motor.index = (u8)(motor.cur_speed_scale / 100);

}
