/*
 * motor_speed_pid.c
 *
 *  Created on: 2021年1月26日
 *      Author: Administrator
 */

#include "pid.h"
#include "motor.h"
#include "motor_speed_pid.h"
#include "MY_TIM.h"
extern uint16_t ZHANKONBI;
extern pid_t mt_pid;
void deteck_walk(void);
u8 valtage;				//电机电压
u16 over_current = OVER_CURRENT_MAX;
u16 over_voltage = OVER_VOLTAGE_MAX;
//检测是否过流
static void checkout_current_over(void)
{
	static u8 low_speed_count;
  //电机低速过流保护
  if(motor.cur_speed_scale<=200)
  {
		if(motor.valtage_cur > OVER_CURRENT_MAX_LOW_SPEED)
			low_speed_count++;
		else
			low_speed_count=0;
		
		if(low_speed_count>20){	//three second
			ctr.status = NULL;
			motor.status = NULL;
      ctr.error_code=ERROR_03;//E03 低速电机堵转
      ctr.status=STATUS_ERROR;
			tim0stop();
			tim6stop();
		}
  }
}
static void Current_Max_Over(void)
{
	static u8 Current_over_flag;

	if(motor.valtage_cur>over_current )  //over_current=OVER_CURRENT_MAX
	{
			Current_over_flag++;
			if(Current_over_flag>10)
			{
					ctr.status = NULL;
					motor.status = NULL;
					ctr.error_code=ERROR_05;//E03 立诚指定要求
					ctr.status=STATUS_ERROR;
					tim0stop();
					tim6stop();
			}
	}
	else
		Current_over_flag=0;

}
//检测是否飞车
static void  checkout_speed_over(void)
{
	int val,val_t;
	static u8 current_max_over;
	if(motor.valtage_up > motor.valtage_down)
	val = motor.valtage_up - motor.valtage_down;
	val_t = CALC_ADC_2_VOLTAGE(val);
	
	if(val_t>=over_voltage)
		current_max_over++;
	else
		current_max_over=0;
	
	if(current_max_over>15){	//three hundred millisesecond
		ctr.status = NULL;
		motor.status = NULL;
		ctr.error_code=ERROR_08;
		ctr.status=STATUS_ERROR;
		tim0stop();
		tim6stop();
	}
}
u16 timestest=0;
void motor_speed_pid_isr(motor_t mot)//1.6k    0.625ms
{
	int16_t cut_adc;			//ad差值
	cut_adc =	mot.valtage_up - mot.valtage_down;
//	timestest++;
	switch(motor.status)
	{
		case STATUS_MT_START:
		//1.获取当前速度的电压值
		//4.计算PID值						//speed change slowly     单位速度下的电压     最小电压
		if(cut_adc<0)
			cut_adc=0;
		motor.stop_valtage = motor.adjust_min_voltage;
		valtage = (u8)(0.123*cut_adc);
		
		if(motor.start_tim%24==0)
		{
			motor.adjust_sta_voltage+=1;
			if(motor.adjust_sta_voltage>(u16)((motor.adjust_min_voltage+3)*8.13))
				motor.adjust_sta_voltage = (u16)((motor.adjust_min_voltage+3)*8.13);
		}
		
		ZHANKONBI = pid_calc(&mt_pid, motor.adjust_sta_voltage,(u16)cut_adc,mot.index);
		
		motor.start_set_val = GET_SPEED_VOLTAGE(100,motor.voltage_param,motor.adjust_min_voltage);
		motor.start_set_val = motor.start_set_val + ((mot.valtage_cur - speed_param[mot.index].adc_base_curren)*speed_param[mot.index].kiv)/10;
		
		if(motor.start_tim<20000)
		motor.start_tim++;
		if(motor.start_tim >= 400 && (motor.start_set_val)  < valtage )
		{
			motor.start_tim=0;
			motor.adjust_sta_voltage = motor.start_valtage;
			motor.status = STATUS_MT_PID;
			motor.start_set_val=0;
		}
		
		break;
		case STATUS_MT_PID:  //base time:7.8ms=75
		
		if((mot.valtage_cur - speed_param[mot.index].adc_base_curren)>0)
		{
			motor.adjust_kiv_voltage = (u16)(((mot.valtage_cur - speed_param[mot.index].adc_base_curren)*speed_param[mot.index].kiv)/10);
				motor.adjust_kiv_voltage = motor.adjust_now_voltage + motor.adjust_kiv_voltage;
		}
		else
			motor.adjust_kiv_voltage = motor.adjust_now_voltage;
		
		ZHANKONBI = pid_calc(&mt_pid, motor.adjust_kiv_voltage,(u16)cut_adc,mot.index);
		Current_Max_Over();
		break;
		case STATUS_MT_STOP:
		if((mot.valtage_cur - speed_param[mot.index].adc_base_curren)>0)
		{
			motor.adjust_kiv_voltage = (u16)(((mot.valtage_cur - speed_param[mot.index].adc_base_curren)*motor.kiv)/10);
				motor.adjust_kiv_voltage = motor.adjust_now_voltage + motor.adjust_kiv_voltage;
		}
		else
			motor.adjust_kiv_voltage = motor.adjust_now_voltage;
		
		valtage = (u8)(0.123*cut_adc);
		ZHANKONBI = pid_calc(&mt_pid, motor.adjust_kiv_voltage,(u16)cut_adc,mot.index);
		
		if(motor.END_speed_scale > valtage)	//when time to end  ZHANKONBI almost 25
		{
			motor.status = NULL;
			ctr.status = STATUS_TO_STOP;
			ZHANKONBI = MT_START_PWM;
						printf("end:%d\r\n",motor.status);
		}
		if(motor.cur_speed_scale>200)
		Current_Max_Over();
		break;
		
		default:
			break;
	  }
}

void error_chick(void)
{
	if(motor.status==STATUS_MT_PID || motor.status==STATUS_MT_STOP)
	{
		checkout_current_over();	//low speed current over
		
		checkout_speed_over();
		
//		Current_Max_Over();
	}
}
