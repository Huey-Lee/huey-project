/*
 * motor.h
 *
 *  Created on: 2022年12月20日
 *      Author: 张明
 */
#ifndef USER_MOTOR_H_
#define USER_MOTOR_H_
#include "hc32f005.h"
#include "bt.h"

#define VAR_SPEED_CNT             (3)     //升速的速度的时间，越小越快
#define VAR1_SPEED_CNT            (3)     //降速的速度的时间，越小越快
#define MT_OPEN_ADC               (100)   //检测电机连接线是否未接时候的ADC值，实际上这个波形是一个50HZ的类正弦波
#define MT_STOP_ACT_SPEED_SCALE   (4)     //20=0.2,当前速度达到的停止的速度----当前最小速度

#define MT_START_SPEED_SCALE      (20)    //30=0.3, 100=1.0---PID启动的最小速度等级
#define MT_TO_PID_SPEED_SCALE     (120)   //120=1.2,启动之后，切换到PID模式的速度等级

#define MT_START_PWM         (25)
#define MT_START_STEP        (3)
#define MT_STOP_SPEED				 (5)

#define speed_step_PID   	 3
#define speed_step_END   	 3

#define GET_SPEED_VOLTAGE(SPEED_SCALE,A,B)  (A*(float)(SPEED_SCALE-100.0)+B)      /*110V电机 获取速度等级时候的当前电压值---即电机两端实际电压*/

#define CALC_VOLTAGE_2_ADC(VOTAGE)    (((VOTAGE/MT_ADC_AMP)/4.95)*4096)                      /*计算当前电压值对应的ADC值*/
#define CALC_ADC_2_VOLTAGE(ADC)       (103.2044*(5*ADC)/4096)                         			 /*计算当前ADC值对应的电压值*/

#define CALC_ADC_2_CURRENT(ADC)       ((((ADC/4096.0)*4.95)/MT_CURRENT_ADC_AMP)/(SAMPLE_RES))/*计算当前ADC值对应的电流值*/

/**
 * 220v电机：R=0.004  110v电机：R=0.008
 * MAX_A=1A
 * KA=20倍
 *
 * OVER_CURRENT_MAX =(1*0.004*20*4096)/5=65.5
 */
#define CURRENT_15A0    (411)  //15A
#define CURRENT_14A0    (384)
#define CURRENT_13A0    (357)
#define CURRENT_12A0    (330)
#define CURRENT_11A0    (303)
#define CURRENT_10A0    (276)
#define CURRENT_9A0     (252)
#define CURRENT_8A0     (225)
#define CURRENT_7A0     (197)
#define CURRENT_6A0     (170)
#define CURRENT_5A0     (144)
#define CURRENT_4A0     (117)
#define CURRENT_3A0     (90)
#define CURRENT_2A0     (63)
#define CURRENT_1A0     (37)


#define OVER_CURRENT_MAX            (CURRENT_15A0)  //最大过流保护
#define OVER_CURRENT_MAX_LOW_SPEED  (CURRENT_6A0)   //最大过流保护
#define OVER_CURRENT_MAX_TEST       (CURRENT_2A0)

#define OVER_CURRENT_TIMEOUT        (100)        //过流保护时间，UNIT:1MS
#define LOW_SPEED_OVER_CURRENT_TIMEOUT   (5000)   //低速过流保护时间，UNIT:1MS
#define OVER_SPEED_TIMEOUT          (200)        //过压飞速时间，UNIT:10MS

#define end_set_val 15
//由于mos短路，造成飞车的参数
#define OVER_VOLTAGE_OFFSET         (80.0)      //偏高50V
#define OVER_VOLTAGE_MAX            (240.0)      //250V=3170adc,300V=3804adc
#define OVER_SPEED_TIMEOUT_VOLTAGE  (50)         //过压飞速时间，UNIT:10MS,由于电压引起的

#define MAX_MT_SPEED_SCALE          (120)        //最大速度等级 12.0
/*
一、错误信号说明：
er01 没有通讯，上下板之间没有建立通讯
er02 功率管短路，电机断线等
er03 电机堵转（电机在一段时间内停止转动）
er04 主继电器粘连
er05 电流超过保护值（电机负载过大）
er06 速度超过设定值，飞车等
er07 下控上送信号线开路，现象为下控上送信号线恒定为高电平
其中er02---er06是由下板送上来的信号触发，er01和er07是由上板自己检测触发
通讯协议：从开机起上下板之间必须不间断的通讯，确保通讯正常，在一定的时间里无通讯建立，那么下板停机，上板报er01。建议这个时间设置成5秒以上，以加强容错，
*/
typedef enum _error_code_enum
{
	ERROR_01=1,     //er01 没有通讯，上下板之间没有建立通讯
	ERROR_02,       //er02 (IGBT损坏)
	ERROR_03,       //er03 电机堵转  低速电流过大
	ERROR_04,       //er04 主继电器粘连					（升降电机）
	ERROR_05,   		//er05 电流超过保护值（电机负载过大）
	ERROR_06,     	//er06 （电机通信异常，电机断线，motor out of control）
  ERROR_07,    		//er07 安全锁
  ERROR_08,      	//er08 速度超过设定值，飞车等
	ERROR_09,
	ERROR_0A,
	ERROR_0B,
	ERROR_0C,
}error_code_enum;


typedef enum _motor_enum
{
	STATUS_POWERON=1,
	STATUS_RUN,
	STATUS_TO_RUN,
	STATUS_STOP,
	STATUS_TO_STOP,
	STATUS_ERROR,

	STATUS_MT_START,			//7
	STATUS_MT_PID,				//8
	STATUS_MT_STOP,				//9
	STATUS_MT_IMMED_STOP,
}motor_enum;


typedef struct _t_motor
{
  u8 status;
	u8 en;
	u8 index;
  u8 adjust_max_voltage;//最大电压
  u8 adjust_min_voltage;//最小电压
	u16 adjust_speed_max;	//最大速度
  u16 adjust_ove_curren;//最大电流
	
  u16 set_speed_scale;	//
  u16 cur_speed_scale;	//
	u16 sta_speed_scale;	//
	u16 END_speed_scale;	//
	u16 accelerated;
	u16 deceleration;
  float voltage_param;  //单位速度下的电压
	
	u16 start_tim;
	
  u16 start_set_val;
  u16 start_cur_val;
	
  u8 adjust_set_voltage;	//调整设置电压
  u8 adjust_cou_voltage;	//调整渐变电压
	u16 adjust_kiv_voltage;	//kiv计算电压
	u16 adjust_sta_voltage;	//STA计算电压
	u16 adjust_now_voltage;	//now计算电压
	
	float stop_valtage;
	float start_valtage;
	u8 kiv;
	u16 valtage_cur;				//adc采集电流值
	u16 valtage_up;
	u16 valtage_down;
	float motor_valtage;
	float now_current;
}motor_t;

typedef struct _t_speed_param
{
	float voltage;
	u16 pwm_max;
	u16 pwm_min;

	u8 kiv; //增加 volatile 2023.2.17
	u8 adjust_max_voltage;
	u8 adjust_cont_voltage;
	u8 adc_base_curren;												//不同速度下的基础电流
}speed_param_t;


typedef struct _t_ctr
{
  u8 status;
  u8 error_code;
  u8 ack;
  u8 immed_stop_flag;
  u8 immed_stop_voltage_flag;
}ctr_t;													//电机状态标志位


extern ctr_t ctr;
extern motor_t motor;
extern speed_param_t speed_param[];  

void motor_speed(void);
extern void ctr_proc_loop(void);
extern void ctr_init(void);
extern void stop_motor(void);
extern void set_motor_speed(u8 scale);

#endif /* USER_MOTOR_H_ */
