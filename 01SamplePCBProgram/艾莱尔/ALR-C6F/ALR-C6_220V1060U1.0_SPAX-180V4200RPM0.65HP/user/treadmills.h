#ifndef USER_TREADMILLS_H_
#define USER_TREADMILLS_H_
#include "common.h"
#include "ee_param.h"

#define LED1       (P05)
#define Bluetooth  (P15)

#define STOP_DISPLAY_INTERVAL (11)  //base time:10ms 停止的时候倒计时速度，数字大倒计时慢
#define STOP_WAIT_TIMEOUT     (250) //BASE TIME:100ms
#define Overflow_time  (6000)  //跑步机运行最大时间

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
	ERROR_02,       //er02 电机断线
	ERROR_03,       //er03 电机堵转  低速电流过大
	ERROR_04,       //er04 主继电器粘连
	ERROR_05,   		//er05 电流超过保护值（电机负载过大）
	ERROR_06,     	//er06 速度超过设定值，飞车等
  ERROR_07,    		//er07 安全锁
  ERROR_08,      	//er08 功率管短路
	ERROR_09,
	ERROR_0A,
	ERROR_0B,
	ERROR_0C,
}error_code_enum;

typedef enum _status_e
{
	STATUS_POWERON=1,
	STATUS_WAIT,

	STATUS_RUN,
	STATUS_RUNNING,

	STATUS_STOP,
	STATUS_STOP_WAIT,
	STATUS_STOP_OVER,
  STATUS_SET_PARAM,

	STATUS_PAUSE,					//暂停
  STATUS_SLEEP_MODE,    //休眠模式
  STATUS_FULL_DISPLAY,   
	
	STATUS_ERROR,
	STATUS_REED_ERROR,
	DISP_MODE_AUTO,
	DISP_MODE_TO_AUTO,
	DISP_MODE_SINGLE,
}status_e;


typedef struct _t_treadmills
{
	u8 ack;
	u8 error_reed;
	u8 error_code;
	u8 error_cnt;
	u8 status;
	u8 basetime;
	struct
	{
    u8 beep_status;    //2023.3.24
		u16 speed;
		u32 second;
		float distance;
		u32 calorie;  //卡路里
    u16 runstep;  //步数
    u8 speed_step;     //2023.3.23
    u16 speed_max;
    u8 voltage_max;
    u8 voltage_min;
    u8 over_current_max;
    volatile u16 kiv[KIV_NUM];        //u16->u8 2023.2.17    	
	}param;         //参数

	struct
	{
		u8 index;    //0--speed 1--time 2--distance 3--calorie 4--programe
		u8 mode;     //模式档
	}display;      //显示
  
}treadmills_t;


extern u8 booktime_en;
extern u8 booktime_cnt;
extern bit STATUS_PAUSE_FLAG;//暂停

extern treadmills_t treadmills;
extern void treadmills_init(void);
extern void treadmills_proc_loop(void);
extern void treadmills_disp_loop(void);
extern void communication_checkout(void);
extern void treadmills_param_calc(void);
extern void boot_time(void);
extern void treadmills_display(index);
#endif /* USER_TREADMILLS_H_ */
