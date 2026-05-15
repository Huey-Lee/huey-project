#include "pid.h"
#include "motor.h"

pid_t mt_pid;
float PID_P[13]={
	0.10,//0km					/
	0.12,//1km					/
	0.15,//2km					/
	0.15,//3km					/
	0.20,//4km
	0.20,//5km
	0.20,//6km
	0.20,//7km
	0.18,//8km
	0.18,//9km
	0.18,//10km
	0.18,//11km
	0.18,//12km
};
float PID_D[13]={
	0.000,//0km				/
	0.000,//1km				/
	0.016,//2km				/
	0.020,//3km				/
	0.020,//4km
	0.020,//5km
	0.020,//6km
	0.02,//7km
	0.02,//8km
	0.02,//9km
	0.02,//10km
	0.02,//11km
	0.02,//12km
};
u16 pid_calc(pid_t *pid,u16 target,u16 actual,u8 index)
{
    int error;//误差
    float uk;//增量
		pid->kp = PID_P[index];

    //计算误差
    error = (int)target - (int)actual;
    //增量计算公式
//	switch(index)
//	{
//		case(0):uk = ((float)error)/3; break;//uk = ((float)error)/5;
//		case(1):uk = ((float)error)/3; break;
//		case(2):uk = pid->kp*((float)error ); break;
//		case(3):uk = pid->kp*((float)error ); break;
//		case(4):uk = pid->kp*((float)error ); break;
//		case(5):uk = pid->kp*((float)error ); break;
//		case(6):uk = pid->kp*((float)error ); break;
//		case(7):uk = pid->kp*((float)error ); break;
//		case(8):uk = pid->kp*((float)error ); break;
//		case(9):uk = pid->kp*((float)error ); break;
//		case(10):uk = pid->kp*((float)error ); break;
//		case(11):uk = pid->kp*((float)error ); break;
//		case(12):uk = pid->kp*((float)error ); break;
//		case(13):uk = pid->kp*((float)error ); break;
//		case(14):uk = pid->kp*((float)error ); break;
//		default:uk = pid->kp*((float)error );
//	}
//    uk = ((float)error)/5;
      uk = pid->kp*(error );
    //计算实际增量
    pid->uk+=uk;

    //限制幅度输出
    if(pid->uk >= pid->uk_max){pid->uk=pid->uk_max;}
    if(pid->uk <= pid->uk_min){pid->uk=pid->uk_min;}
    return (u16)(pid->uk);
}

void default_pid_int(pid_t *pid)    //默认PID设置
{
	//山东 AGS1
	pid->kp=0.05;  //0.35 0.23      0.20
//	pid->ki=0.25;  //0.25 0.18			0.15
	pid->kd=0.04;  //0.04 0.12			0.04
	pid->error_1=0;
	pid->error_2=0;
	pid->uk=0;
	pid->uk_max=UK_PWM_MAX;
	pid->uk_min=UK_PWM_MIN;
	pid->max_step_uk=UK_STEP_MAX;
	pid->min_step_uk=UK_STEP_MIN;
}

