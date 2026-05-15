#include "pid.h"
#include "motor.h"

pid_t mt_pid;

u16 pid_calc(pid_t *pid,u16 target,u16 actual,u8 index)
{
    float error;//误差
    float uk;//增量
    
    //计算误差
    error = (float)target - (float)actual;
    //增量计算公式
    uk = pid->kp*(error - pid->error_1)
        + pid->ki*error
  		  + pid->kd*(error - 2*pid->error_1+pid->error_2);
		
    pid->error_2 = pid->error_1;//下一次迭代
    pid->error_1 = error;
		
#if(1)
    //限制增量幅度
    if(uk >=UK_STEP_MAX){uk=UK_STEP_MAX;}
    if(uk <=UK_STEP_MIN){uk=UK_STEP_MIN;}  //UK_STEP_MIN (-100)
#endif
		
    pid->delta_uk=uk;
		
    //计算实际增量
    pid->uk+=uk;
		
    //限制幅度输出
    if(pid->uk >= UK_PWM_MAX){pid->uk=UK_PWM_MAX;}
    if(pid->uk <= UK_PWM_MIN){pid->uk=UK_PWM_MIN;}
    return (u16)(pid->uk);
}

void default_pid_int(pid_t *pid)    //默认PID设置
{
	//山东 AGS1
	pid->kp=0.20;  //0.35 0.23      0.20
	pid->ki=0.15;  //0.25 0.18			0.15
	pid->kd=0.04;  //0.04 0.12			0.04
	pid->error_1=0;
	pid->error_2=0;
	pid->uk=0;
	pid->uk_max=UK_PWM_MAX;
	pid->uk_min=UK_PWM_MIN;
	pid->max_step_uk=UK_STEP_MAX;
	pid->min_step_uk=UK_STEP_MIN;
}

