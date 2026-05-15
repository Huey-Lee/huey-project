#ifndef _PID_H
#define _PID_H

#include "hc32f005.h"
#include "bt.h"


typedef struct _t_pid
{
  u8 step_en;
  float sum_error;   //误差累计
  int16_t error_1;     //上一个偏差
  int16_t error_2;     //上上一个偏差
  float kp,ki,kd;    //定义比例，积分，微分参数
  float uk;
  float delta_uk;

  u16 uk_max;
  u16 uk_min;

  float max_step_uk;
  float min_step_uk;
}pid_t;



#define UK_STEP_MAX (20)
#define UK_STEP_MIN (-20)
#define UK_PWM_MAX  (540)
#define UK_PWM_MIN   (15)

extern u16 locpid_cacl(pid_t *pid,u16 target,u16 actual);
extern u16 pid_calc(pid_t *pid,u16 target,u16 actual,u8 index);

extern void default_pid_int(pid_t *pid);
extern void start_pid_int(pid_t *pid);
extern pid_t mt_pid;

#endif

