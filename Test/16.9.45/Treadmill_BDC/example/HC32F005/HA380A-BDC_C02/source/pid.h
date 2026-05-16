/* pid.h - speed-loop PID_t, default_pid_int. */
#ifndef _PID_H
#define _PID_H

#include "hc32f005.h"
#include "bt.h"


typedef struct _t_pid
{
  u8 step_en;
  float sum_error;  /* (unused) integration optional */
  int16_t error_1;  /* e[k-1] */
  int16_t error_2;  /* e[k-2] */
  float kp, ki, kd; /* speed-loop terms */
  float uk;
  float delta_uk;

  u16 uk_max;
  u16 uk_min;

  float max_step_uk;
  float min_step_uk;
}pid_t;


// 先晃、抖、麻多来自来回纠：可先略降 P 或 I，看步进（UK_STEP）也一起动时别一次改太狠。
// 发肉、带载掉速：多半是 P/I 过小了 或要配合 KIV/最低电压 等，而不是继续猛减。
#define UK_STEP_MAX (20)	/* max delta duty per step */
#define UK_STEP_MIN (-20)
/* ���ٵ� (index 0..1, Լ 1.0~1.5 km/h): ��Сÿ��ռ�ձ仯, ���� PID ΢�� + ��еġ�ϸ��/���顱�� */
#define UK_STEP_MAX_LOW_SPD (10)
#define UK_PWM_MAX  (540)
#define UK_PWM_MIN   (1)

extern u16 locpid_cacl(pid_t *pid,u16 target,u16 actual);
extern u16 pid_calc(pid_t *pid,u16 target,u16 actual,u8 index);

extern void default_pid_int(pid_t *pid);
extern void start_pid_int(pid_t *pid);
/* Keep current duty; clear derivative history to avoid a PWM spike on fault / soft-stop entry. */
extern void pid_clear_integral_keep_output(pid_t *pid, u16 current_pwm);
extern pid_t mt_pid;

#endif

