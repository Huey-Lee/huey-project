/* pid.c - fixed-point PID, output clamp, anti-windup. */
#include "pid.h"
#include "motor.h"

pid_t mt_pid;

/* 增量式（Velocity-form）PID，前馈由外部 adjust_now_voltage 提供，
 * PID 仅负责偏差补偿（微量修正）。
 *
 * 积分饱和抑制（Clamping Anti-windup）：
 *   输出 uk 已到上/下限时，若增量 delta 将使其更深入饱和区，
 *   则不再累加——防止重载时积分堆到极值，负载突卸瞬间喷出。
 *
 * 微分冲击抑制（Derivative Kick）：
 *   误差从大正值（重载掉速）突降至 0 时，kd*(0−0+error_2) 仍含
 *   旧误差，会产生正向脉冲。此处在积分饱和被抑制的同时，
 *   外层已有 ZHANKONBI_SLEW_UP 限幅（motor.h），二者共同消除冲击。 */
u16 pid_calc(pid_t *pid, u16 target, u16 actual, u8 index)
{
    float error;
    float uk;
    float step_max;
    float step_min;

    error = (float)target - (float)actual;

    uk = pid->kp * (error - pid->error_1)
       + pid->ki * error
       + pid->kd * (error - 2.0f * pid->error_1 + pid->error_2);

    pid->error_2 = pid->error_1;
    pid->error_1 = error;

    step_max = (float)UK_STEP_MAX;
    step_min = (float)UK_STEP_MIN;
    if (index <= 1u) {
        step_max =  (float)UK_STEP_MAX_LOW_SPD;
        step_min = -(float)UK_STEP_MAX_LOW_SPD;
    }
    if (uk >= step_max) uk = step_max;
    if (uk <= step_min) uk = step_min;

    pid->delta_uk = uk;

    /* 饱和抑制：上限且增量为正 / 下限且增量为负 → 不累加 */
    if (!((pid->uk >= (float)UK_PWM_MAX && uk > 0.0f) ||
          (pid->uk <= (float)UK_PWM_MIN && uk < 0.0f))) {
        pid->uk += uk;
    }

    if (pid->uk >= (float)UK_PWM_MAX) pid->uk = (float)UK_PWM_MAX;
    if (pid->uk <= (float)UK_PWM_MIN) pid->uk = (float)UK_PWM_MIN;
    return (u16)(pid->uk);
}

void default_pid_int(pid_t *pid)
{
	pid->kp=0.15f;   /* 上控下发若失败则用此默认值 */
	pid->ki=0.08f;
	pid->kd=0.05f;
	pid->error_1=0;
	pid->error_2=0;
	pid->uk=0;
	pid->uk_max=UK_PWM_MAX;
	pid->uk_min=UK_PWM_MIN;
	pid->max_step_uk=UK_STEP_MAX;
	pid->min_step_uk=UK_STEP_MIN;
}

void pid_clear_integral_keep_output(pid_t *pid, u16 current_pwm)
{
	float u = (float)current_pwm;
	if (u >= (float)UK_PWM_MAX) u = (float)UK_PWM_MAX;
	if (u <= (float)UK_PWM_MIN) u = (float)UK_PWM_MIN;
	pid->error_1 = 0;
	pid->error_2 = 0;
	pid->delta_uk = 0.0f;
	pid->uk = u;
}
