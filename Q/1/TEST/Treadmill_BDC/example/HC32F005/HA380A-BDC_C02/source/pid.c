/* pid.c - fixed-point PID, output clamp. */
#include "pid.h"
#include "motor.h"

pid_t mt_pid;

u16 pid_calc(pid_t *pid,u16 target,u16 actual,u8 index)
{
    float error;
    float uk;
    float step_max;
    float step_min;

    error = (float)target - (float)actual;

    {
        float i_term = pid->ki * error;
        if (i_term > 10.0f)
            i_term = 10.0f;
        else if (i_term < -10.0f)
            i_term = -10.0f;
        uk = pid->kp * (error - (float)pid->error_1)
            + i_term
            + pid->kd * (error - 2.0f * (float)pid->error_1 + (float)pid->error_2);
    }

    pid->error_2 = pid->error_1;
    pid->error_1 = error; /* int16, truncated from float (legacy) */

    step_max = (float)UK_STEP_MAX;
    step_min = (float)UK_STEP_MIN;
    if (index <= 1u) {
        step_max = (float)UK_STEP_MAX_LOW_SPD;
        step_min = -(float)UK_STEP_MAX_LOW_SPD;
    }
#if(1)
    if(uk >= step_max){uk= step_max;}
    if(uk <= step_min){uk= step_min;}
#endif

    pid->delta_uk=uk;

    pid->uk+=uk;

    if(pid->uk >= UK_PWM_MAX){pid->uk=UK_PWM_MAX;}
    if(pid->uk <= UK_PWM_MIN){pid->uk=UK_PWM_MIN;}
    return (u16)(pid->uk);
}

void default_pid_int(pid_t *pid)
{
	pid->kp = 0.16f;
	pid->ki = 0.10f;
	pid->kd = 0.035f;
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
