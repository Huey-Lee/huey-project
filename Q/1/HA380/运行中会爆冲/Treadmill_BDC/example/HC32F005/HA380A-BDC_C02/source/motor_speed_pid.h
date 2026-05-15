/* motor_speed_pid.h - speed PID ISR, OC/OV, error_chick. */
#ifndef BSP_MOTOR_SPEED_PID_H_
#define BSP_MOTOR_SPEED_PID_H_
#include "ddl.h"
#include "motor.h"

extern u16 pwm_delta;
extern u16 voltage_over_error_cnt;
extern u16 over_voltage;
extern void motor_speed_pid_isr(motor_t mot);
void error_chick(void);
/* 状态切换时复位电压目标斜率滤波，避免沿用上一次的缓变目标 */
void motor_pid_target_filter_reset(void);
#endif /* BSP_MOTOR_SPEED_PID_H_ */
