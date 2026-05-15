/*
 * motor_speed_pid.h
 *
 *  Created on: 2021Äê1ÔÂ26ÈÕ
 *      Author: Administrator
 */

#ifndef BSP_MOTOR_SPEED_PID_H_
#define BSP_MOTOR_SPEED_PID_H_
#include "ddl.h"
#include "motor.h"

extern u16 pwm_delta;
extern u16 voltage_over_error_cnt;
extern u16 over_voltage;
extern u16 over_current;

/* PTFM-1.0 open-loop controller interface */
extern void ptfm_bus_init(void);     /* call once at end of STATUS_POWERON   */
extern void ptfm_slew_reset(void);   /* call at STATUS_TO_RUN entry          */

extern void motor_speed_pid_isr(motor_t mot);
void error_chick(void);
#endif /* BSP_MOTOR_SPEED_PID_H_ */
