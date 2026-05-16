/*
 * Function: 电机驱动子系统对外的占空范围声明与 ISR/保护入口。
 * Method:   UK_PWM_* 限幅；声明 motor_drive_isr、母线捕获、过压刷新及 error_chick。
 * Name:     Huey
 * Date:     May 16, 2026 18:00
 */

#ifndef BSP_MOTOR_DRIVE_H_
#define BSP_MOTOR_DRIVE_H_
#include "ddl.h"
#include "motor.h"

#define UK_PWM_MAX  ((uint16_t)((MOTOR_PWM_PERIOD_TICKS) - 5u))
#define UK_PWM_MIN  (1)

extern volatile u16 oc_lim_full_mirror;
extern volatile u16 over_voltage;

void motor_drive_refresh_over_voltage_from_vmax(void);
void motor_drive_boot_sync_limits(void);

extern u32 motor_vbus_adc;

extern void motor_drive_on_to_run_capture_bus(u16 valtage_up_adc);
void motor_drive_on_to_run_capture_bus_average(void);

extern void motor_drive_isr(void);

extern void reset_current_lp_filter(void);

void error_chick(void);

#endif /* BSP_MOTOR_DRIVE_H_ */
