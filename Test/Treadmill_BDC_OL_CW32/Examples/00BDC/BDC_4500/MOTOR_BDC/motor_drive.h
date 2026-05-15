/* motor_drive.h  开环电机驱动模块对外接口 */
#ifndef BSP_MOTOR_DRIVE_H_
#define BSP_MOTOR_DRIVE_H_
#include <stdint.h>
#include "motor.h"

/* PWM 输出范围（ADTIM6 三角波，PER=MOTOR_PWM_PERIOD_TICKS；
 * MAX=PER−5≈98.7%，MIN≈占空刻度 1/MOTOR_PWM_PERIOD_TICKS） */
#define UK_PWM_MAX  ((uint16_t)((MOTOR_PWM_PERIOD_TICKS) - 5u))
#define UK_PWM_MIN  (1)

extern volatile u16 oc_lim_full_mirror; /* ISR 全速过流阈；与 motor.adjust_ove_curren / CMD 同步 */
extern volatile u16 over_voltage; /* 由 vmax（adjust_max_voltage）推导的整型 V 标尺 */

void motor_drive_refresh_over_voltage_from_vmax(void); /* vmax 下发/变更后刷新 over_voltage */
void motor_drive_boot_sync_limits(void); /* oc_lim_full_mirror ← adjust_ove_curren；刷新 over_voltage 与 vbus 线性系数 */

/* 直流母线【固定分母】ADC；delay500ms 后锁存；运行中 motor_drive_isr（ADC 节拍）不从端压/占空反推改写 */
extern u32 motor_vbus_adc;

extern void motor_drive_on_to_run_capture_bus(u16 valtage_up_adc);
/* 合闸后 20 ms×1 ms（默认）对 M+ RAW 算术平均后锁 motor_vbus_adc；Keil 可覆写 MOTOR_BUS_CAPTURE_AVG_* */
void motor_drive_on_to_run_capture_bus_average(void);

/* ADC 完成节拍调用的开环驱动入口（user_adc→get_adc_value）；PWM 载波频 ≈ MCU_PCLK/(4×PER)，二者勿混 */
extern void motor_drive_isr(void);

/* 复位电流低通与 motor_drive_isr（ADC 节拍）侧保护计数（每次切入 MT_START / 再起动时调用） */
extern void reset_current_lp_filter(void);

/* 主循环保护检测（timing 依赖于主循环对 tim_handle.tim6_cur 的轮询与 tim6_change_speed_time）*/
void error_chick(void);

#endif /* BSP_MOTOR_DRIVE_H_ */
