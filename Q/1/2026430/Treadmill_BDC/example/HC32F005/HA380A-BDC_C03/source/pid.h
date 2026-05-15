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
/* 低速 (index 0..1, 约 1.0~1.5 km/h): 减小每次占空比变化, 配合 PID 微调 */
#define UK_STEP_MAX_LOW_SPD (10)
#define UK_PWM_MAX  (540)
#define UK_PWM_MIN   (1)

/* 前馈限幅 Anti-windup 范围（STATUS_MT_PID专用）：
 *
 * uk 限制在"前馈PWM ± PID_FF_WINDUP_RANGE"以内，
 * 防止重载时积分堆积到 UK_PWM_MAX（540），脚一抬大幅回落产生"噌"声。
 *
 * 前馈PWM = adjust_kiv_voltage × UK_PWM_MAX / (adjust_max_voltage × 8.13)
 *          ≈ V_target × 6  （90V供电，540满占空）
 *
 * 15% × 540 = 81 单位 ≈ 13.5V 补偿余量：
 *   1km/h(18V)：uk 上限 108+81=189，电机最多被推到 189/540×90≈31.5V，
 *               比目标多 13.5V，足以应对重载；脚抬起时只需退 81 步，约 2.7ms。
 *   原来无限制：uk 可达 540，脚抬后需退 432 步 ≈ 14ms，产生明显"噌"冲。
 *
 * KIV 非零时，adjust_kiv_voltage 含载荷前馈，分子自动升高，
 * uk 上限随之提高，重载时补偿能力更强，两者协同工作。 */
#define PID_FF_WINDUP_RANGE  (81u)

extern u16 locpid_cacl(pid_t *pid,u16 target,u16 actual);
extern u16 pid_calc(pid_t *pid,u16 target,u16 actual,u8 index);

extern void default_pid_int(pid_t *pid);
extern void start_pid_int(pid_t *pid);
/* Keep current duty; clear derivative history to avoid a PWM spike on fault / soft-stop entry. */
extern void pid_clear_integral_keep_output(pid_t *pid, u16 current_pwm);
extern pid_t mt_pid;

#endif

