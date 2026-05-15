/* motor_speed_pid.c - ISR: START 电压 PID；RUN 前馈表 + 软 KIV；STOP 仍为 PID；OC/OV 不变。
 *
 * Power & Transition：仅 STATUS_MT_START 使用 pid_calc；motor.c 切入 RUN 前做积分同步。
 * Feed-forward：adjust_now_voltage（GET_SPEED_VOLTAGE×8.13，与 T1 参数表一致）→ 线性映射占空。
 * Soft KIV：ΔI*kiv/10 + 百分比与 ~10 V 绝对上限；PWM 输出 ±PWM_SLEW_STEP_MAX。
 */
#include "pid.h"
#include "motor.h"
#include "motor_speed_pid.h"
#include "user_timer.h"

extern uint16_t ZHANKONBI;
extern pid_t mt_pid;

u8 valtage;

u16 over_current = OVER_CURRENT_MAX;
u16 over_voltage = OVER_VOLTAGE_MAX;
u16 timestest = 0;

/* E08: low speed (scale <= 200) and current above low-speed cap for too long. */
static void checkout_current_over(void)
{
	static u8 low_speed_count;
	if (motor.cur_speed_scale <= 200) {
		if (motor.valtage_cur > OVER_CURRENT_MAX_LOW_SPEED)
			low_speed_count++;
		else
			low_speed_count = 0;

		if (low_speed_count > 20) { /* debounce, ~1s depending on call rate */
			low_speed_count = 0;
			motor_hard_fault_set(ERROR_08);
		}
	}
}

/* E08: any-speed over-current, sustained. */
static void Current_Max_Over(void)
{
	static u8 Current_over_flag;
	if (motor.valtage_cur > over_current) {
		Current_over_flag++;
		if (Current_over_flag > 200) {
			Current_over_flag = 0;
			motor_hard_fault_set(ERROR_08);
		}
	} else
		Current_over_flag = 0;
}

/* E05: bus over-voltage from (valtage_up - valtage_down) for ~300 ms. */
static void checkout_speed_over(void)
{
	int val, val_t;
	static u8 current_max_over;
	if (motor.valtage_up > motor.valtage_down)
		val = motor.valtage_up - motor.valtage_down;
	val_t = CALC_ADC_2_VOLTAGE(val);

	if (val_t >= over_voltage)
		current_max_over++;
	else
		current_max_over = 0;

	if (current_max_over > 15) {
		current_max_over = 0;
		motor_hard_fault_set(ERROR_05);
	}
}

u16 old_pwm = 0;

#ifndef PID_VOLT_CMD_SLEW_PER_ISR
#define PID_VOLT_CMD_SLEW_PER_ISR (6u)
#endif
#ifndef PWM_SLEW_STEP_MAX
#define PWM_SLEW_STEP_MAX (1u)
#endif
#ifndef FF_VOLT_CMD_MIN
#define FF_VOLT_CMD_MIN (140u)
#endif
#ifndef FF_VOLT_CMD_MAX
#define FF_VOLT_CMD_MAX (900u)
#endif
#ifndef SOFT_KIV_MAX_PERCENT
#define SOFT_KIV_MAX_PERCENT (12u)
#endif
#ifndef SOFT_KIV_ABS_RAW_MAX
#define SOFT_KIV_ABS_RAW_MAX (81u) /* ~10 V * 8.13 */
#endif

static u16 s_pid_volt_cmd;
static u16 s_pid_pwm_prev;
static u8 s_pid_sess_init;

static u16 ff_duty_from_volt_cmd(u16 cmd)
{
	u32 c = cmd;

	if (c < (u32)FF_VOLT_CMD_MIN)
		c = FF_VOLT_CMD_MIN;
	if (c > (u32)FF_VOLT_CMD_MAX)
		c = FF_VOLT_CMD_MAX;
	return (u16)(MT_START_PWM + (c - (u32)FF_VOLT_CMD_MIN) * (u32)(UK_PWM_MAX - MT_START_PWM)
		     / ((u32)FF_VOLT_CMD_MAX - (u32)FF_VOLT_CMD_MIN));
}

static u16 compute_soft_kiv_raw(motor_t mot)
{
	float di;
	float ov;
	u32 pctcap;

	if (mot.valtage_cur <= speed_param[mot.index].adc_base_curren)
		return 0u;
	di = (float)mot.valtage_cur - (float)speed_param[mot.index].adc_base_curren;
	if (di <= 0.0f)
		return 0u;
	ov = di * mot.kiv / 10.0f;
	if (ov <= 0.0f)
		return 0u;

	pctcap = (u32)mot.adjust_now_voltage * (u32)SOFT_KIV_MAX_PERCENT / 100u;
	if (pctcap == 0u)
		pctcap = 1u;
	if (ov > (float)pctcap)
		ov = (float)pctcap;
	if (ov > (float)SOFT_KIV_ABS_RAW_MAX)
		ov = (float)SOFT_KIV_ABS_RAW_MAX;
	return (u16)ov;
}

static u16 pwm_apply_slew_step(u16 raw_req)
{
	u32 step;
	u16 prev = s_pid_pwm_prev;

	if (raw_req > prev) {
		step = (u32)(raw_req - prev);
		if (step > (u32)PWM_SLEW_STEP_MAX)
			step = (u32)PWM_SLEW_STEP_MAX;
		prev = (u16)(prev + step);
	} else {
		step = (u32)(prev - raw_req);
		if (step > (u32)PWM_SLEW_STEP_MAX)
			step = (u32)PWM_SLEW_STEP_MAX;
		prev = (u16)(prev - step);
	}
	mt_pid.uk = (float)prev;
	s_pid_pwm_prev = prev;
	return prev;
}

void motor_speed_pid_isr(motor_t mot)
{
	int16_t cut_adc;
	u32 step;
	u16 duty_req;

	cut_adc = mot.valtage_up - mot.valtage_down;

	switch (motor.status) {
	case STATUS_MT_START:
		if (cut_adc < 0)
			cut_adc = 0;
		motor.stop_valtage = motor.adjust_min_voltage;
		ZHANKONBI = pid_calc(&mt_pid, motor.adjust_sta_voltage, (u16)cut_adc, mot.index);
		old_pwm = ZHANKONBI;
		break;
	case STATUS_MT_PID:
		if (cut_adc < 0)
			cut_adc = 0;
		if (!s_pid_sess_init) {
			s_pid_volt_cmd = motor.adjust_now_voltage;
			s_pid_pwm_prev = (u16)ZHANKONBI;
			s_pid_sess_init = 1u;
		} else {
			if (s_pid_volt_cmd < motor.adjust_now_voltage) {
				step = (u32)(motor.adjust_now_voltage - s_pid_volt_cmd);
				if (step > PID_VOLT_CMD_SLEW_PER_ISR)
					step = PID_VOLT_CMD_SLEW_PER_ISR;
				s_pid_volt_cmd = (u16)(s_pid_volt_cmd + step);
			} else if (s_pid_volt_cmd > motor.adjust_now_voltage) {
				step = (u32)(s_pid_volt_cmd - motor.adjust_now_voltage);
				if (step > PID_VOLT_CMD_SLEW_PER_ISR)
					step = PID_VOLT_CMD_SLEW_PER_ISR;
				s_pid_volt_cmd = (u16)(s_pid_volt_cmd - step);
			}
		}
		{
			u16 kiv_raw = compute_soft_kiv_raw(mot);
			u32 vsum = (u32)s_pid_volt_cmd + (u32)kiv_raw;
			u16 vcomb;

			if (vsum > 0xFFF0u)
				vcomb = 0xFFF0u;
			else
				vcomb = (u16)vsum;
			motor.adjust_kiv_voltage = vcomb;
			duty_req = ff_duty_from_volt_cmd(vcomb);
		}
		ZHANKONBI = pwm_apply_slew_step(duty_req);
		old_pwm = ZHANKONBI;
		Current_Max_Over();
		break;
	case STATUS_MT_STOP:
		motor.adjust_kiv_voltage = motor.adjust_now_voltage;

		if (cut_adc < 0)
			cut_adc = 0;
		valtage = (u8)(0.126f * (float)cut_adc);
		ZHANKONBI = pid_calc(&mt_pid, motor.adjust_kiv_voltage, (u16)cut_adc, mot.index);
		if ((motor.END_speed_scale >= valtage) || (motor.stop_time_late > 8u)) {
			motor.stop_time_late = 0u;
			motor.status = NULL;
			ctr.status = STATUS_TO_STOP;
			ZHANKONBI = MT_START_PWM;
		}
		if (motor.cur_speed_scale > 300u)
			Current_Max_Over();
		old_pwm = ZHANKONBI;
		break;
	default:
		break;
	}
	if (motor.status != STATUS_MT_PID)
		s_pid_sess_init = 0u;
}

void error_chick(void)
{
	if (motor.status == STATUS_MT_PID || motor.status == STATUS_MT_STOP) {
		checkout_current_over();
		checkout_speed_over();
	}
}
