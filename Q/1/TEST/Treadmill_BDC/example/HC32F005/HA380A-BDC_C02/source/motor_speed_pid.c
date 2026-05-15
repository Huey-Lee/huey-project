/* motor_speed_pid.c - speed PID ISR, OC/OV, runs in ADC tick. */
#include "pid.h"
#include "motor.h"
#include "motor_speed_pid.h"
#include "user_timer.h"

extern uint16_t ZHANKONBI;
extern pid_t mt_pid;

/* Inferred bus voltage (0.126 * sense ADC) for stop ramp compare (see STATUS_MT_STOP). */
u8 valtage;

u16 over_current = OVER_CURRENT_MAX;
u16 over_voltage = OVER_VOLTAGE_MAX;
u16 timestest = 0;

/* E03: low speed (scale <= 200) and current above low-speed cap for too long. */
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
			motor_hard_fault_set(ERROR_03);
		}
	}
}

/* E03: any-speed over-current, sustained. */
static void Current_Max_Over(void)
{
	static u8 Current_over_flag;
	if (motor.valtage_cur > over_current) {
		Current_over_flag++;
		if (Current_over_flag > 200) {
			Current_over_flag = 0;
			motor_hard_fault_set(ERROR_03);
		}
	} else
		Current_over_flag = 0;
}

/*
 * E08：指令电压 vs 实测 cut_adc（默认 adjust_now_voltage，不含 KIV）。
 * 原因：PWM 输出斜率限制后 PI 仍以 adjust_kiv_voltage 为目标时，实测会暂时跟不上 → 若 E08 也比 KIV 目标会持续误报。
 * 可选 E08_COMPARE_USE_KIV=1 恢复与 PI 同一目标（须放宽阈值或关掉输出斜率）。
 */
static void checkout_voltage_cmd_vs_sense(void)
{
	int16_t cut;
	u16 actual;
	u16 cmd_ref;
	u32 diff;
	static u8 mismatch_run;

	if (motor.status != STATUS_MT_PID && motor.status != STATUS_MT_STOP)
		return;

	cut = motor.valtage_up - motor.valtage_down;
	if (cut < 0)
		cut = 0;
	actual = (u16)cut;

#if E08_COMPARE_USE_KIV
	cmd_ref = motor.adjust_kiv_voltage;
#else
	cmd_ref = motor.adjust_now_voltage;
#endif

	if (cmd_ref > actual)
		diff = (u32)cmd_ref - (u32)actual;
	else
		diff = (u32)actual - (u32)cmd_ref;

	if (diff > (u32)VOLT_MISMATCH_RAW) {
		mismatch_run++;
		if (mismatch_run >= E08_MISMATCH_DEBOUNCE_CNT) {
			mismatch_run = 0;
			motor_hard_fault_set(ERROR_08);
		}
	} else
		mismatch_run = 0;
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

/* 限制真实占空每拍变化，防止 pid->uk 在猛踩时单帧顶到 UK_PWM_MAX */
static u16 s_pwm_applied_out;
static u8 s_pwm_prev_stat = 0xFFu;

static u16 pwm_output_slew_apply(u16 raw_req)
{
	u32 step;
	u16 prev = s_pwm_applied_out;

	if (raw_req > prev) {
		step = (u32)(raw_req - prev);
		if (step > (u32)PWM_OUTPUT_SLEW_PER_ISR)
			step = (u32)PWM_OUTPUT_SLEW_PER_ISR;
		prev = (u16)(prev + step);
	} else {
		step = (u32)(prev - raw_req);
		if (step > (u32)PWM_OUTPUT_SLEW_PER_ISR)
			step = (u32)PWM_OUTPUT_SLEW_PER_ISR;
		prev = (u16)(prev - step);
	}
	mt_pid.uk = (float)prev;
	s_pwm_applied_out = prev;
	return prev;
}

/* ~1.6 kHz (0.625 ms) in ADC / motor tick context. */
void motor_speed_pid_isr(motor_t mot)
{
	int16_t cut_adc;
	u16 raw_duty;

	cut_adc = mot.valtage_up - mot.valtage_down;

	if (motor.status != (u8)s_pwm_prev_stat) {
		if (motor.status == STATUS_MT_START || motor.status == STATUS_MT_PID
		    || motor.status == STATUS_MT_STOP)
			s_pwm_applied_out = (u16)ZHANKONBI;
		s_pwm_prev_stat = motor.status;
	}

	switch (motor.status) {
	case STATUS_MT_START:
		/* Open-loop ramp: target voltage vs sense; slow step via PID. */
		if (cut_adc < 0)
			cut_adc = 0;
		motor.stop_valtage = motor.adjust_min_voltage;
		raw_duty = pid_calc(&mt_pid, motor.adjust_sta_voltage, (u16)cut_adc, mot.index);
		ZHANKONBI = pwm_output_slew_apply(raw_duty);
		old_pwm = ZHANKONBI;
		break;
	case STATUS_MT_PID:
		/* PI 给定 = adjust_now_voltage + KIV；与反馈 cut_adc 比较同 pid_calc(target, actual, …) */
		if ((mot.valtage_cur - speed_param[mot.index].adc_base_curren) > 0) {
			motor.adjust_kiv_voltage = (u16)(((mot.valtage_cur - speed_param[mot.index].adc_base_curren) * motor.kiv) / 10);
			motor.adjust_kiv_voltage = motor.adjust_now_voltage + motor.adjust_kiv_voltage;
		} else
			motor.adjust_kiv_voltage = motor.adjust_now_voltage;

		raw_duty = pid_calc(&mt_pid, motor.adjust_kiv_voltage, (u16)cut_adc, mot.index);
		ZHANKONBI = pwm_output_slew_apply(raw_duty);
		old_pwm = ZHANKONBI;
		Current_Max_Over();
		checkout_voltage_cmd_vs_sense();
		break;
	case STATUS_MT_STOP:
		/* PI 给定同 PID 段：adjust_now_voltage + KIV */
		if ((mot.valtage_cur - speed_param[mot.index].adc_base_curren) > 0) {
			motor.adjust_kiv_voltage = (u16)(((mot.valtage_cur - speed_param[mot.index].adc_base_curren) * motor.kiv) / 10);
			motor.adjust_kiv_voltage = motor.adjust_now_voltage + motor.adjust_kiv_voltage;
		} else
			motor.adjust_kiv_voltage = motor.adjust_now_voltage;

		if (cut_adc < 0)
			cut_adc = 0;
		valtage = (u8)(0.126f * (float)cut_adc);
		raw_duty = pid_calc(&mt_pid, motor.adjust_kiv_voltage, (u16)cut_adc, mot.index);
		ZHANKONBI = pwm_output_slew_apply(raw_duty);
		checkout_voltage_cmd_vs_sense();
		if ((motor.END_speed_scale >= valtage) || (motor.stop_time_late > 8u)) {
			motor.stop_time_late = 0u;
			motor.status = NULL;
			ctr.status = STATUS_TO_STOP;
			ZHANKONBI = MT_START_PWM;
			s_pwm_applied_out = MT_START_PWM;
		}
		if (motor.cur_speed_scale > 300u)
			Current_Max_Over();
		old_pwm = ZHANKONBI;
		break;
	default:
		break;
	}
}

void error_chick(void)
{
	if (motor.status == STATUS_MT_PID || motor.status == STATUS_MT_STOP) {
		checkout_current_over();
		checkout_speed_over();
	}
}
