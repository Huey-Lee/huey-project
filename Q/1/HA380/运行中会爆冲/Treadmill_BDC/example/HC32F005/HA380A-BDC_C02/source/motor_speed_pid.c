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
 * E08：实测母线（cut_adc）相对送入 pid 的给定（pid_target）单向偏高 —— 与 MOS 短路/桥臂失控抬高
 * 更一致；与 pid_calc 同节拍、同刻度，避免速度曲线与 KIV 脱节误判。
 * 减速时加大裕量，减轻回馈导致 actual>target 的误报。
 */
#if (E08_MOS_FAULT_DETECT_EN)
static void e08_mos_bus_runaway_check(u16 pid_target, u16 act_adc, u16 pwm)
{
	static u8 deb;
	u32 margin;
	u32 thr;

	if (motor.status != STATUS_MT_PID && motor.status != STATUS_MT_STOP) {
		deb = 0;
		return;
	}

	margin = (u32)E08_ACT_ABOVE_TARGET_ADC;
#if (E08_REGEN_EXTRA_ADC > 0)
	if (motor.set_speed_scale < motor.cur_speed_scale)
		margin += (u32)E08_REGEN_EXTRA_ADC;
#endif
	thr = (u32)pid_target + margin;

#if (E08_ASSIST_LOW_PWM_ONLY != 0)
	if (pwm > (u16)E08_PWM_ASSIST_MAX) {
		deb = 0;
		return;
	}
#endif

	if ((u32)act_adc > thr) {
		deb++;
		if (deb >= (u8)E08_MOS_FAULT_DEBOUNCE) {
			deb = 0;
			motor_hard_fault_set(ERROR_08);
		}
	} else
		deb = 0;
}

#if (E08_PWM_SPIKE_DETECT_EN)
/* 单周期占空比上升超过 pid 步进上限 → 非正常“爆冲”；prev_pwm==0 跳过（上电首拍） */
static void e08_pwm_spike_check(u16 prev_pwm, u16 new_pwm, u8 index)
{
	static u8 spike_deb;
	u16 lim;
	int32_t d;

	if (motor.status != STATUS_MT_PID && motor.status != STATUS_MT_STOP &&
	    motor.status != STATUS_MT_START) {
		spike_deb = 0;
		return;
	}
	if (prev_pwm == 0u) {
		spike_deb = 0;
		return;
	}
	d = (int32_t)new_pwm - (int32_t)prev_pwm;
	if (d <= 0) {
		spike_deb = 0;
		return;
	}
	lim = (index <= 1u)
		  ? (u16)((u16)UK_STEP_MAX_LOW_SPD + (u16)E08_PWM_SPIKE_STEP_MARGIN)
		  : (u16)((u16)UK_STEP_MAX + (u16)E08_PWM_SPIKE_STEP_MARGIN);
	if ((u32)d > (u32)lim) {
		spike_deb++;
		if (spike_deb >= (u8)E08_PWM_SPIKE_DEBOUNCE) {
			spike_deb = 0;
			motor_hard_fault_set(ERROR_08);
		}
	} else
		spike_deb = 0;
}
#endif
#endif

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

#if (PID_TARGET_VOLT_SLEW_MAX > 0)
static u16 pid_target_slew_last;
static u8 pid_target_slew_inited;
#endif

void motor_pid_target_filter_reset(void)
{
#if (PID_TARGET_VOLT_SLEW_MAX > 0)
	pid_target_slew_inited = 0;
#endif
}

/* KIV：在 adjust_now 上叠加 ΔI*kiv/10，并做硬限幅 */
static void motor_apply_kiv_to_adjust_now(motor_t mot)
{
	int32_t di = (int32_t)mot.valtage_cur - (int32_t)speed_param[mot.index].adc_base_curren;
	u16 base = motor.adjust_now_voltage;
	u32 extra = 0;

	if (di > 0) {
		float ef = ((float)di * motor.kiv) / 10.0f;
		if (ef > 0.0f)
			extra = (u32)(ef + 0.5f);
#if (KIV_VOLT_EXTRA_MAX_NUM > 0) && (KIV_VOLT_EXTRA_MAX_DEN > 0)
		{
			u32 max_extra = ((u32)base * (u32)KIV_VOLT_EXTRA_MAX_NUM) / (u32)KIV_VOLT_EXTRA_MAX_DEN;
			if (extra > max_extra)
				extra = max_extra;
		}
#endif
#if (KIV_VOLT_ADD_ABS_MAX > 0)
		if (extra > (u32)KIV_VOLT_ADD_ABS_MAX)
			extra = (u32)KIV_VOLT_ADD_ABS_MAX;
#endif
	}
	{
		u32 sum = (u32)base + extra;
		if (sum > 0xFFFFu)
			motor.adjust_kiv_voltage = 0xFFFF;
		else
			motor.adjust_kiv_voltage = (u16)sum;
	}
}

/* 限制 pid 电压目标阶跃，减轻 ADC/目标突变时的冲击 */
static u16 motor_pid_target_slew_limit(u16 raw_target)
{
#if (PID_TARGET_VOLT_SLEW_MAX <= 0)
	return raw_target;
#else
	u16 step = (u16)PID_TARGET_VOLT_SLEW_MAX;
	u16 out;

	if (!pid_target_slew_inited) {
		out = raw_target;
		pid_target_slew_last = out;
		pid_target_slew_inited = 1;
		return out;
	}
	out = pid_target_slew_last;
	if (raw_target > out) {
		u16 d = raw_target - out;
		if (d > step)
			out = (u16)(out + step);
		else
			out = raw_target;
	} else {
		u16 d = out - raw_target;
		if (d > step)
			out = (u16)(out - step);
		else
			out = raw_target;
	}
	pid_target_slew_last = out;
	return out;
#endif
}

/* ~1.6 kHz (0.625 ms) in ADC / motor tick context. */
void motor_speed_pid_isr(motor_t mot)
{
	int16_t cut_adc;
	u16 pid_target;
	cut_adc = mot.valtage_up - mot.valtage_down;

	switch (motor.status) {
	case STATUS_MT_START:
		/* Open-loop ramp: target voltage vs sense; slow step via PID. */
		if (cut_adc < 0)
			cut_adc = 0;
		motor.stop_valtage = motor.adjust_min_voltage;
#if (E08_MOS_FAULT_DETECT_EN) && (E08_PWM_SPIKE_DETECT_EN)
		{
			u16 prev_pwm = old_pwm;
			ZHANKONBI = pid_calc(&mt_pid, motor.adjust_sta_voltage, (u16)cut_adc, mot.index);
			e08_pwm_spike_check(prev_pwm, ZHANKONBI, mot.index);
		}
#else
		ZHANKONBI = pid_calc(&mt_pid, motor.adjust_sta_voltage, (u16)cut_adc, mot.index);
#endif
		old_pwm = ZHANKONBI;
		break;
	case STATUS_MT_PID:
		/* Closed loop: kiv term from current + PID on voltage error. */
		motor_apply_kiv_to_adjust_now(mot);
		pid_target = motor_pid_target_slew_limit(motor.adjust_kiv_voltage);
#if (E08_MOS_FAULT_DETECT_EN)
		{
#if (E08_PWM_SPIKE_DETECT_EN)
			u16 prev_pwm = old_pwm;
#endif
			ZHANKONBI = pid_calc(&mt_pid, pid_target, (u16)cut_adc, mot.index);
#if (E08_PWM_SPIKE_DETECT_EN)
			e08_pwm_spike_check(prev_pwm, ZHANKONBI, mot.index);
#endif
			{
				u16 act = (cut_adc < 0) ? 0u : (u16)cut_adc;
				e08_mos_bus_runaway_check(pid_target, act, ZHANKONBI);
			}
		}
#else
		ZHANKONBI = pid_calc(&mt_pid, pid_target, (u16)cut_adc, mot.index);
#endif
		old_pwm = ZHANKONBI;
		Current_Max_Over();
		break;
	case STATUS_MT_STOP:
		/* PID 将末段占空自然收到约~MT_START_PWM 再判停, 不强制写 0 再关 tim */
		motor_apply_kiv_to_adjust_now(mot);
		if (cut_adc < 0)
			cut_adc = 0;
		valtage = (u8)(0.126f * (float)cut_adc);
		pid_target = motor_pid_target_slew_limit(motor.adjust_kiv_voltage);
#if (E08_MOS_FAULT_DETECT_EN)
		{
#if (E08_PWM_SPIKE_DETECT_EN)
			u16 prev_pwm = old_pwm;
#endif
			ZHANKONBI = pid_calc(&mt_pid, pid_target, (u16)cut_adc, mot.index);
#if (E08_PWM_SPIKE_DETECT_EN)
			e08_pwm_spike_check(prev_pwm, ZHANKONBI, mot.index);
#endif
			e08_mos_bus_runaway_check(pid_target, (u16)cut_adc, ZHANKONBI);
		}
#else
		ZHANKONBI = pid_calc(&mt_pid, pid_target, (u16)cut_adc, mot.index);
#endif
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
}

void error_chick(void)
{
	if (motor.status == STATUS_MT_PID || motor.status == STATUS_MT_STOP) {
		checkout_current_over();
		checkout_speed_over();
	}
}
