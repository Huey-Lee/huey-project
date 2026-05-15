/* motor_speed_pid.c - speed PID ISR, OC/OV, runs in ADC tick. */
#include "pid.h"
#include "motor.h"
#include "motor_speed_pid.h"
#include "user_timer.h"

extern uint16_t ZHANKONBI;

/* 运行阶段前馈模型：V = 0.084 * (Speed-100) + 18 */
#define FF_SPEED_BASE_SCALE            (100u)  /* speed_scale=100 对应 1.0km/h */
#define FF_VOLTAGE_SLOPE               (0.084f)
#define FF_VOLTAGE_OFFSET              (18.0f)
#define FF_ADC_PER_VOLT                (8.13f)
#define SOFT_KIV_LIMIT_PERCENT         (12u)
#define SOFT_KIV_ADC_MAX               (81u)
#define PWM_SLEW_STEP                  (1u)

/* 起步阶段：导引斜坡 + 重载踢动；起点/时长/终点由上控 CMD_SOFT_* 与 VOLTAGE_MIN 配置 */
#define START_RAMP_STEP_ADC            (1u)    /* 预留 */
#define START_RAMP_STEP_VOLTAGE        (0.1f)  /* 每步 +0.1V */
#define START_RAMP_DONE_EPS_V          (0.05f) /* 判满容差 */
#define START_BEMF_DEBOUNCE_TICKS      (8u)    /* 反电势连续满足次数，防止误切入运行 */
#define START_HEAVY_LOAD_V_ADC         (122u)  /* 重载判定电压门槛，约 15V */
#define START_HEAVY_LOAD_I_DELTA       (45u)   /* 重载判定电流增量门槛 */
#define START_MOVING_BEMF_ADC          (22u)   /* 检测“已转动”的反电势门槛 */
#define START_KICK_ADC                 (24u)   /* 踢动脉冲增量，约 +3V */
#define START_KICK_TICKS               (8u)    /* 踢动脉冲持续拍数 */
#define START_KICK_COOLDOWN_TICKS      (80u)   /* 踢动冷却时间，防止连续触发 */

/* 停止阶段：纯开环线性下滑；分频/地板默认见 motor_t.cfg_* */

/* 电压外环：指令电压(V) 跟踪 ADC 换算的实测端压 */
#define V_LOOP_KP                      (6.0f)
#define V_LOOP_CORR_MAX                (45)

/* 实测超过上控 adjust_max：安全兜底下拉占空（非主调节） */
#define V_RUN_OVER_SOFT_V              (0.12f)
#define V_RUN_OVER_MID_V               (0.55f)
#define V_RUN_OVER_HARD_V              (1.2f)
#define V_RUN_PWM_DROP_MIN             (14u)
#define V_RUN_PWM_DROP_PER_V           (10u)
#define V_RUN_PWM_DROP_MID_MAX         (50u)
#define V_RUN_PWM_DROP_HARD_MAX        (90u)
#define V_RUN_FAULT_OVER_V             (6.0f)   /* 仍长期高出最高电压 → 软故障减停 */
#define V_RUN_FAULT_DEBOUNCE_ISR       (120u)

u16 old_pwm = 0;

/* 运行电压上/下限对应的 ADC，与 adjust_min/max 一致 */
static u16 adc_cap_max(void)
{
	return (u16)((float)motor.adjust_max_voltage * FF_ADC_PER_VOLT);
}

static u16 adc_cap_min(void)
{
	return (u16)((float)motor.adjust_min_voltage * FF_ADC_PER_VOLT);
}

static u16 ff_voltage_to_pwm(u16 target_adc)
{
	u16 adc_span;
	u32 pwm;
	{
		u16 cmax = adc_cap_max();
		u16 cmin = adc_cap_min();
		if (target_adc > cmax) {
			target_adc = cmax;
		}
		if (target_adc < cmin) {
			target_adc = cmin;
		}
	}

	adc_span = (u16)((float)motor.adjust_max_voltage * FF_ADC_PER_VOLT);
	/* 除数过小会把 PWM 直接顶到 UK_PWM_MAX，表现为“一启动就 28~29V” */
	if (adc_span < (u16)(24.0f * FF_ADC_PER_VOLT)) {
		adc_span = (u16)(90.0f * FF_ADC_PER_VOLT);
	}

	pwm = ((u32)target_adc * (u32)MOTOR_PWM_TOP_AT_VMAX) / (u32)adc_span;
	if (pwm > (u32)MOTOR_PWM_TOP_AT_VMAX) {
		pwm = (u32)MOTOR_PWM_TOP_AT_VMAX;
	}
	if (pwm > (u32)UK_PWM_MAX) {
		pwm = (u32)UK_PWM_MAX;
	}
	if (pwm < UK_PWM_MIN) {
		pwm = UK_PWM_MIN;
	}
	return (u16)pwm;
}

static u16 ff_target_voltage_adc(u16 speed_scale)
{
	float ff_voltage;
	u16 ff_adc;

	if (speed_scale <= FF_SPEED_BASE_SCALE) {
		ff_voltage = FF_VOLTAGE_OFFSET;
	} else {
		ff_voltage = FF_VOLTAGE_SLOPE * (float)(speed_scale - FF_SPEED_BASE_SCALE) + FF_VOLTAGE_OFFSET;
	}

	if (ff_voltage < (float)motor.adjust_min_voltage) {
		ff_voltage = (float)motor.adjust_min_voltage;
	}
	if (ff_voltage > (float)motor.adjust_max_voltage) {
		ff_voltage = (float)motor.adjust_max_voltage;
	}

	ff_adc = (u16)(ff_voltage * FF_ADC_PER_VOLT);
	return ff_adc;
}

static u16 soft_kiv_ff_adc(motor_t mot, u16 base_adc)
{
	int16_t delta_current;
	u16 extra_adc = 0u;
	u16 limit_by_percent;
	u16 final_limit;

	delta_current = (int16_t)mot.valtage_cur - (int16_t)speed_param[mot.index].adc_base_curren;
	if (delta_current > 0) {
		extra_adc = (u16)(((u16)delta_current * motor.kiv) / 10.0f);
	}

	limit_by_percent = (u16)(((u32)base_adc * SOFT_KIV_LIMIT_PERCENT) / 100u);
	final_limit = (limit_by_percent < SOFT_KIV_ADC_MAX) ? limit_by_percent : SOFT_KIV_ADC_MAX;
	if (extra_adc > final_limit) {
		extra_adc = final_limit;
	}

	{
		u16 sum = (u16)(base_adc + extra_adc);
		u16 cap = adc_cap_max();
		if (sum > cap) {
			sum = cap;
		}
		return sum;
	}
}

static u16 apply_pwm_slew_limit(u16 target_pwm)
{
	if (old_pwm + PWM_SLEW_STEP < target_pwm) {
		return (u16)(old_pwm + PWM_SLEW_STEP);
	}
	if (old_pwm > target_pwm + PWM_SLEW_STEP) {
		return (u16)(old_pwm - PWM_SLEW_STEP);
	}
	return target_pwm;
}

static u16 apply_pwm_slew_limit_start(u16 target_pwm)
{
	if (old_pwm + PWM_SLEW_STEP < target_pwm) {
		return (u16)(old_pwm + PWM_SLEW_STEP);
	}
	if (old_pwm > target_pwm + PWM_SLEW_STEP) {
		return (u16)(old_pwm - PWM_SLEW_STEP);
	}
	return target_pwm;
}

/* 实测端压(V)：与 checkout_speed_over 同一换算，与指令同一物理量 */
static float motor_sense_voltage_v(int16_t cut_adc_raw)
{
	int v = (int)cut_adc_raw;
	if (v < 0) {
		v = 0;
	}
	return (float)CALC_ADC_2_VOLTAGE(v);
}

/* V_cmd：目标端压(V)；前馈 PWM + 比例修正，使 V_meas 逼近 V_cmd */
static u16 voltage_track_pwm(float V_cmd, int16_t cut_adc_raw, u16 (*slew_apply)(u16))
{
	u16 pwm_ff;
	float V_meas;
	float err;
	int corr;
	int out;
	float v_max = (float)motor.adjust_max_voltage;
	float v_min = (float)motor.adjust_min_voltage;

	if (V_cmd > v_max) {
		V_cmd = v_max;
	}
	if (V_cmd < v_min) {
		V_cmd = v_min;
	}

	V_meas = motor_sense_voltage_v(cut_adc_raw);
	motor.motor_valtage = V_meas;

	pwm_ff = ff_voltage_to_pwm((u16)(V_cmd * FF_ADC_PER_VOLT));

	if (V_cmd < 0.5f) {
		out = (int)pwm_ff;
	} else {
		err = V_cmd - V_meas;
		corr = (int)(V_LOOP_KP * err);
		if (corr > (int)V_LOOP_CORR_MAX) {
			corr = (int)V_LOOP_CORR_MAX;
		}
		if (corr < -(int)V_LOOP_CORR_MAX) {
			corr = -(int)V_LOOP_CORR_MAX;
		}
		out = (int)pwm_ff + corr;
	}

	if (out > (int)MOTOR_PWM_TOP_AT_VMAX) {
		out = (int)MOTOR_PWM_TOP_AT_VMAX;
	}
	if (out > (int)UK_PWM_MAX) {
		out = (int)UK_PWM_MAX;
	}
	if (out < (int)UK_PWM_MIN) {
		out = (int)UK_PWM_MIN;
	}

	{
		u16 tgt = (u16)out;
		float over = V_meas - v_max;

		if (over > V_RUN_OVER_HARD_V) {
			u16 drop = (u16)((float)V_RUN_PWM_DROP_MIN + over * (float)V_RUN_PWM_DROP_PER_V);
			if (drop > V_RUN_PWM_DROP_HARD_MAX) {
				drop = V_RUN_PWM_DROP_HARD_MAX;
			}
			if (old_pwm > UK_PWM_MIN + drop) {
				tgt = old_pwm - drop;
			} else {
				tgt = UK_PWM_MIN;
			}
			return tgt;
		}
		if (over > V_RUN_OVER_MID_V) {
			u16 drop = (u16)(8.0f + over * (float)V_RUN_PWM_DROP_PER_V);
			if (drop > V_RUN_PWM_DROP_MID_MAX) {
				drop = V_RUN_PWM_DROP_MID_MAX;
			}
			if (old_pwm > UK_PWM_MIN + drop) {
				tgt = old_pwm - drop;
			} else {
				tgt = UK_PWM_MIN;
			}
			if (tgt > (u16)out) {
				tgt = (u16)out;
			}
			return tgt;
		}
		if (over > V_RUN_OVER_SOFT_V) {
			if (tgt > old_pwm) {
				tgt = old_pwm;
			}
			return slew_apply(tgt);
		}
	}
	return slew_apply((u16)out);
}

/* Inferred bus voltage (0.126 * sense ADC) for stop ramp compare (see STATUS_MT_STOP). */
u8 valtage;

u16 over_current = OVER_CURRENT_MAX;
u16 over_voltage = OVER_VOLTAGE_MAX;
u16 timestest = 0;

/* E08：低速区过流检测（带去抖） */
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

/* E08：全速域持续过流检测 */
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

/* E05：母线过压检测（上/下桥臂差分电压） */
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

/* 主控制 ISR：约 1.6kHz（0.625ms） */
void motor_speed_pid_isr(motor_t mot)
{
	int16_t cut_adc;
	u16 target_adc;
	u16 ramp_end_adc;
	float ramp_end_v;
	u16 ramp_iv;
	u8 stop_div;
	u16 stop_floor;
	static u8 start_ramp_div_cnt;
	static u8 start_kick_ticks;
	static u8 start_kick_cooldown;
	static u8 stop_ramp_div_cnt;
	static u8 start_stage_inited;
	static float start_ramp_voltage;
	static u8 start_bemf_debounce;
	cut_adc = mot.valtage_up - mot.valtage_down;
	ramp_end_v = (float)motor.adjust_min_voltage;
	ramp_end_adc = (u16)(ramp_end_v * FF_ADC_PER_VOLT);
	ramp_iv = motor.cfg_start_ramp_interval_ticks;
	if (ramp_iv < 1u) {
		ramp_iv = 1u;
	}
	stop_div = motor.cfg_stop_ramp_div;
	if (stop_div == 0u) {
		stop_div = 24u;
	}
	stop_floor = motor.cfg_stop_floor_pwm;
	if (stop_floor == 0u) {
		stop_floor = MOTOR_STOP_PWM_FLOOR_DEFAULT;
	}

	switch (motor.status) {
	case STATUS_MT_START:
		/* 起步：纯开环导引斜坡 + 重载踢动 */
		if (cut_adc < 0)
			cut_adc = 0;
		motor.stop_valtage = motor.adjust_min_voltage;
		if (start_stage_inited == 0u) {
			start_stage_inited = 1u;
			start_ramp_div_cnt = 0u;
			start_kick_ticks = 0u;
			start_kick_cooldown = 0u;
			start_bemf_debounce = 0u;
			start_ramp_voltage = (float)motor.cfg_ramp_start_volt;
		}

		/* 爬到 adjust_min_voltage（运行最小电压），步进间隔由上控缓起时长换算 */
		if (++start_ramp_div_cnt >= ramp_iv) {
			start_ramp_div_cnt = 0u;
			if (start_ramp_voltage < (ramp_end_v - START_RAMP_DONE_EPS_V)) {
				start_ramp_voltage += START_RAMP_STEP_VOLTAGE;
			}
			if (start_ramp_voltage > ramp_end_v) {
				start_ramp_voltage = ramp_end_v;
			}
		}
		motor.adjust_sta_voltage = (u16)(start_ramp_voltage * FF_ADC_PER_VOLT);

		if (start_kick_cooldown > 0u) {
			start_kick_cooldown--;
		}

		if (motor.adjust_sta_voltage >= START_HEAVY_LOAD_V_ADC &&
			mot.valtage_cur > (u16)(speed_param[mot.index].adc_base_curren + START_HEAVY_LOAD_I_DELTA) &&
			(u16)cut_adc < START_MOVING_BEMF_ADC &&
			start_kick_cooldown == 0u &&
			start_kick_ticks == 0u) {
			start_kick_ticks = START_KICK_TICKS;
			start_kick_cooldown = START_KICK_COOLDOWN_TICKS;
		}

		target_adc = motor.adjust_sta_voltage;
		if (start_kick_ticks > 0u) {
			start_kick_ticks--;
			target_adc = (u16)(target_adc + START_KICK_ADC);
		} else {
			/* 非踢动：不超过当前爬升值，且不超过运行最小电压对应 ADC */
			if (target_adc > ramp_end_adc) {
				target_adc = ramp_end_adc;
			}
		}

		/* 指令电压 = target_adc/8.13，与实测 CALC_ADC_2_VOLTAGE 同一标尺闭环 */
		ZHANKONBI = voltage_track_pwm((float)target_adc / FF_ADC_PER_VOLT, cut_adc,
					      apply_pwm_slew_limit_start);
		old_pwm = ZHANKONBI;

		/* 必须先把起步电压→运行最小电压爬完，再凭反电势（去抖）切入前馈运行 */
		if (start_ramp_voltage >= (ramp_end_v - START_RAMP_DONE_EPS_V)) {
			if ((u16)cut_adc >= START_MOVING_BEMF_ADC) {
				if (start_bemf_debounce < START_BEMF_DEBOUNCE_TICKS) {
					start_bemf_debounce++;
				}
			} else {
				start_bemf_debounce = 0u;
			}
			if (start_bemf_debounce >= START_BEMF_DEBOUNCE_TICKS) {
				motor.status = STATUS_MT_PID;
				motor.start_tim = 0u;
				start_stage_inited = 0u;
				start_bemf_debounce = 0u;
			}
		} else {
			start_bemf_debounce = 0u;
		}
		break;
	case STATUS_MT_PID:
		/* 运行：前馈 + 软 KIV 目标电压，电压外环使输出端压与指令一致 */
		{
			static u8 v_run_fault_cnt;
			float vm;
			float vlim = (float)motor.adjust_max_voltage;

			motor.adjust_now_voltage = ff_target_voltage_adc(motor.cur_speed_scale);
			motor.adjust_kiv_voltage = soft_kiv_ff_adc(mot, motor.adjust_now_voltage);
			ZHANKONBI = voltage_track_pwm((float)motor.adjust_kiv_voltage / FF_ADC_PER_VOLT, cut_adc,
						      apply_pwm_slew_limit);
			old_pwm = ZHANKONBI;
			vm = motor.motor_valtage;
			if (vm > vlim + V_RUN_FAULT_OVER_V) {
				if (++v_run_fault_cnt >= V_RUN_FAULT_DEBOUNCE_ISR) {
					v_run_fault_cnt = 0u;
					motor_soft_fault_set(ERROR_05);
				}
			} else {
				v_run_fault_cnt = 0u;
			}
		}
		Current_Max_Over();
		break;
	case STATUS_MT_STOP:
		/* 停止：纯开环 PWM 线性递减 + 反电势判停 */
		start_stage_inited = 0u;
		if (cut_adc < 0) {
			cut_adc = 0;
		}
		valtage = (u8)(0.126f * (float)cut_adc);

		if (++stop_ramp_div_cnt >= stop_div) {
			stop_ramp_div_cnt = 0u;
			if (ZHANKONBI > stop_floor) {
				ZHANKONBI--;
			}
		}

		if ((motor.END_speed_scale >= valtage) || (motor.stop_time_late > 8u)) {
			motor.stop_time_late = 0u;
			motor.status = NULL;
			ctr.status = STATUS_TO_STOP;
			ZHANKONBI = stop_floor;
		}
		if (motor.cur_speed_scale > 300u)
			Current_Max_Over();
		old_pwm = ZHANKONBI;
		break;
	default:
		start_stage_inited = 0u;
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
