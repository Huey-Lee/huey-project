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

/* ~1.6 kHz (0.625 ms) in ADC / motor tick context. */
void motor_speed_pid_isr(motor_t mot)
{
	int16_t cut_adc;
	cut_adc = mot.valtage_up - mot.valtage_down;

	switch (motor.status) {
	case STATUS_MT_START:
		/* Open-loop ramp: target voltage vs sense; slow step via PID. */
		if (cut_adc < 0)
			cut_adc = 0;
		motor.stop_valtage = motor.adjust_min_voltage;
		/* 钳位：防止软起目标电压超过 adjust_max_voltage，110V 电机防飞车 */
		{ u16 _ms = (u16)((u32)motor.adjust_max_voltage * 813u / 100u);
		  if (motor.adjust_sta_voltage > _ms) motor.adjust_sta_voltage = _ms; }
		{ u16 _npwm = pid_calc(&mt_pid, motor.adjust_sta_voltage, (u16)cut_adc, mot.index);
		  /* 上升斜率限制：每ISR最多涨3，防踩上去瞬间PWM暴增电压噌 */
		  if (_npwm > old_pwm + 3u) { _npwm = old_pwm + 3u; mt_pid.uk = (float)_npwm; }
		  ZHANKONBI = _npwm; old_pwm = _npwm; }
		break;
	case STATUS_MT_PID:
		/* Closed loop: kiv term from current + PID on voltage error. */
		if ((mot.valtage_cur - speed_param[mot.index].adc_base_curren) > 0) {
			motor.adjust_kiv_voltage = (u16)(((mot.valtage_cur - speed_param[mot.index].adc_base_curren) * motor.kiv) / 10);
			motor.adjust_kiv_voltage = motor.adjust_now_voltage + motor.adjust_kiv_voltage;
		} else
			motor.adjust_kiv_voltage = motor.adjust_now_voltage;
		/* 钳位：KIV 补偿叠加后不超过 adjust_max_voltage，防踩上去时电压噌一下 */
		{ u16 _mk = (u16)((u32)motor.adjust_max_voltage * 813u / 100u);
		  if (motor.adjust_kiv_voltage > _mk) motor.adjust_kiv_voltage = _mk; }
		{ u16 _npwm = pid_calc(&mt_pid, motor.adjust_kiv_voltage, (u16)cut_adc, mot.index);
		  /* 上升斜率限制：每ISR最多涨3，防踩上去瞬间PWM暴增电压噌 */
		  if (_npwm > old_pwm + 3u) { _npwm = old_pwm + 3u; mt_pid.uk = (float)_npwm; }
		  ZHANKONBI = _npwm; old_pwm = _npwm; }
		Current_Max_Over();
		break;
	case STATUS_MT_STOP:
		/* PID 灏嗘湯娈靛崰绌鸿嚜鐒舵敹鍒扮害~MT_START_PWM 鍐嶅垽鍋? 涓嶅己鍒跺啓 0 鍐嶅叧 tim */
		if ((mot.valtage_cur - speed_param[mot.index].adc_base_curren) > 0) {
			motor.adjust_kiv_voltage = (u16)(((mot.valtage_cur - speed_param[mot.index].adc_base_curren) * motor.kiv) / 10);
			motor.adjust_kiv_voltage = motor.adjust_now_voltage + motor.adjust_kiv_voltage;
		} else
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
}

void error_chick(void)
{
	if (motor.status == STATUS_MT_PID || motor.status == STATUS_MT_STOP) {
		checkout_current_over();
		checkout_speed_over();
	}
}
