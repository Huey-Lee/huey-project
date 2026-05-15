/* motor_speed_pid.c - 电机速度PID中断与故障保护（ADC节拍调用） */
#include "pid.h"
#include "motor.h"
#include "motor_speed_pid.h"
#include "user_timer.h"

extern uint16_t ZHANKONBI;
extern pid_t mt_pid;

/* 由母线差分ADC推算电机电压（0.126*ADC），用于停机阶段比较 */
u8 valtage;

u16 over_current = OVER_CURRENT_MAX;
u16 over_voltage = OVER_VOLTAGE_MAX;
u16 timestest = 0;

#define MOS_SHORT_DETECT_ADC      (OVER_CURRENT_MAX_LOW_SPEED)
#define MOS_SHORT_PWM_GUARD       (MT_START_PWM + 2u)
#define MOS_SHORT_DEBOUNCE_TICK   (12u)

#define BUS_LOW_VOLTAGE_TH        (10)  /* V，低压保护阈值 */
#define BUS_LOW_DEBOUNCE_TICK     (10u)

#define OVERSPEED_MARGIN_V        (18.0f)
#define OVERSPEED_PWM_LOW_TH      (MT_START_PWM + 8u)
#define OVERSPEED_DEBOUNCE_TICK   (25u)
#define LOW_SPEED_STALL_DEBOUNCE_TICK (60u) /* 约3秒，沿用当前节拍基准 */

#define MOTOR_OPEN_VOLTAGE_TH     (10)  /* V，疑似开路判据 */
#define MOTOR_SHORT_VOLTAGE_TH    (15)  /* V，疑似短路判据 */
#define MOTOR_OPEN_SHORT_PWM_TH   (MT_START_PWM + 8u)
#define MOTOR_OPEN_DEBOUNCE_TICK  (25u)
#define MOTOR_SHORT_DEBOUNCE_TICK (12u)

/* E03：低速过流堵转（速度<=200），约3秒去抖后上报 */
static void checkout_current_over(void)
{
	static u8 low_speed_count;
	if (motor.cur_speed_scale <= 200) {
		if (motor.valtage_cur > OVER_CURRENT_MAX_LOW_SPEED)
			low_speed_count++;
		else
			low_speed_count = 0;

		if (low_speed_count > LOW_SPEED_STALL_DEBOUNCE_TICK) {
			low_speed_count = 0;
			motor_hard_fault_set(ERROR_03);
		}
	}
}

/* E03：全速段持续过流保护 */
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

/* E05：母线过压保护（由valtage_up - valtage_down换算） */
static void checkout_speed_over(void)
{
	int val = 0, val_t;
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

/* E05：母线低压保护 */
static void checkout_speed_low(void)
{
	int val = 0, val_t;
	static u8 low_v_cnt;

	if (motor.valtage_up > motor.valtage_down)
		val = motor.valtage_up - motor.valtage_down;
	val_t = CALC_ADC_2_VOLTAGE(val);

	if (val_t <= BUS_LOW_VOLTAGE_TH)
		low_v_cnt++;
	else
		low_v_cnt = 0;

	if (low_v_cnt > BUS_LOW_DEBOUNCE_TICK) {
		low_v_cnt = 0;
		motor_hard_fault_set(ERROR_05);
	}
}

/* E08：MOS短路/开关异常保护（低PWM下电流异常偏高） */
static void checkout_mos_short(void)
{
	static u8 mos_short_cnt;
	if ((motor.status == STATUS_MT_STOP || motor.status == NULL) &&
	    (ZHANKONBI <= MOS_SHORT_PWM_GUARD) &&
	    (motor.valtage_cur > MOS_SHORT_DETECT_ADC)) {
		mos_short_cnt++;
	} else {
		mos_short_cnt = 0;
	}

	if (mos_short_cnt > MOS_SHORT_DEBOUNCE_TICK) {
		mos_short_cnt = 0;
		motor_hard_fault_set(ERROR_08);
	}
}

/* E02：电机断路（电气通路异常） */
static void checkout_motor_open_short(void)
{
	int val = 0;
	int val_t;
	static u8 open_cnt;
	static u8 short_cnt;

	if (motor.valtage_up > motor.valtage_down)
		val = motor.valtage_up - motor.valtage_down;
	val_t = CALC_ADC_2_VOLTAGE(val);

	if (motor.status == STATUS_MT_START || motor.status == STATUS_MT_PID) {
		if ((ZHANKONBI >= MOTOR_OPEN_SHORT_PWM_TH) &&
		    (val_t <= MOTOR_OPEN_VOLTAGE_TH))
			open_cnt++;
		else
			open_cnt = 0;

		if ((motor.valtage_cur > over_current) &&
		    (val_t <= MOTOR_SHORT_VOLTAGE_TH))
			short_cnt++;
		else
			short_cnt = 0;
	} else {
		open_cnt = 0;
		short_cnt = 0;
	}

	if (open_cnt > MOTOR_OPEN_DEBOUNCE_TICK || short_cnt > MOTOR_SHORT_DEBOUNCE_TICK) {
		open_cnt = 0;
		short_cnt = 0;
		motor_hard_fault_set(ERROR_02);
	}
}

/* E08：飞车疑似（低PWM但母线推算电压显著高于目标） */
static void checkout_over_speed(void)
{
	int val = 0;
	float val_t;
	float tgt_v;
	static u8 over_speed_cnt;

	if (motor.valtage_up > motor.valtage_down)
		val = motor.valtage_up - motor.valtage_down;
	val_t = (float)CALC_ADC_2_VOLTAGE(val);
	tgt_v = GET_SPEED_VOLTAGE((float)motor.cur_speed_scale, motor.voltage_param, motor.stop_valtage);

	if ((motor.status == STATUS_MT_PID) &&
	    (ZHANKONBI <= OVERSPEED_PWM_LOW_TH) &&
	    (val_t > (tgt_v + OVERSPEED_MARGIN_V))) {
		over_speed_cnt++;
	} else {
		over_speed_cnt = 0;
	}

	if (over_speed_cnt > OVERSPEED_DEBOUNCE_TICK) {
		over_speed_cnt = 0;
		motor_hard_fault_set(ERROR_08);
	}
}

u16 old_pwm = 0;

/* 约1.6kHz（0.625ms）在ADC/电机节拍中调用 */
void motor_speed_pid_isr(motor_t mot)
{
	int16_t cut_adc;
	cut_adc = mot.valtage_up - mot.valtage_down;

	switch (motor.status) {
	case STATUS_MT_START:
		/* 开环启动阶段：按启动目标电压与采样值做PID缓升 */
		if (cut_adc < 0)
			cut_adc = 0;
		motor.stop_valtage = motor.adjust_min_voltage;
		ZHANKONBI = pid_calc(&mt_pid, motor.adjust_sta_voltage, (u16)cut_adc, mot.index);
		old_pwm = ZHANKONBI;
		break;
	case STATUS_MT_PID:
		/* 闭环运行阶段：电流补偿(kiv)+电压误差PID */
		if ((mot.valtage_cur - speed_param[mot.index].adc_base_curren) > 0) {
			motor.adjust_kiv_voltage = (u16)(((mot.valtage_cur - speed_param[mot.index].adc_base_curren) * motor.kiv) / 10);
			motor.adjust_kiv_voltage = motor.adjust_now_voltage + motor.adjust_kiv_voltage;
		} else
			motor.adjust_kiv_voltage = motor.adjust_now_voltage;

		ZHANKONBI = pid_calc(&mt_pid, motor.adjust_kiv_voltage, (u16)cut_adc, mot.index);
		old_pwm = ZHANKONBI;
		Current_Max_Over();
		break;
	case STATUS_MT_STOP:
		/* 停机阶段：PID自然收占空到末段阈值，再进入TO_STOP，不强制立即清0 */
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
		checkout_speed_low();
		checkout_mos_short();
		checkout_over_speed();
	}
	if (motor.status == STATUS_MT_START || motor.status == STATUS_MT_PID) {
		checkout_motor_open_short();
	}
}
