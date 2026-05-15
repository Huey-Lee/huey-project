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

/* E05: bus over-voltage from (valtage_up - valtage_down) for ~300 ms. */
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

u16 old_pwm = 0;

/* Two-tier voltage-difference protection (actual motor terminal vs commanded):
 *
 * Tier-1  VOLT_OVERSHOOT  20 V = 159 ADC  → E06 immediate cut
 *   Catches running speed overshoot (load suddenly removed, PID wind-up, etc.).
 *   Debounce: 60 ticks × ~45 µs ≈ 2.7 ms (avoids PID transient false triggers).
 *
 * Tier-2  MOS_SHORT       40 V = 317 ADC  → E08 immediate cut
 *   Catches MOS drain-source short (motor always powered, actual >> commanded).
 *   Debounce: 20 ticks × ~45 µs ≈ 900 µs  (< 1 ms hard requirement).
 *
 * 1 ADC count ≈ 103.2044 × 5 / 4096 = 0.126 V (bus-sense divider 1/100). */
#define VOLT_OVERSHOOT_DIFF   (159u)   /*  20 V */
#define VOLT_OVERSHOOT_CNT    (60u)
#define MOS_SHORT_VOLT_DIFF   (317u)   /*  40 V */
#define MOS_SHORT_CNT_MAX     (20u)

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
		ZHANKONBI = pid_calc(&mt_pid, motor.adjust_sta_voltage, (u16)cut_adc, mot.index);
		old_pwm = ZHANKONBI;
		break;
	case STATUS_MT_PID:
		/* Closed loop: kiv term from current + PID on voltage error. */
		if ((mot.valtage_cur - speed_param[mot.index].adc_base_curren) > 0) {
			motor.adjust_kiv_voltage = (u16)(((mot.valtage_cur - speed_param[mot.index].adc_base_curren) * motor.kiv) / 10);
			motor.adjust_kiv_voltage = motor.adjust_now_voltage + motor.adjust_kiv_voltage;
		} else
			motor.adjust_kiv_voltage = motor.adjust_now_voltage;

		ZHANKONBI = pid_calc(&mt_pid, motor.adjust_kiv_voltage, (u16)cut_adc, mot.index);
		old_pwm = ZHANKONBI;
		Current_Max_Over();
		{
			u16 actual_v = (cut_adc > 0) ? (u16)cut_adc : 0u;

			/* Tier-1: voltage overshoot → E06 immediate cut.
			 * actual > commanded + 15 V: motor spinning faster than intended
			 * (load removed, runaway, kiv wind-up). */
			{
				static u8 volt_overshoot_cnt;
				if (actual_v > motor.adjust_kiv_voltage + VOLT_OVERSHOOT_DIFF) {
					if (++volt_overshoot_cnt >= VOLT_OVERSHOOT_CNT) {
						volt_overshoot_cnt = 0;
						motor_hard_fault_set(ERROR_06);
					}
				} else {
					volt_overshoot_cnt = 0;
				}
			}

			/* Tier-2: MOS short-circuit → E08 immediate cut.
			 * actual > commanded + 40 V: MOS drain-source short, motor receives
			 * near-full bus voltage regardless of commanded duty. < 1 ms response. */
			{
				static u8 mos_short_cnt;
				if (actual_v > motor.adjust_kiv_voltage + MOS_SHORT_VOLT_DIFF) {
					if (++mos_short_cnt >= MOS_SHORT_CNT_MAX) {
						mos_short_cnt = 0;
						motor_hard_fault_set(ERROR_08);
					}
				} else {
					mos_short_cnt = 0;
				}
			}
		}
		break;
	case STATUS_MT_STOP:
		/* PID 将末段占空自然收到约~MT_START_PWM 再判停, 不强制写 0 再关 tim */
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

/* Speed overshoot: cur_speed_scale (commanded ramp) exceeds set_speed_scale (target)
 * by >= OVERSHOOT_MARGIN, sustained ~5 ISR ticks (~200 µs).
 * Triggers on emergency stop or sudden large speed-down command (≥ 2 km/h drop).
 * Responds with immediate cut (E06).
 * NOTE: cur_speed_scale is the software ramp value, not true motor speed; this
 * provides indirect overshoot protection in the absence of a tachometer. */
#define OVERSHOOT_MARGIN       (200u)
#define OVERSHOOT_CNT_MAX      (5u)
static void checkout_speed_overshoot(void)
{
	static u8 overshoot_cnt;
	if (motor.set_speed_scale > 0u &&
	    motor.cur_speed_scale >= (motor.set_speed_scale + OVERSHOOT_MARGIN)) {
		if (++overshoot_cnt > OVERSHOOT_CNT_MAX) {
			overshoot_cnt = 0;
			motor_hard_fault_set(ERROR_06);
		}
	} else {
		overshoot_cnt = 0;
	}
}

/* E06: speed-scale runaway above allowed max, sustained ~10 ticks. */
#define OVER_SPEED_CNT_MAX   (10u)
static void checkout_over_speed_scale(void)
{
	static u8 over_speed_cnt;
	if (motor.cur_speed_scale > (motor.adjust_speed_max + 100u)) {
		if (++over_speed_cnt > OVER_SPEED_CNT_MAX) {
			over_speed_cnt = 0;
			motor_hard_fault_set(ERROR_06);
		}
	} else {
		over_speed_cnt = 0;
	}
}

/* E03: over-current during open-loop start; same debounce as low-speed check. */
#define START_OC_CNT_MAX     (20u)
static void checkout_current_over_start(void)
{
	static u8 start_oc_cnt;
	if (motor.valtage_cur > OVER_CURRENT_MAX_LOW_SPEED) {
		if (++start_oc_cnt > START_OC_CNT_MAX) {
			start_oc_cnt = 0;
			motor_hard_fault_set(ERROR_03);
		}
	} else {
		start_oc_cnt = 0;
	}
}

/* E03: motor failed to start within timeout — belt jammed or open circuit.
 * start_tim is incremented every motor_speed() call (~91 ms/tick at 22 kHz PWM).
 * Timeout ~60 * 91 ms ≈ 5.5 s. */
#define START_STALL_TIMEOUT  (60u)
static void checkout_start_stall(void)
{
	if (motor.start_tim > START_STALL_TIMEOUT) {
		motor_hard_fault_set(ERROR_03);
	}
}

void error_chick(void)
{
	if (motor.status == STATUS_MT_START) {
		checkout_current_over_start();
		checkout_start_stall();
	}
	if (motor.status == STATUS_MT_PID || motor.status == STATUS_MT_STOP) {
		checkout_current_over();
		checkout_speed_over();
	}
	if (motor.status == STATUS_MT_PID || motor.status == STATUS_MT_START) {
		checkout_over_speed_scale();
	}
	if (motor.status == STATUS_MT_PID) {
		checkout_speed_overshoot();
	}
}
