/* motor_speed_pid.c - 转速PID中断服务，过流/过压检测，在ADC采样节拍中运行 */
#include "pid.h"
#include "motor.h"
#include "motor_speed_pid.h"
#include "user_timer.h"

extern uint16_t ZHANKONBI;
extern pid_t mt_pid;

/* 母线电压估算值（0.126×采样ADC），用于停机末段比较 */
u8 valtage;

u16 over_current         = OVER_CURRENT_MAX;
u16 over_current_low_spd = OVER_CURRENT_MAX_LOW_SPEED;   /* 低速堵转门限，默认6A；由CMD_OVER_CURRENT_LOW_SPD更新 */
u16 over_voltage_adc     = (u16)(90u * 1032u / 100u);    /* 默认90V×130%≈117V，ADC≈928；由CMD_VOLTAGE_MAX更新 */
u16 timestest = 0;

/* E03：过流保护
 *
 * 低速堵转（E03）：scale≤200，电流超限持续约3s触发
 *   8000次×0.625ms=5.0s
 *
 * 全速过流（E03）：任意转速，电流超限约125ms触发
 *   200次×0.625ms=125ms
 *
 * E02断线：仅由上电/启动前静态桥检（check_bridge_static）检测 */
static void checkout_current_over(void)
{
	static u16 low_spd_cnt  = 0;
	static u8  any_spd_cnt  = 0;

	/* 低速堵转（E03）：覆盖 0~4km/h（scale ≤ 400）
	 * 原范围 scale≤200（≤2km/h），2~4km/h 存在盲区——人踩停皮带无报错。
	 * 4km/h 正常带载电流约 3~6A，低于门限 8.4A（over_current_low_spd），
	 * 扩展到 scale≤400 不会误触，但可以检测到人踩停（电流激增超门限持续5s）。 */
	if (motor.cur_speed_scale <= 400u) {
		if (motor.valtage_cur > over_current_low_spd) {
			if (++low_spd_cnt > 8000u) {
				low_spd_cnt = 0;
				motor_hard_fault_set(ERROR_03);
				return;
			}
		} else {
			low_spd_cnt = 0;
		}
	} else {
		low_spd_cnt = 0;
	}

	/* 全速过流 */
	if (motor.valtage_cur > over_current) {
		if (++any_spd_cnt > 200u) {
			any_spd_cnt = 0;
			motor_hard_fault_set(ERROR_03);
		}
	} else {
		any_spd_cnt = 0;
	}
}

/* E08：给定电压与实测电压偏差超 40V → 2次ISR（≈1ms）立即停机
 *
 * 监控范围：仅 STATUS_MT_START 和 STATUS_MT_PID
 *   STATUS_MT_STOP 排除：减速时 back-EMF 自然高于下降目标，属正常物理现象，
 *   强行监控会在每次减速时误报 E08。绝对过压由 E05 兜底。
 *
 * 基准修正：
 *   START态：以 motor.adjust_sta_voltage 为基准（与cut_adc同量纲）
 *   PID  态：以 motor.adjust_kiv_voltage × 977/1000 为基准
 *            KIV已含载荷补偿，用adjust_now_voltage会误把正常KIV升压计为爆冲
 *
 * 触发：连续 VOLT_RUNAWAY_TRIP_ACC（2）次超门限 → 2×0.625ms ≈ 1ms */
static void checkout_voltage_runaway(motor_t mot)
{
	static u8 run_over_acc = 0;
	int16_t cut_adc = (int16_t)mot.valtage_up - (int16_t)mot.valtage_down;
	u16     target_adc;
	int16_t excess;

	if (cut_adc < 0) cut_adc = 0;

	/* 已在故障态，不重复触发 */
	if (ctr.status == STATUS_ERROR) {
		run_over_acc = 0;
		return;
	}

	if (motor.status == STATUS_MT_START) {
		/* START态：adjust_sta_voltage 与 cut_adc 量纲相同，直接比较 */
		target_adc = motor.adjust_sta_voltage;
		excess = (int16_t)cut_adc - (int16_t)(target_adc + VOLT_RUN_OVER_ADC);
	} else if (motor.status == STATUS_MT_PID) {
		/* PID态：以实际PID目标（含KIV补偿）换算至cut_adc量纲 */
		target_adc = (u16)((u32)motor.adjust_kiv_voltage * 977u / 1000u);
		excess = (int16_t)cut_adc - (int16_t)(target_adc + VOLT_RUN_OVER_ADC);
	} else {
		/* STOP态及其他：不做E08监控，E05负责绝对过压保护 */
		run_over_acc = 0;
		return;
	}

	if (excess > 0) {
		if (++run_over_acc >= VOLT_RUNAWAY_TRIP_ACC) {
			run_over_acc = 0;
			motor_hard_fault_set(ERROR_08);
		}
	} else {
		run_over_acc = 0;
	}
}

/* E05/E06：母线过压/欠压分级保护
 *
 * 过压（E05）：阈值 = 上控下发最大电压×130%（由CMD_VOLTAGE_MAX更新over_voltage_adc）
 *              带消退（cnt--）过滤瞬态毛刺；持续超压 → 硬停防炸电容
 *
 * 欠压（E06）：双通道，运行态检测，受控减停 */
static void checkout_bus_voltage(void)
{
	static u8  ov_cnt = 0;
	static u16 uv_cnt = 0;
	u16 cut_adc;

	cut_adc = (motor.valtage_up > motor.valtage_down) ?
	          (motor.valtage_up - motor.valtage_down) : 0u;

	/* 过压：直接与预换算缓存值比较，无运算，带消退滤毛刺 */
	if (cut_adc >= over_voltage_adc) {
		if (ov_cnt < 255u) ov_cnt++;
	} else {
		if (ov_cnt > 0u) ov_cnt--;
	}
	if (ov_cnt > BUS_OV_TRIP_COUNT) {
		ov_cnt = 0;
		uv_cnt = 0;
		motor_hard_fault_set(ERROR_05);
		return;
	}

	/* 欠压双通道检测（START/PID态）：
	 *
	 * 快速通道：PWM已打满(PID饱和)且实测<目标×94%
	 *   触发时机：供电限压后，scale爬坡到饱和点（约40s）再约2s触发
	 *   物理含义：占空100%仍追不上目标，供电被硬性封顶
	 *
	 * 慢速通道：非饱和时实测<目标×80%（高阻软电源场景）
	 *   长延时，不被启动瞬间电压下沉误触 */
	if (motor.status == STATUS_MT_PID ||
	    motor.status == STATUS_MT_START) {
		if (cut_adc > 0u) {
			u8 is_sat = (ZHANKONBI >= (u16)(UK_PWM_MAX - 2));
			u8 pct    = is_sat ? BUS_UV_SAT_PCT : BUS_UV_PCT;
			u16 limit = is_sat ? BUS_UV_SAT_COUNT : BUS_UV_TRIP_COUNT;
			/* 低速排除策略：
			 * 慢速通道（非饱和）：scale<200（≈2km/h）时跳过——轻踩皮带back-EMF极小易误报
			 * 快速通道（饱和）  ：scale<BUS_UV_SAT_LOW_SPD_SCALE（≈3km/h）也跳过——
			 *   低速时KIV=0导致任何大载荷均饱和，属正常物理现象而非真实欠压；
			 *   慢速通道（900ms）已在高层兜底真实欠压 */
			if (is_sat == 0u && motor.cur_speed_scale < 200u) {
				uv_cnt = 0;
			} else if (is_sat == 1u && motor.cur_speed_scale < BUS_UV_SAT_LOW_SPD_SCALE) {
				uv_cnt = 0;
			} else if ((u32)cut_adc * 100u <
			           (u32)motor.adjust_now_voltage * (u32)pct) {
				if (++uv_cnt > limit) {
					uv_cnt = 0;
					motor_controlled_stop_set(ERROR_06);
				}
			} else {
				uv_cnt = 0;
			}
		} else {
			uv_cnt = 0;
		}
	} else {
		uv_cnt = 0;
	}
}

u16 old_pwm = 0;

/* ADC采样中断节拍，约1.6kHz（0.625ms/次） */
void motor_speed_pid_isr(motor_t mot)
{
	int16_t cut_adc;
	cut_adc = mot.valtage_up - mot.valtage_down;

	switch (motor.status) {
	case STATUS_MT_START:
		/* 开环启动：目标电压爬坡，PID跟随，加上升斜率限制 */
		if (cut_adc < 0)
			cut_adc = 0;
		motor.stop_valtage = motor.adjust_min_voltage;
		{
			u16 new_pwm = pid_calc(&mt_pid, motor.adjust_sta_voltage, (u16)cut_adc, mot.index);
			if (new_pwm > old_pwm + ZHANKONBI_SLEW_UP) {
				new_pwm = old_pwm + ZHANKONBI_SLEW_UP;
				mt_pid.uk = (float)new_pwm;   /* 反积分饱和：回写限幅后值 */
			}
			ZHANKONBI = new_pwm;
			old_pwm   = new_pwm;
		}
		break;
	case STATUS_MT_PID:
		/* 闭环：电流前馈KIV + 电压PID
		 * 管道：pid_calc → 前馈限幅 → SLEW限幅 → 输出
		 *
		 * ① KIV计算（载荷前馈）
		 * ② pid_calc：增量式PID，含饱和抑制（clamping anti-windup）
		 * ③ 前馈限幅（PID_FF_WINDUP_RANGE）：uk不超前馈PWM±81单位
		 *    = 防重载积分堆积 → 脚抬后"噌"声暴冲
		 * ④ SLEW上升限制：防启动/加速瞬间冲高
		 * ⑤ SLEW下降限制：防足抬起时急减速→颗粒微抖 */
		if ((mot.valtage_cur - speed_param[mot.index].adc_base_curren) > 0) {
			motor.adjust_kiv_voltage = (u16)(((mot.valtage_cur - speed_param[mot.index].adc_base_curren) * motor.kiv) / 10);
			motor.adjust_kiv_voltage = motor.adjust_now_voltage + motor.adjust_kiv_voltage;
		} else
			motor.adjust_kiv_voltage = motor.adjust_now_voltage;

		{
			u16 new_pwm = pid_calc(&mt_pid, motor.adjust_kiv_voltage, (u16)cut_adc, mot.index);

			/* ③ 前馈限幅：uk 限制在 feedforward_pwm ± PID_FF_WINDUP_RANGE
			 * feedforward_pwm = adjust_kiv_voltage × UK_PWM_MAX / (adjust_max_voltage × 8.13)
			 * 纯整数运算，无浮点开销。×813/100 是 ×8.13 的整数近似。
			 * 上下限均钳位至[UK_PWM_MIN, UK_PWM_MAX]。 */
			{
				u16 denom   = (u16)((u32)motor.adjust_max_voltage * 813u / 100u); /* 90×813/100=731 */
				u16 pwm_ff  = (denom > 0u) ? (u16)((u32)motor.adjust_kiv_voltage * UK_PWM_MAX / denom) : (u16)UK_PWM_MAX;
				u16 ff_max  = (pwm_ff + PID_FF_WINDUP_RANGE > UK_PWM_MAX) ? (u16)UK_PWM_MAX : pwm_ff + PID_FF_WINDUP_RANGE;
				u16 ff_min  = (pwm_ff > PID_FF_WINDUP_RANGE + UK_PWM_MIN) ? pwm_ff - PID_FF_WINDUP_RANGE : (u16)UK_PWM_MIN;
				if (new_pwm > ff_max) { new_pwm = ff_max; mt_pid.uk = (float)new_pwm; }
				else if (new_pwm < ff_min) { new_pwm = ff_min; mt_pid.uk = (float)new_pwm; }
			}

			/* ④⑤ SLEW限幅 */
			if (new_pwm > old_pwm + ZHANKONBI_SLEW_UP) {
				new_pwm = old_pwm + ZHANKONBI_SLEW_UP;
				mt_pid.uk = (float)new_pwm;
			} else if (old_pwm > new_pwm && (old_pwm - new_pwm) > ZHANKONBI_SLEW_DOWN) {
				new_pwm = old_pwm - ZHANKONBI_SLEW_DOWN;
				mt_pid.uk = (float)new_pwm;
			}
			ZHANKONBI = new_pwm;
			old_pwm   = new_pwm;
		}
		checkout_current_over();
		break;
	case STATUS_MT_STOP:
		/* PID 跟随减速，不叠加 KIV
		 *
		 * adjust_now_voltage 由 motor_speed() 按 deceleration 步长平滑递减。
		 * 减速阶段故意不使用 KIV：KIV 的设计目的是"维持速度"，减速时若叠加
		 * 会把人踩踏产生的负载电流反向转化为更高电压目标，阻碍减速并产生
		 * "噌"声（电压/电流冲高）。主循环中 motor.kiv 自然衰减至 0 即可。
		 *
		 * 双向斜率钳位：
		 *   上升：最多 +ZHANKONBI_SLEW_UP/ISR——不能完全禁止，否则E06时
		 *         误差正→ZHANKONBI冻结→无法减速
		 *   下降：最多 -ZHANKONBI_SLEW_DOWN/ISR——防暴力制动→大电流→E03 */
		if (cut_adc < 0)
			cut_adc = 0;
		valtage = (u8)(0.126f * (float)cut_adc);
		{
			u16 new_pwm = pid_calc(&mt_pid, motor.adjust_now_voltage, (u16)cut_adc, mot.index);
			if (new_pwm > old_pwm + ZHANKONBI_SLEW_UP) {
				new_pwm = old_pwm + ZHANKONBI_SLEW_UP;
				mt_pid.uk = (float)new_pwm;
			}
			else if (old_pwm > new_pwm && (old_pwm - new_pwm) > ZHANKONBI_SLEW_DOWN) {
				new_pwm = old_pwm - ZHANKONBI_SLEW_DOWN;
				mt_pid.uk = (float)new_pwm;
			}
			ZHANKONBI = new_pwm;
			old_pwm   = new_pwm;
		}
		if ((motor.END_speed_scale >= valtage) || (motor.stop_time_late > 8u)) {
			motor.stop_time_late = 0u;
			motor.status = NULL;
			ctr.status = STATUS_TO_STOP;
			ZHANKONBI = MT_START_PWM;
		}
		checkout_current_over();
		break;
	default:
		break;
	}
	/* 每次ADC采样都执行电压偏差监控 */
	checkout_voltage_runaway(mot);
}

void error_chick(void)
{
	checkout_bus_voltage();
}
