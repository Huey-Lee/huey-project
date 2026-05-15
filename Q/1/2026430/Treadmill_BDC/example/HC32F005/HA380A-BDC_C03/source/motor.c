/* motor.c - motor FSM, speed, faults. */
#include "motor.h"
#include "user_timer.h"
#include "pid.h"
#include "user_adc.h"
#include "uart_frame.h"
extern adc_t adc_handle;
extern pid_t mt_pid;
extern u8 waitforamoment;
extern uint16_t ZHANKONBI;
ctr_t ctr;
motor_t motor;

#define FAST_BRAKE_DECEL_LEVEL   (25u)
#define SOFT_STOP_DECEL_LEVEL    (speed_step_END)
speed_param_t  speed_param[15]=
{
	/* per-step row: (line voltage, pwm caps, kiv, V limits, no-load I ADC) */
  //	float voltage;
  //	u16 pwm_max;
  //	u16 pwm_min;
  //	float kiv;
  //	u8 adjust_max_voltage;
  //	u8 adjust_cont_voltage;
  //	float load_base_curren;
  /* index0~1 (1km/h): adc_base_curren 23→18（≈0.5A）
   * 原值23≈0.62A 高于人轻踩时的实际电流增量，导致 KIV 始终不激活。
   * 降至18≈0.5A：电流超过半安即触发KIV，配合 kiv=6 补偿掉速。
   * 若空载跑电压偏高（KIV空载误激活），可回调至 20~22。 */
  /* adc_base_curren = 空载基准电流（ADC）：
   *   1km/h(0,1)：18≈0.5A，原23太高KIV不激活→改低
   *   2km/h(2)及以上：恢复原厂校准值，空载不超基线→KIV不误激活→无脚麻
   *   9km/h(9)：30→40（修正笔误，与7/8km/h一致） */
  {GET_SPEED_VOLTAGE(100,0.14,18),  1000,1000, 2, 30,4,18}, //0--- 18V  base≈0.5A（优化）
  {GET_SPEED_VOLTAGE(100,0.14,18),  1000,1000, 2, 30,4,18}, //1--- 18V  base≈0.5A（优化）
  {GET_SPEED_VOLTAGE(200,0.14,18),  1000,1000, 3, 30,4,23}, //2--- 32V  base原值
  {GET_SPEED_VOLTAGE(300,0.14,18),  1000,1000, 3, 30,4,30}, //3--- 46V  base原值
  {GET_SPEED_VOLTAGE(400,0.14,18),  1000,1000, 3, 25,5,30}, //4--- 60V  base原值
  {GET_SPEED_VOLTAGE(500,0.14,18),  1000,1000, 3, 25,6,30}, //5--- 74V  base原值
  {GET_SPEED_VOLTAGE(600,0.14,18),  1000,1000, 2, 25,7,30}, //6--- 88V  base原值
  {GET_SPEED_VOLTAGE(700,0.14,18),  1000,1000, 2, 20,7,40}, //7--- 102V base原值
  {GET_SPEED_VOLTAGE(800,0.14,18),  1000,1000, 2, 20,7,40}, //8--- 116V base原值
  {GET_SPEED_VOLTAGE(900,0.14,18),  1000,1000, 2, 20,8,40}, //9--- 130V base修正30→40
  {GET_SPEED_VOLTAGE(1000,0.14,18), 1000,1000, 2, 20,8,33}, //10---144V base原值
  {GET_SPEED_VOLTAGE(1100,0.14,18), 1000,1000, 2, 20,8,35}, //11---158V
	{GET_SPEED_VOLTAGE(1200,0.14,18), 1000,1000, 2, 20,8,35}, //12---172V
	{GET_SPEED_VOLTAGE(1300,0.14,18), 1000,1000, 2, 20,8,35}, //13---186V
	{GET_SPEED_VOLTAGE(1400,0.14,18), 1000,1000, 2, 20,8,35}, //14---200V
};
void ctr_init(void)
{
	ctr.status = STATUS_POWERON;
	
	motor.stop_time_late = 0;
	motor.start_valtage = 10;	// 10
	motor.adjust_sta_voltage = 10;
	motor.start_add_val = 5;	//1
	motor.sta_speed_scale = 50;
	motor.END_speed_scale = end_set_val;
	motor.accelerated = speed_step_PID;
	motor.deceleration = speed_step_END;
	motor.adjust_ove_curren = 400;
	motor.adjust_max_voltage = 90;
	motor.adjust_min_voltage = 20;
	motor.adjust_speed_max   = 1200;
	motor.voltage_param = ((motor.adjust_max_voltage - motor.adjust_min_voltage)*1.0)/(motor.adjust_speed_max - 100);//0.1463636363636364
}

/* Set error + soft decel: clear PID history but keep current PWM to avoid a duty step ("surge"). */
void motor_soft_fault_set(u8 err_code)
{
	if (err_code) {
		ctr.error_code = err_code;
		ctr.ack        = 1u;
	}
	if (motor.status == STATUS_MT_PID || motor.status == STATUS_MT_START || motor.status == STATUS_MT_STOP) {
		u16 p = (u16)ZHANKONBI;
		if (p > UK_PWM_MAX) p = UK_PWM_MAX;
		if (p < UK_PWM_MIN) p = UK_PWM_MIN;
		pid_clear_integral_keep_output(&mt_pid, p);
		motor.set_speed_scale = MT_STOP_SPEED;
		motor.status = STATUS_MT_STOP;
	}
}

static void motor_immediate_pwm_cutoff(void)
{
    tim0stop();
    tim6stop();
//    ZHANKONBI = 0;
		ctr.status = NULL;
    motor.status = NULL;
    motor.en = 0;
    waitforamoment = 0;
}

void motor_hard_fault_set(u8 err_code)
{
    if (err_code != 0u) {
        ctr.error_code = err_code;
    }
    ctr.ack = 1u;
    motor_immediate_pwm_cutoff();
    ctr.status = STATUS_ERROR;
}

void motor_emergency_brake_set(u8 err_code)
{
    if (err_code != 0u) {
        ctr.error_code = err_code;
    }
    ctr.ack = 1u;
    if (motor.deceleration < FAST_BRAKE_DECEL_LEVEL) {
        motor.deceleration = FAST_BRAKE_DECEL_LEVEL;
    }
    motor.set_speed_scale = 0;
    motor.status = STATUS_MT_STOP;
    ctr.status = STATUS_ERROR;
}

void motor_controlled_stop_set(u8 err_code)
{
    if (err_code != 0u) {
        ctr.error_code = err_code;
    }
    ctr.ack = 1u;
    motor.deceleration = SOFT_STOP_DECEL_LEVEL;
    motor_soft_fault_set(0u);
    /* 对齐批量版：通信丢失按最小安全速度减停，避免u16速度下溢 */
    motor.set_speed_scale = MT_STOP_SPEED;
    motor.en = 0;
    waitforamoment = 0;
    ctr.status = STATUS_ERROR;
}

/* 桥臂静态检测：上电/启动前，PWM未开，采样100次
 * 返回非0 = 硬件结构异常（调用方统一报 E02）
 *
 * 判据1：|up - down| > 70V（ADC≈556）超5次
 *   桥臂开路/电机线断：一侧浮至母线，另一侧为0，差值大
 *
 * 判据2：cut_adc(up-down) > 15V（ADC≈119）超5次
 *   正常接线时绕组将两端拉齐，差值≈0；断线时悬空端残压 > 15V
 *   兼容低压（<70V）供电时单线断路的检测
 *
 * 判据3：up < 10V 且 down < 10V 超5次
 *   双路均低：母线未建立或两路均断路 → E02 */
static u8 check_bridge_static(void)
{
	u8  flag = 0, er02 = 0, res = 0, low = 0;
	u16 cut;
	for (u8 i = 0; i < 100u; i++) {
		if (adc_handle.irq_flag == 0 && flag == 0)
			Adc_Start();
		if (flag != adc_handle.irq_flag) {
			if (adc_handle.irq_flag > 0) {
				flag = adc_handle.irq_flag;
				Adc_Start();
			}
			if (adc_handle.irq_flag == 0 && flag > 0) {
				flag = 0;
				/* 判据1：桥臂严重不对称（>70V） */
				if (motor.valtage_up > motor.valtage_down + BRIDGE_ASYM_ADC ||
				    motor.valtage_down > motor.valtage_up + BRIDGE_ASYM_ADC)
					er02++;
				/* 判据2：任何残压（>15V），兼容低压断线场景 */
				cut = (motor.valtage_up > motor.valtage_down) ?
				      (motor.valtage_up - motor.valtage_down) : 0u;
				if (cut > VOLT_STOP_MAX_ADC)
					res++;
				/* 判据3：双路均低 */
				if (motor.valtage_up < BUS_BOTH_LOW_ADC &&
				    motor.valtage_down < BUS_BOTH_LOW_ADC)
					low++;
			}
		}
	}
	if (er02 > 5u) return (u8)ERROR_02;
	if (res  > 5u) return (u8)ERROR_02;
	if (low  > 5u) return (u8)ERROR_02;
	return 0u;
}

void ctr_proc_loop(void)
{
	u8 bridge_result;
	/*
	 * 上电/启动前：采样桥臂静态状态，任何异常均属硬件结构故障 → E02
	 * 包含：桥臂不对称（|up-down|>70V）和双路均低（<10V）两种判据
	 */
	switch(ctr.status)
	{
	case STATUS_POWERON:
		user_adc_init();
		bridge_result = check_bridge_static();
		if (bridge_result != 0u) {
			ctr.error_code = ERROR_02;   /* 不对称或双路均低，均属硬件结构异常 */
			ctr.status = STATUS_ERROR;
		} else {
			ctr.status = NULL;
		}
	/* end STATUS_POWERON */
	break;
	case STATUS_RUN:
		user_adc_init();
		bridge_result = check_bridge_static();
		if (bridge_result != 0u) {
			ctr.error_code = ERROR_02;   /* 不对称或双路均低，均属硬件结构异常 */
			ctr.status = STATUS_ERROR;
		} else {
			ctr.status = NULL;
			waitforamoment = 1;
		}
	break;
	case STATUS_TO_RUN:
		motor.adjust_set_voltage = speed_param[0].voltage;
		/* 状态锁：仅在定时器完全停止时才重置 PWM 和 PID
		 * 若定时器正在运行（电机仍在转），保持当前 ZHANKONBI 不变，
		 * 防止高速运行中误触 STATUS_TO_RUN 导致 PWM 突降或 PID 积分清零 */
		if(M0P_ADTIM6->GCONR_f.START == 0 || M0P_TIM0->CR_f.CTEN == 0)
		{
			motor.adjust_now_voltage = 13;
			ZHANKONBI = MT_START_PWM;
			motor.status = STATUS_MT_START;
			default_pid_int(&mt_pid);
			clear_adcbuf();
			motor.cur_speed_scale = motor.sta_speed_scale;
			motor.adjust_sta_voltage = motor.start_valtage;
			motor.start_tim = 0;
			tim0run();
			delay1ms(500);
			tim6run();
		}
		ctr.status = NULL;
	break;
	case STATUS_STOP:
		/* 无扰停机（STATUS_MT_STOP ISR 不使用 KIV）
		 *
		 * 原逻辑减去 kiv_est：是为"STOP有KIV"设计的，
		 * 现在 STOP ISR 已删除 KIV，目标直接用 adjust_now_voltage。
		 * 若仍减 kiv_est → 初始目标 < 实测 → PID立即扣减PWM → 抽动。
		 *
		 * 正确：adjust_now_voltage = cut_now（零误差切入）
		 * stop_valtage 反推：使 motor_speed() 首拍 STOP 输出 = cut_now → 连续。
		 *
		 * 顺序：先写目标，再切状态（ISR线程安全，无竞争窗口）。 */
		{
			u16 cut_now = (motor.valtage_up > motor.valtage_down) ?
			              (motor.valtage_up - motor.valtage_down) : 0u;
			motor.adjust_now_voltage = cut_now;
			{
				u16 next_scale = (motor.cur_speed_scale > motor.deceleration) ?
				                 (motor.cur_speed_scale - motor.deceleration) : 0u;
				float sv = (float)cut_now / 8.13f
				           - motor.voltage_param * ((float)(int16_t)next_scale - 100.0f);
				if (sv < 2.0f)  sv = 2.0f;
				if (sv > (float)motor.adjust_max_voltage) sv = (float)motor.adjust_max_voltage;
				motor.stop_valtage = sv;
			}
		}
		motor.status = STATUS_MT_STOP;
		ctr.status = NULL;
	break;
	case STATUS_TO_STOP:
		tim0stop();
		tim6stop();
//				printf("ctr:%d\r\n",ctr.status);
		uart_frame_tx(CMD_STOP_OVER,1);
		ctr.status = NULL;
	break;
	case STATUS_ERROR:
        /* 故障分级：
         * 1级：E02/E03/E05/E08 → 立即切断PWM
         * 2级：E07 → 快速减速制动
         * 3级：E01/E06 → 受控平滑减停（保持当前PWM，按deceleration逐步减速）
         */
        switch (ctr.error_code)
        {
            case ERROR_08:   /* 电压偏差/爆冲 */
            case ERROR_02:
            case ERROR_03:   /* 过流 */
            case ERROR_05:
                motor_immediate_pwm_cutoff();
                break;
            case ERROR_07:
                if (motor.deceleration < FAST_BRAKE_DECEL_LEVEL) {
                    motor.deceleration = FAST_BRAKE_DECEL_LEVEL;
                }
                motor.set_speed_scale = 0;
                motor.status = STATUS_MT_STOP;
                break;
            case ERROR_06:   /* 欠压：同E01，缓慢受控减停，不硬切PWM */
            case ERROR_01:   /* 通讯丢失：受控减停到MT_STOP_SPEED后进入TO_STOP */
                {
                    u16 p = (u16)ZHANKONBI;
                    if (p > UK_PWM_MAX) p = UK_PWM_MAX;
                    if (p < UK_PWM_MIN) p = UK_PWM_MIN;
                    pid_clear_integral_keep_output(&mt_pid, p);
                }
                motor.deceleration = SOFT_STOP_DECEL_LEVEL;
                motor.set_speed_scale = MT_STOP_SPEED;
                motor.status = STATUS_MT_STOP;
                break;
            default:
                motor_soft_fault_set(0u);
                break;
        }
        if (ctr.error_code)
            uart_frame_tx(CMD_ERROR, ctr.error_code);
        ctr.status = NULL;
        break;
	default:
	break;
	}
}
void motor_speed(void)
{
	u16 val;
	int16_t cut_adc;
	u16 now_valtage=0;
	switch(motor.status)
	{
		case(STATUS_MT_START):
		
		
		if(motor.cur_speed_scale < motor.set_speed_scale)
		{
			motor.cur_speed_scale+=motor.accelerated;
		}
		if(motor.cur_speed_scale > motor.set_speed_scale)
		{
			motor.cur_speed_scale-=motor.accelerated;
		}
		val = (u16)GET_SPEED_VOLTAGE(motor.cur_speed_scale,motor.voltage_param,motor.stop_valtage);
		motor.adjust_now_voltage = (u16)(val*8.13);
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
		cut_adc =	motor.valtage_up - motor.valtage_down;
		if(cut_adc<0)
			cut_adc=0;
		now_valtage = (u8)(0.126*cut_adc);

		motor.adjust_sta_voltage += motor.start_add_val;
		/* 上限1：绝对上限 = 额定最大电压（V×8.13量纲）*/
		{
			u16 sta_max = (u16)((u32)motor.adjust_max_voltage * 813u / 100u);
			if (motor.adjust_sta_voltage > sta_max)
				motor.adjust_sta_voltage = sta_max;
		}
		/* 上限2：与PID目标精确对齐（零误差切换）
		 * adjust_sta_voltage 不超过 adjust_now_voltage（= val×8.13）。
		 *
		 * 原理：PID切入后目标 = adjust_now_voltage = val×8.13；
		 *   START阶段ISR目标也限制为 val×8.13，电机稳定在同一工作点；
		 *   切换时PID误差 = adjust_now_voltage - cut_adc = 0，完全无顿挫。
		 *
		 * 过渡条件：电机运行在 val×8.13 对应的实际电压（val×1.024V），
		 *   now_valtage = val×1.024 > val，切换条件自然满足。 */
		if (motor.adjust_sta_voltage > motor.adjust_now_voltage)
			motor.adjust_sta_voltage = motor.adjust_now_voltage;

		motor.start_tim++;
		if(motor.start_tim > 15 && val  < now_valtage )
		{
			motor.start_tim=0;
			motor.adjust_sta_voltage = motor.start_valtage;
			motor.kiv = 0.0f;   /* KIV 从0平滑爬升，避免切入瞬间目标跳变 */
			motor.status = STATUS_MT_PID;
			motor.start_set_val=0;
		}
		break;
		case(STATUS_MT_PID):
		if(motor.cur_speed_scale < motor.set_speed_scale)
		{
			motor.cur_speed_scale+=motor.accelerated;
		}
		if(motor.cur_speed_scale > motor.set_speed_scale)
		{
			motor.cur_speed_scale-=motor.accelerated;
		}
		val = (u16)GET_SPEED_VOLTAGE(motor.cur_speed_scale,motor.voltage_param,motor.stop_valtage);
		motor.adjust_now_voltage = (u16)(val*8.13);
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
		if(motor.kiv>speed_param[motor.index].kiv)
			motor.kiv = motor.kiv-0.2;
		else if(motor.kiv<speed_param[motor.index].kiv)
			motor.kiv = motor.kiv+0.2;
/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/
		break;
		case(STATUS_MT_STOP):
		if(motor.cur_speed_scale < motor.set_speed_scale)
		{
			uint16_t diff_up = (uint16_t)(motor.set_speed_scale - motor.cur_speed_scale);
			if(motor.deceleration >= diff_up)
			{
				motor.cur_speed_scale = motor.set_speed_scale;
			}
			else
			{
				motor.cur_speed_scale += motor.deceleration;
			}
		}
		else if(motor.cur_speed_scale > motor.set_speed_scale)
		{
			uint16_t diff_down = (uint16_t)(motor.cur_speed_scale - motor.set_speed_scale);
			if(motor.deceleration >= diff_down)
			{
				motor.cur_speed_scale = motor.set_speed_scale;
			}
			else
			{
				motor.cur_speed_scale -= motor.deceleration;
			}
		}
		
		if(motor.cur_speed_scale<MT_STOP_SPEED)
		{
			motor.stop_time_late++;
		}
		
		if(motor.stop_valtage>2)
			motor.stop_valtage = motor.stop_valtage - 0.4;//0.5
		else
			motor.stop_valtage=2;
		
		val = (u16)GET_SPEED_VOLTAGE(motor.cur_speed_scale,motor.voltage_param,motor.stop_valtage);
		motor.adjust_now_voltage = (u16)(val*8.13);
			
		if(motor.kiv>0.1)
			motor.kiv = motor.kiv-0.1;
		else
			motor.kiv = 0.1;
		break;
	}
	motor.index = (u8)(motor.cur_speed_scale / 100);
	if(motor.index > 14u)
	{
		motor.index = 14u;
	}

}
