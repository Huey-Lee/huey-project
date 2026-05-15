/* motor.c - motor FSM, speed, faults. */
#include "motor.h"
#include "user_timer.h"
#include "pid.h" /* UK_PWM_* */
#include "user_adc.h"
#include "uart_frame.h"
extern adc_t adc_handle;
extern u8 waitforamoment;
extern uint16_t ZHANKONBI;
ctr_t ctr;
motor_t motor;

#define FAST_BRAKE_DECEL_LEVEL   (25u)
#define SOFT_STOP_DECEL_LEVEL    (speed_step_END)
#define RUN_FF_SLOPE             (0.084f)
#define RUN_FF_OFFSET            (18.0f)
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
  /* 1.0~1.5 km/h 在 index0~1: KIV 略减可减轻电流补偿阶跃感; 若带载发闷可改回 3 */
  {GET_SPEED_VOLTAGE(100,RUN_FF_SLOPE,RUN_FF_OFFSET),  1000,1000, 2, 30,4,23}, //0--- 18V
  {GET_SPEED_VOLTAGE(100,RUN_FF_SLOPE,RUN_FF_OFFSET),  1000,1000, 2, 30,4,23}, //1--- 18V
  {GET_SPEED_VOLTAGE(200,RUN_FF_SLOPE,RUN_FF_OFFSET),  1000,1000, 3, 30,4,23}, //2
  {GET_SPEED_VOLTAGE(300,RUN_FF_SLOPE,RUN_FF_OFFSET),  1000,1000, 3, 30,4,30}, //3
  {GET_SPEED_VOLTAGE(400,RUN_FF_SLOPE,RUN_FF_OFFSET),  1000,1000, 3, 25,5,30}, //4
  {GET_SPEED_VOLTAGE(500,RUN_FF_SLOPE,RUN_FF_OFFSET),  1000,1000, 3, 25,6,30}, //5
  {GET_SPEED_VOLTAGE(600,RUN_FF_SLOPE,RUN_FF_OFFSET),  1000,1000, 2, 25,7,30}, //6
  {GET_SPEED_VOLTAGE(700,RUN_FF_SLOPE,RUN_FF_OFFSET),  1000,1000, 2, 20,7,40}, //7
  {GET_SPEED_VOLTAGE(800,RUN_FF_SLOPE,RUN_FF_OFFSET),  1000,1000, 2, 20,7,40}, //8
  {GET_SPEED_VOLTAGE(900,RUN_FF_SLOPE,RUN_FF_OFFSET),  1000,1000, 2, 20,3,30}, //9
  {GET_SPEED_VOLTAGE(1000,RUN_FF_SLOPE,RUN_FF_OFFSET), 1000,1000, 2, 20,8,33}, //10
  {GET_SPEED_VOLTAGE(1100,RUN_FF_SLOPE,RUN_FF_OFFSET), 1000,1000, 2, 20,8,35}, //11
	{GET_SPEED_VOLTAGE(1200,RUN_FF_SLOPE,RUN_FF_OFFSET), 1000,1000, 2, 20,8,35}, //12
	{GET_SPEED_VOLTAGE(1300,RUN_FF_SLOPE,RUN_FF_OFFSET), 1000,1000, 2, 20,8,35}, //13
	{GET_SPEED_VOLTAGE(1400,RUN_FF_SLOPE,RUN_FF_OFFSET), 1000,1000, 2, 20,8,35}, //14
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
	motor.voltage_param = RUN_FF_SLOPE;

	motor.cfg_ramp_start_volt = 7u;
	motor.cfg_soft_start_ms = 5000u;
	motor.cfg_soft_stop_ms = 0u;
	motor.cfg_stop_ramp_div = 24u;
	motor.cfg_stop_floor_pwm = 0u;
	motor_recalc_start_ramp_interval();
	motor_recalc_stop_ramp_div();
}

#define MOTOR_ISR_HZ_ASSUMED  1600u

void motor_recalc_start_ramp_interval(void)
{
	float v0 = (float)motor.cfg_ramp_start_volt;
	float v1 = (float)motor.adjust_min_voltage;
	float steps_f;
	u32 steps;
	u16 ms;
	u32 ticks_total;
	u32 interval;

	if (v1 < (v0 + 0.1f)) {
		v0 = v1 - 0.1f;
		if (v0 < 5.0f) {
			v0 = 5.0f;
		}
		motor.cfg_ramp_start_volt = (u8)v0;
	}
	steps_f = (v1 - v0) / 0.1f;
	steps = (u32)steps_f;
	if (steps < 1u) {
		steps = 1u;
	}
	ms = motor.cfg_soft_start_ms;
	if (ms == 0u) {
		ms = 5000u;
	}
	ticks_total = ((u32)ms * MOTOR_ISR_HZ_ASSUMED) / 1000u;
	if (ticks_total < 1u) {
		ticks_total = 1u;
	}
	interval = ticks_total / steps;
	if (interval < 1u) {
		interval = 1u;
	}
	if (interval > 65535u) {
		interval = 65535u;
	}
	motor.cfg_start_ramp_interval_ticks = (u16)interval;
}

void motor_recalc_stop_ramp_div(void)
{
	u16 ms = motor.cfg_soft_stop_ms;
	u32 span;
	u32 ticks;
	u32 div;
	u16 floor_pwm;

	if (ms == 0u) {
		motor.cfg_stop_ramp_div = 24u;
		return;
	}
	floor_pwm = motor.cfg_stop_floor_pwm;
	if (floor_pwm == 0u) {
		floor_pwm = MOTOR_STOP_PWM_FLOOR_DEFAULT;
	}
	span = (u32)UK_PWM_MAX - (u32)floor_pwm;
	if (span < 1u) {
		span = 1u;
	}
	ticks = ((u32)ms * MOTOR_ISR_HZ_ASSUMED) / 1000u;
	div = ticks / span;
	if (div < 1u) {
		div = 1u;
	}
	if (div > 255u) {
		div = 255u;
	}
	motor.cfg_stop_ramp_div = (u8)div;
}

u16 motor_drive_pwm_from_volt_u8(u8 volts_v)
{
	u16 adc_span;
	u32 pwm;
	u16 target_adc;

	if (volts_v < 2u) {
		volts_v = 2u;
	}
	if (volts_v > motor.adjust_max_voltage) {
		volts_v = motor.adjust_max_voltage;
	}
	target_adc = (u16)((float)volts_v * 8.13f);
	adc_span = (u16)((float)motor.adjust_max_voltage * 8.13f);
	if (adc_span < (u16)(24.0f * 8.13f)) {
		adc_span = (u16)(90.0f * 8.13f);
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

/* Set error + soft decel: 保持当前 PWM，切入 STOP 斜坡 */
void motor_soft_fault_set(u8 err_code)
{
	if (err_code) {
		ctr.error_code = err_code;
		ctr.ack        = 1u;
	}
	if (motor.status == STATUS_MT_PID || motor.status == STATUS_MT_START || motor.status == STATUS_MT_STOP) {
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

void ctr_proc_loop(void)
{
	static u8 interrupt_flag=0,er02_code=0,er08_code=0;
//	uint16_t cut_adc;
	uint16_t val_up,val_down;
	/*
	 * Power-on: sample ADC, check bus / bridge. Relay stuck can skew PWM vs sense;
	 * this path flags E02 (unbalanced bus) and E05 (abnormal bus) before run.
	 */
	switch(ctr.status)
	{
	case STATUS_POWERON:
		user_adc_init();
		for(u8 i=0;i<100;i++)
		{
			if(adc_handle.irq_flag==0 && interrupt_flag==0)
			{
				Adc_Start();
			}
			if(interrupt_flag!=adc_handle.irq_flag)/* ADC half-cycle */
			{
				if(adc_handle.irq_flag>0)						 /* conv done, restart */
				{
					interrupt_flag =adc_handle.irq_flag;
					Adc_Start();
				}
				if(adc_handle.irq_flag == 0 && interrupt_flag > 0)/* use sample */
				{
					interrupt_flag=0;
					val_up = CALC_ADC_2_VOLTAGE(motor.valtage_up);
					val_down = CALC_ADC_2_VOLTAGE(motor.valtage_down);
					if((val_up>(val_down + 70)) || (val_down > (val_up + 70)))
					{
						er02_code++;
					}
					if(val_up<10 && val_down < 10)
					{
						er08_code++;
					}
				}
			}
		}
		if(er02_code > 5)
		{
			er02_code = 0;
      ctr.error_code = ERROR_02;
      ctr.status = STATUS_ERROR;
		}
		else if(er08_code > 5)
		{
			er08_code = 0;
      ctr.error_code = ERROR_05;		/* bus abnormal (low both rails) */
      ctr.status = STATUS_ERROR;
		}
		else
		ctr.status = NULL;
	/* end STATUS_POWERON */
	break;
	case STATUS_RUN:
		user_adc_init();
		for(u8 i=0;i<100;i++)
		{
			if(adc_handle.irq_flag==0 && interrupt_flag==0)
			{
				Adc_Start();
			}
			if(interrupt_flag!=adc_handle.irq_flag)
			{
				if(adc_handle.irq_flag>0)
				{
					interrupt_flag =adc_handle.irq_flag;
					Adc_Start();
				}
				if(adc_handle.irq_flag == 0 && interrupt_flag > 0)
				{
					interrupt_flag=0;
					val_up = CALC_ADC_2_VOLTAGE(motor.valtage_up);
					val_down = CALC_ADC_2_VOLTAGE(motor.valtage_down);
					if((val_up>(val_down + 70)) || (val_down > (val_up + 70)))
					{
						er02_code++;
					}
					if(val_up<10 && val_down < 10)
					{
						er08_code++;
					}
				}
			}
		}
		if(er02_code > 5)
		{
			er02_code = 0;
      ctr.error_code=ERROR_02;								
      ctr.status=STATUS_ERROR;
		}
		else if(er08_code > 5)
		{
			er08_code = 0;
      ctr.error_code=ERROR_08;							//
      ctr.status=STATUS_ERROR;
		}
		else
		{
			ctr.status = NULL;
			waitforamoment = 1;
		}
	break;
	case STATUS_TO_RUN:
		motor.adjust_set_voltage = speed_param[0].voltage;
		/* 与下控起步导引一致：7V 起，不再用上位机 STA_LEVEL 的 start_valtage（否则会出现 4.9V/29V 档位跳变） */
		motor.adjust_now_voltage = (u16)(7.0f * 8.13f);
		ZHANKONBI = MT_START_PWM;
		if(M0P_ADTIM6->GCONR_f.START == 0 || M0P_TIM0->CR_f.CTEN == 0)
		{
			motor.status = STATUS_MT_START;
			clear_adcbuf();
			motor.cur_speed_scale = motor.sta_speed_scale;
			motor.adjust_sta_voltage = (u16)(7.0f * 8.13f);
			motor.start_tim = 0;
			tim0run();
			delay1ms(500);
			tim6run();
		}

		
		ctr.status = NULL;
	break;
	case STATUS_STOP:
		motor.status=STATUS_MT_STOP;
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
        /* 统一分级：
         * Class-1: E08/E02/E03/E05 -> 立即切断PWM
         * Class-2: E07 -> 快速紧急制动（快速减速）
         * Class-3: E01 -> 受控平滑减速
         */
        switch (ctr.error_code)
        {
            case ERROR_08:
            case ERROR_02:
            case ERROR_03:
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
            case ERROR_01:
                /* 通讯丢失：与批量版一致，减停到MT_STOP_SPEED后进入TO_STOP */
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
		val = (u16)GET_SPEED_VOLTAGE(motor.cur_speed_scale, RUN_FF_SLOPE, RUN_FF_OFFSET);
		if (val < motor.adjust_min_voltage) {
			val = motor.adjust_min_voltage;
		}
		if (val > motor.adjust_max_voltage) {
			val = motor.adjust_max_voltage;
		}
		motor.adjust_now_voltage = (u16)(val * 8.13f);
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
		val = (u16)GET_SPEED_VOLTAGE(motor.cur_speed_scale, RUN_FF_SLOPE, RUN_FF_OFFSET);
		if (val < motor.adjust_min_voltage) {
			val = motor.adjust_min_voltage;
		}
		if (val > motor.adjust_max_voltage) {
			val = motor.adjust_max_voltage;
		}
		motor.adjust_now_voltage = (u16)(val * 8.13f);
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
