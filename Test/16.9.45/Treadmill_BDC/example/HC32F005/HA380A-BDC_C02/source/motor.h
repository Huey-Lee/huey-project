/* motor.h - motor state, speed steps, error codes. */
#ifndef USER_MOTOR_H_
#define USER_MOTOR_H_
#include "hc32f005.h"
#include "bt.h"

#define VAR_SPEED_CNT             (3)     /* ramp timing when acc/dec */
#define VAR1_SPEED_CNT            (3)     /* same, alternate path */
#define MT_OPEN_ADC               (100)   /* motor-off ADC threshold, ~50Hz task */
#define MT_STOP_ACT_SPEED_SCALE   (4)     /* 4 => 0.2 km/h scale; act when near stop */

#define MT_START_SPEED_SCALE      (20)    /* 20 => 0.2; min run speed before climb */
#define MT_TO_PID_SPEED_SCALE     (120)   /* 120 => 1.2; hand off to closed-loop PID */

/* Before TO_STOP: write this duty level, not 0, then stop timers. Tune 8~30: lower=sharper end, higher=softer. */
#define MT_START_PWM         (25)
#define MT_START_STEP        (3)
#define MT_STOP_SPEED				 (5)

#define speed_step_PID   	 3
#define speed_step_END   	 3

#define GET_SPEED_VOLTAGE(SPEED_SCALE,A,B)  (A*(float)(SPEED_SCALE-100.0)+B)      /* V from speed scale, line fit */

#define CALC_VOLTAGE_2_ADC(VOTAGE)    (((VOTAGE/MT_ADC_AMP)/4.95)*4096) /* bus V to ADC */
// M+ ��ѹ��·��5.1 / (255 + 255+ 5.1) �� 1 / 101; 103.2044��ѹ��·�汶��; 5��Ƭ���ⲿ�ο���Դ
#define CALC_ADC_2_VOLTAGE(ADC)       (103.2044*(5*ADC)/4096) /* sense ADC to bus V */
// ������·��MOS+��Χ���磩�� shunt �źŵĵ�Ч����(MT_CURRENT_ADC_AMP)��Լ 1.1 ����
#define CALC_ADC_2_CURRENT(ADC)       ((((ADC/4096.0)*4.95)/MT_CURRENT_ADC_AMP)/(SAMPLE_RES)) /* to A */

/* Shunt: 220V R=4mOhm, 110V R=8mOhm, gain, maps A to ADC. Example: 15A threshold. */
#define CURRENT_20A0    (548)  //20A
#define CURRENT_15A0    (411)  //15A
#define CURRENT_14A0    (384)
#define CURRENT_13A0    (357)
#define CURRENT_12A0    (330)
#define CURRENT_11A0    (303)
#define CURRENT_10A0    (276)
#define CURRENT_9A0     (252)
#define CURRENT_8A0     (225)
#define CURRENT_7A0     (197)
#define CURRENT_6A0     (170)
#define CURRENT_5A0     (144)
#define CURRENT_4A0     (117)
#define CURRENT_3A0     (90)
#define CURRENT_2A0     (63)
#define CURRENT_1A0     (37)


#define OVER_CURRENT_MAX           			  (CURRENT_20A0)  /* global OC trip level */
#define OVER_CURRENT_MAX_LOW_SPEED  			(CURRENT_6A0)   /* stricter at low speed */
#define OVER_CURRENT_MAX_TEST       			(CURRENT_3A0)

#define OVER_CURRENT_TIMEOUT        			(100)    /* 1ms units */
#define LOW_SPEED_OVER_CURRENT_TIMEOUT    (5000)   /* 1ms units */
#define OVER_SPEED_TIMEOUT          			(200)    /* 10ms units */

#define end_set_val 15
/* Bus OV window (bridge / bus sense) */
#define OVER_VOLTAGE_OFFSET         (80.0)       /* margin */
#define OVER_VOLTAGE_MAX            (240.0)      /* ref: 250V~ADC */
#define OVER_SPEED_TIMEOUT_VOLTAGE  (50)         /* 10ms debounce */

#define MAX_MT_SPEED_SCALE          (120)        /* 12.0 in 0.1 units */
/* E06: volt/ramp/max-scale overshoot; E08: OC + MOS-short tier (~40 V diff vs cmd, same fault code). */
typedef enum _error_code_enum
{
	ERROR_01=1,
	ERROR_02,
	ERROR_03,
	ERROR_04,
	ERROR_05,   	/* E05: over-voltage */
	ERROR_06,     /* E06 */
	ERROR_07,    	/* E07 */
	ERROR_08,     /* E08: over-current */
	ERROR_09,
	ERROR_0A,
	ERROR_0B,
	ERROR_0C,
}error_code_enum;


typedef enum _motor_enum
{
	STATUS_POWERON=1,
	STATUS_RUN,
	STATUS_TO_RUN,
	STATUS_STOP,
	STATUS_TO_STOP,
	STATUS_ERROR,

	STATUS_MT_START,
	STATUS_MT_PID,
	STATUS_MT_STOP,
	STATUS_MT_IMMED_STOP,
}motor_enum;


typedef struct _t_motor
{
  u8 status;
	u8 en;
	u8 index;
  u8 adjust_max_voltage; /* from display param */
  u8 adjust_min_voltage;
	u16 adjust_speed_max;
  u16 adjust_ove_curren; /* over-current setpoint index */
	
  u16 set_speed_scale;
  u16 cur_speed_scale;
	u16 sta_speed_scale;
	u16 END_speed_scale; /* end speed step */
	u16 accelerated;
	u16 deceleration;
  float voltage_param;  /* dV / d(speed) line */
	
	u16 start_tim;
	u16 start_add_val;

  u16 start_set_val;
  u16 start_cur_val;
	
  u8 adjust_set_voltage;
  u8 adjust_cou_voltage;
	u16 adjust_kiv_voltage; /* with kiv integrator */
	u16 adjust_cur_voltage;
	u16 adjust_sta_voltage; /* start ramp V */
	u16 adjust_now_voltage; /* running V target */
	
	u16 stop_time_late;
	float stop_valtage;
	float start_valtage;
	float kiv; /* I-gain from display per km */
	u16 valtage_cur;				/* shunt / current sense ADC */
	u16 valtage_up; /* bus high */
	u16 valtage_down; /* bus low (diff for V) */
	float motor_valtage;
	float now_current;
}motor_t;

typedef struct _t_speed_param
{
	float voltage;
	u16 pwm_max;
	u16 pwm_min;

	u8 kiv; /* per-step kiv (display table) */
	u8 adjust_max_voltage;
	u8 adjust_cont_voltage;
	u8 adc_base_curren;	/* no-load / base line for current in PID */
}speed_param_t;


typedef struct _t_ctr
{
  u8 status;
  u8 error_code;
  u8 ack;
  u8 immed_stop_flag;
  u8 immed_stop_voltage_flag;
}ctr_t; /* app + fault control block */


extern ctr_t ctr;
extern motor_t motor;
extern speed_param_t speed_param[];  

void motor_speed(void);
extern void ctr_proc_loop(void);
extern void ctr_init(void);
extern void stop_motor(void);
extern void set_motor_speed(u8 scale);
/* Soft ramp + latch error/ACK: no hard tim stop (avoids surge when display lost / fault). */
extern void motor_soft_fault_set(u8 err_code);
/* Class-1: immediate power cut (PWM off now). */
extern void motor_hard_fault_set(u8 err_code);
/* Class-2: fast emergency braking profile (safety key / e-stop). */
extern void motor_emergency_brake_set(u8 err_code);
/* Class-3: controlled smooth stop (communication loss, etc.). */
extern void motor_controlled_stop_set(u8 err_code);

#endif /* USER_MOTOR_H_ */
