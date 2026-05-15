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

/* 占空比上升斜率限制：每次 ADC ISR（≈0.625ms）最多上升 1 个单位
 * kp已升至0.25，3ADC误差→Δu=0.75，积分累计2拍即可产生1单位修正，不需要SLEW=2
 * SLEW=2时每次跳0.18V，产生颗粒感；SLEW=1平滑 */
#define ZHANKONBI_SLEW_UP    (1u)

/* 占空比下降斜率限制：STOP 态纯开环减速时每 ISR 最多下降 3 个单位（≈4.8/ms）
 * 1000→0 约需 208ms 最快，实际由 adjust_now_voltage 自然缓降决定速度
 * 仅作保护：防止 adjust_now_voltage 突降（如速度级突变）造成脉冲制动 */
#define ZHANKONBI_SLEW_DOWN  (3u)

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

/* E02桥臂不对称检测（直接ADC整数比较，无浮点）
 * 换算：1V ≈ 7.937 ADC（4096/516，与cut_adc同量纲）
 *   70V → ADC ≈ 556   上电/停机态桥臂不对称门限
 *   10V → ADC ≈  79   双路母线均低门限（上电前静态检查）
 * 连续检测门限：停机态 ISR 中 3 次连续（约1.5ms）去抖 */
#define BRIDGE_ASYM_ADC     (556u)   /* 桥臂不对称：|up-down|>70V */
#define BUS_BOTH_LOW_ADC    (79u)    /* 双路均低：up和down均<10V */
#define BRIDGE_ASYM_CNT     (3u)     /* 停机持续监测去抖次数 */

/* 母线过/欠压保护参数 */
#define OVER_VOLTAGE_OFFSET         (80.0)
#define OVER_SPEED_TIMEOUT_VOLTAGE  (50)

/* 过压（E05）：阈值 = 上控CMD_VOLTAGE_MAX下发最大电压×130%
 *   由uart_frame.c在CMD_VOLTAGE_MAX处更新over_voltage_adc
 *   默认值：90V×130%≈117V，ADC≈928（motor_speed_pid.c初始化）
 * 跳闸门限：带消退（cnt--）持续超压 BUS_OV_TRIP_COUNT 次触发 */
#define BUS_OV_TRIP_COUNT           (12u)        /* 过压跳闸门限（带消退，约1s持续） */

/* 欠压保护参数（双通道）
 *
 * 时基说明：error_chick() 在主循环每 2000 PWM周期（≈90ms）调用一次，
 *   故"N次"对应的实际时间 = N × 90ms。
 *
 * 慢速通道（非饱和，通用）：实测母线 < 前馈目标×80%，连续10次（≈900ms）触发
 *   适用：高阻抗软电源在高速时的累积欠压。
 *   低速（scale<200≈2km/h）非饱和时跳过，避免轻踩皮带误报。
 *
 * 快速通道（PID饱和）：PWM已打满(ZHANKONBI≥UK_PWM_MAX-2)时，
 *   实测母线 < 前馈目标×94%，连续10次（≈900ms）触发
 *   物理含义：PWM占空100%仍追不上目标，说明供电被硬性限压
 *   count 从5→10：KIV=0时空载补偿缺失，任何大载荷都会饱和，
 *   5次（450ms）太快误报；10次（900ms）给PID足够时间恢复。
 *
 *   低速（scale<300≈3km/h）饱和也跳过快速通道：
 *   低速back-EMF极小，重踩极易饱和，属正常物理现象，
 *   由慢速通道900ms兜底。
 *
 * 单位说明：adjust_now_voltage 以 V×8.13 为单位，cut_adc 以 V×7.937
 *   同电压时 cut_adc/adjust_now_voltage = 97.6%，故94%留有3.6%裕量
 *   正常满速（90V供电）：cut_adc/adj = 97.5% > 94% → 不误报 */
#define BUS_UV_PCT                  (80u)        /* 慢速欠压触发比例(%) */
#define BUS_UV_TRIP_COUNT           (10u)        /* 慢速门限计数（≈900ms） */
#define BUS_UV_SAT_PCT              (94u)        /* 饱和快速欠压触发比例(%) */
#define BUS_UV_SAT_COUNT            (10u)        /* 饱和快速门限计数（5→10，≈900ms，防KIV=0误报） */
#define BUS_UV_SAT_LOW_SPD_SCALE    (300u)       /* 快速通道低速排除门限（≈3km/h） */

#define MAX_MT_SPEED_SCALE          (120)        /* 12.0 in 0.1 units */

/* E02/E08 电压检测阈值（cut_adc量纲，1V≈7.937ADC）
 *
 * 停机/运行前（E02）：cut_adc > 15V → 硬件结构异常（断线/断臂/继电器粘连）
 *
 * 运行态（E08）：实测超 PID 实际目标（含KIV补偿）+30V → 反时限累积跳闸
 *   START态：以 adjust_sta_voltage（单位同cut_adc）为基准
 *   PID  态：以 adjust_kiv_voltage×977/1000 为基准（KIV已补偿）
 *   STOP 态：不做E08，减速时back-EMF自然高于下降目标，由E05绝对过压兜底
 *
 * 反时限触发时间（ADC中断≈0.625ms/次）：
 *   超目标+30V → ≈1.0ms   (excess=238, acc=476; 476/238≈2次)
 *   超目标+60V → ≈0.5ms */
#define VOLT_STOP_MAX_ADC      (119u)  /* 停机态上限：>15V异常（15×7.937≈119） */
#define VOLT_RUN_OVER_ADC      (318u)  /* 运行偏差门限：超目标+40V（40×7.937≈318） */
#define VOLT_RUNAWAY_TRIP_ACC  (2u)    /* 连续超限次数门限：2次×0.625ms≈1.25ms≤1ms响应 */

/* E01通信, E02桥臂开路, E03过流, E04继电器, E05过压, E06超速, E07急停, E08电压偏差 */
typedef enum _error_code_enum
{
	ERROR_01=1,
	ERROR_02,
	ERROR_03,     /* E03: 过流 */
	ERROR_04,
	ERROR_05,   	/* E05: 过压 */
	ERROR_06,     /* E06 */
	ERROR_07,    	/* E07: 急停 */
	ERROR_08,     /* E08: 电压偏差/爆冲 */
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
