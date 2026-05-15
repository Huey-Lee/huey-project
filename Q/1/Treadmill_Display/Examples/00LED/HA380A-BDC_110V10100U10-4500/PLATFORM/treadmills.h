/*
 * treadmills.h  -  Single configuration file for the treadmill display board
 *
 * THIS IS THE ONLY FILE YOU NEED TO MODIFY.
 * All display behaviour and all parameters pushed to the motor controller
 * are defined here.  After editing, rebuild and flash:
 *   - Display board only  : for most parameters
 *   - BOTH boards         : when TM_UART_BAUD is changed
 *
 * Speed unit throughout: 0.1 km/h integer steps.
 *   TM_SPEED_MIN = 10  ->  1.0 km/h
 *   TM_SPEED_MAX = 80  ->  8.0 km/h
 * Wire encoding: display sends raw byte; motor controller computes
 *   speed_scale = byte × 10   (scale 100 = 1.0 km/h, scale 800 = 8.0 km/h)
 */

#ifndef __TREADMILLS_H
#define __TREADMILLS_H

#include "main.h"


/*=============================================================================
 * [1] Firmware version
 *     Boot screen shows "U x.x" then "C x.x" when BOOT_SHOW_VERSION is set.
 *===========================================================================*/
/* #define BOOT_SHOW_VERSION */
#define FW_VER_U    10u   /* upper board version displayed as "U 1.0" */
#define FW_VER_C    0u    /* fallback lower version when no CMD_ACK received */


/*=============================================================================
 * [2] Communication
 *     [BOTH BOARDS] TM_UART_BAUD must match the lower board UART baud
 *     (e.g. HC-32_1 init). Reflash both boards if changed.
 *===========================================================================*/
#define TM_UART_BAUD                2400u   /* 9600: ~7 ms/frame  |  2400: ~29 ms/frame */


/*=============================================================================
 * [3] Speed range  (unit: 0.1 km/h)
 *     Hold step = TM_SPEED_STEP × TM_SPEED_HOLD_STEP_MULT
 *===========================================================================*/
#define TM_SPEED_MIN                10u     /* 1.0 km/h — minimum */
#define TM_SPEED_MAX                100u     /* 8.0 km/h — maximum */
#define TM_SPEED_STEP               1u      /* 0.1 km/h per tap   */
#define TM_SPEED_HOLD_STEP_MULT     5u      /* × 5 when held       */


/*=============================================================================
 * [4] Workout targets: Time / Distance / Calories
 *===========================================================================*/
/* Time: stored in seconds; UI shows 0:mm.  Range 5..99 minutes. */
#define TM_TIME_MIN                 300u        /*  5 min */
#define TM_TIME_MAX                 5940u       /* 99 min */
#define TM_TIME_STEP                60u         /*  1 min per tap */
#define TM_TIME_SET                 1800u       /* default 30 min */
#define TM_TIME_HOLD_STEP_MULT      5u

/* Distance: stored in metres; UI shows xx.x km. */
#define TM_DIST_MIN                 1000.0f     /*  1.0 km */
#define TM_DIST_MAX                 99000.0f    /* 99.0 km */
#define TM_DIST_STEP                1000.0f     /*  0.1 km per tap */
#define TM_DIST_SET                 5000.0f     /* default 5.0 km */
#define TM_DIST_HOLD_STEP_MULT      5u

/* Calories: stored in milli-kcal (×1000); UI shows integer kcal. */
#define TM_CAL_MIN                  20000.0f    /*  20 kcal */
#define TM_CAL_MAX                  990000.0f   /* 990 kcal */
#define TM_CAL_STEP                 10000.0f    /*  10 kcal per tap */
#define TM_CAL_SET                  100000.0f   /* default 100 kcal */
#define TM_CAL_HOLD_STEP_MULT       5u

/* Calorie burn model: kcal per km. */
#define CAL_PER                     70u


/*=============================================================================
 * [5] Motor controller parameters  (pushed to lower board at every power-on)
 *
 *  HOW TO ADAPT TO YOUR MOTOR TYPE — change only this section:
 *
 *  ┌─────────────────────┬────────────────────┬────────────────────┐
 *  │ Parameter           │ 110 V motor        │ 220 V motor        │
 *  ├─────────────────────┼────────────────────┼────────────────────┤
 *  │ VOLTAGE_MAX         │  95                │ 187                │
 *  │ VOLTAGE_MIN         │  20                │  40                │
 *  │ STA_LEVEL (1..5)    │  1 (40V/3)         │  2 (80V/5)         │
 *  │ OVER_VOLTAGE_MAX    │ 130                │ 240                │
 *  │ OVER_CURRENT_MAX    │  30 (same)         │  30 (same)         │
 *  └─────────────────────┴────────────────────┴────────────────────┘
 *
 *  After editing, rebuild and flash UPPER BOARD only.
 *  Lower board firmware stays the same for any motor type.
 *===========================================================================*/

/* --- Motor type: uncomment the block that matches your motor --- */

/* === 220 V motor (default) ============================================== */
#define TM_LOWER_VOLTAGE_MAX        90u    /* max drive voltage (V)        */
#define TM_LOWER_VOLTAGE_MIN        18u    	/* min drive voltage (V)，1km/h基础电压 */
#define TM_LOWER_OVER_VOLTAGE_MAX   140u    /* runaway protection (V)       */
#define TM_LOWER_OVER_CURRENT_MAX   54u
#define TM_LOWER_SPEED_MAX_PARAM    100u     /* 10.0 km/h */
#define TM_LOWER_TX_SPEED_MAX_BYTE  ((uint8_t)(TM_LOWER_SPEED_MAX_PARAM))


/* HC-32_1：软起仅 CMD_STA_LEVEL(28)，档位表见下控 uart_frame.c（1→40/3 … 5→200/8）
 * 110V 电机用 1（start_valtage=40 ADC≈5V），220V 用 2（80 ADC≈10V） */
#define TM_LOWER_STA_LEVEL          2u    /* 110V 大功率电机用 1 */
/* --- Stop ramp: lower board declares stopped when back-EMF < this (V) --- */
#define TM_LOWER_END_LEVEL          8u

/* --- Acceleration curve (running): lower cmd CMD_ACCELERATED_LEVEL (30) → motor.accelerated
 * Range 1..6 per uart_frame.c. */
 #define TM_LOWER_ACCEL_LEVEL        2u  // 91ms,0.1km/h 10s：10

/* --- Deceleration curve (stop ramp) ---
 * Step removed per 91 ms tick.  Range 1..10.
 * At step 5 from 8.0 km/h: ~14.5 s to reach stop threshold. */
#define TM_LOWER_DECEL_LEVEL        2u

/* --- KIV: 电流注入电压补偿（每 1km/h 速度段）
 *
 * 物理含义：当马达电流超过 adc_base_curren 基线时，
 *   额外电压 = (实测电流 - 基线电流) × kiv / 10   (V×8.13单位)
 *
 * 调参背景（实测掉速数据）：
 *   最低速：空载22m/min，有人16m/min → 掉速27%，需大幅提升低速KIV
 *   最高速：空载132m/min，有人116m/min → 掉速12%，高速KIV也需同步提升
 *   脚麻根源 = 皮带比脚慢 = 摩擦 → 与掉速同根，提升KIV即可同时解决
 *
 * 补偿原理：KIV/10 需≈ R内阻归一化值，才能完全补偿 I×R 压降
 *   此次 KIV 从 3~4 提升至 6~8，目标将掉速控制在 <5%
 *
 * 调参指导：
 *   空载电压不稳/超调抖动 → 减小对应速度的kiv
 *   带载仍明显掉速        → 继续增大kiv（上限10）
 *   低速过补偿（速度随踩踏上升）→ 减小1~3km/h的kiv */
/* KIV 调参原则（配合 adc_base_curren 平滑递增后）：
 *   低速（1~5km/h）：重人慢走皮带要"硬"——KIV 高，base 已降至 0.5A，线性激活无死区
 *   高速（6~10km/h）：声音要"纯净"——KIV 极低，靠 kd=5 抑噪，不引入额外振荡
 *
 * 若低速仍软：KIV_2/3KM 可再升至 6；若一抖一抖：降回 4
 * 若高速有异响：KIV_6/7KM 降至 1 */
/* KIV 全部清零：先纯 PD 调脚麻，调好后再逐步加 KIV 补掉速 */
#define TM_LOWER_KIV_1KM             4u
#define TM_LOWER_KIV_2KM             4u
#define TM_LOWER_KIV_3KM             3u
#define TM_LOWER_KIV_4KM             3u
#define TM_LOWER_KIV_5KM             3u
#define TM_LOWER_KIV_6KM             2u
#define TM_LOWER_KIV_7KM             3u
#define TM_LOWER_KIV_8KM             2u
#define TM_LOWER_KIV_9KM             1u
#define TM_LOWER_KIV_10KM            1u
#define TM_LOWER_KIV_11KM            2u   /* 快跑·高速反电动势大，I×R占比小 */
#define TM_LOWER_KIV_12KM            2u

/* PID 参数由下控固件内部固定，上控不下发 */


/*=============================================================================
 * [6] Timing & boot sequencing
 *     s_cfg_table: sent during BOOT full-on (after lower-board gate), before IDLE.
 *===========================================================================*/

/* Boot: all segments on (100 ms ticks). 20 = 2.0 s; no keys during this. */
#define TM_BOOT_FULL_TICKS              20u

/* 250 ms ticks to wait before starting parameter download (lower board boot). */
#define TM_LOWER_CFG_250MS_GATE         1u   /* 250 ms */

/* Additional 100 ms ticks after gate before first parameter frame. */
#define TM_LOWER_CFG_POST_GATE_100MS    2u   /* 200 ms */

/* CMD_READ_PARAM 现由验证状态机统一管理，此处禁用旧的单次发送 */
#define TM_LOWER_SEND_READ_PARAM_BOOT   0u

/* 参数下发：每100ms发2行，30行共1.5s完成
 * 下控"发什么回什么"（echo cmd+data[0]），上控不阻塞等待，echo仅用于新固件检测 */
#define TM_LOWER_CFG_ROWS_PER_TICK      2u   /* 每100ms发送行数 */

/* Stop display ramp BASE duration at maximum speed (100 ms ticks).
 * 45 s at max speed → 45 × 10 = 450 ticks.
 * Actual ramp = TM_STOP_TIME_COUNT × start_speed / TM_SPEED_MAX.
 *   At 1.0 km/h: 450 × 10/80 ≈ 56 ticks = 5.6 s
 *   At 8.0 km/h: 300 × 80/80 = 300 ticks = 30.0 s 
 *   At 10.0 km/h: 300 × 100/1000 = 400 ticks = 40.0 s */
#define TM_STOP_TIME_COUNT              360u

/* Safety fallback: force IDLE this many 100 ms ticks after display reaches 0
 * without receiving CMD_STOP_OVER.  30 = 3 s extra grace period. */
#define TM_STOP_SAFETY_TIMEOUT          30u

/* Running-state comm timeout (100 ms ticks).  50 = 5 s before er01. */
#define TM_COMM_TIMEOUT_COUNT           50u

/* Error alarm beep count.  Safety-key error always uses 255 (continuous). */
#define TM_ERROR_BEEP_COUNT             10u


/*=============================================================================
 * [7] State machine types  — do not edit
 *===========================================================================*/
typedef enum {
    TM_STATE_BOOT = 0,
    TM_STATE_IDLE,
    TM_STATE_SETTING,
    TM_STATE_COUNTDOWN,
    TM_STATE_RUNNING,
    TM_STATE_STOPPING,
    TM_STATE_ERROR
} TM_State_t;

typedef enum {
    DISP_SPEED = 0,
    DISP_TIME,
    DISP_DIST,
    DISP_CAL
} TM_DispIndex_t;

typedef enum {
    TARGET_NONE = 0,
    TARGET_TIME,
    TARGET_DIST,
    TARGET_CAL
} TargetMode_t;

typedef struct {
    TM_State_t      state;
    TargetMode_t    target_mode;
    TM_DispIndex_t  disp_index;

    uint16_t        speed;      /* current speed, 0.1 km/h units */
    uint32_t        time;       /* elapsed / remaining seconds   */
    float           dist;       /* elapsed / remaining metres    */
    float           cal;        /* elapsed / remaining milli-kcal*/

    uint16_t        timer_100ms;/* general-purpose countdown timer (100 ms ticks) */
    uint8_t         steady_timer;/* UI blink inhibit after key press (100 ms ticks)*/
    uint16_t        cycle_timer; /* display auto-cycle counter (100 ms ticks)      */
    _Bool           safety_key;  /* safety-key present flag                        */
    uint16_t        error_code;  /* last error code (0 = none)                     */
    uint8_t         lower_c_bcd; /* lower board version from CMD_ACK (0 = not yet) */
} TM_Control_t;

extern TM_Control_t g_TM;


/*=============================================================================
 * [8] Public API
 *===========================================================================*/
void  Treadmill_Init(void);
void  Treadmill_Manager_100ms(void);
bool  Treadmill_On_Event(uint8_t key_id, uint8_t evt);
_Bool Treadmill_ConsumeKeyBeepSuppress(void);
_Bool Treadmill_CmdTestDisplayActive(void);
_Bool Treadmill_LowerBoardReady(void);
void  Treadmill_On250msTasks(void);
void  Treadmill_On1000msUartWatchdog(void);

#endif /* __TREADMILLS_H */
