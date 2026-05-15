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
#define TM_SPEED_MAX                80u     /* 8.0 km/h — maximum */
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
#define TM_LOWER_VOLTAGE_MAX        180u    /* max drive voltage (V)        */
#define TM_LOWER_VOLTAGE_MIN        35u    	/* min drive voltage (V)        */
#define TM_LOWER_OVER_VOLTAGE_MAX   240u    /* runaway protection (V)       */
#define TM_LOWER_OVER_CURRENT_MAX   30u
#define TM_LOWER_SPEED_MAX_PARAM    80u     /* 8.0 km/h */
#define TM_LOWER_TX_SPEED_MAX_BYTE  ((uint8_t)(TM_LOWER_SPEED_MAX_PARAM))


/* HC-32_1：软起仅 CMD_STA_LEVEL(28)，档位表见下控 uart_frame.c（1→40/3 … 5→200/8） */
#define TM_LOWER_STA_LEVEL          2u    /* 1..5，220V 典型用 3 */
/* --- Stop ramp: lower board declares stopped when back-EMF < this (V) --- */
#define TM_LOWER_END_LEVEL          10u

/* --- Acceleration curve (running): lower cmd CMD_ACCELERATED_LEVEL (30) → motor.accelerated
 * Range 1..6 per uart_frame.c. */
#define TM_LOWER_ACCEL_LEVEL         2u

/* --- Deceleration curve (stop ramp) ---
 * Step removed per 91 ms tick.  Range 1..10.
 * At step 5 from 8.0 km/h: ~14.5 s to reach stop threshold. */
#define TM_LOWER_DECEL_LEVEL         2u

/* --- KIV: current-injection voltage gain per 1 km/h speed band ---
 * Higher = more voltage boost when motor pulls extra current (load compensation).
 * Tune per-speed for each motor.  Typical: 2-3. */
#define TM_LOWER_KIV_1KM             1u
#define TM_LOWER_KIV_2KM             1u
#define TM_LOWER_KIV_3KM             3u
#define TM_LOWER_KIV_4KM             3u
#define TM_LOWER_KIV_5KM             3u
#define TM_LOWER_KIV_6KM             3u
#define TM_LOWER_KIV_7KM             3u
#define TM_LOWER_KIV_8KM             3u
#define TM_LOWER_KIV_9KM             3u
#define TM_LOWER_KIV_10KM            3u
#define TM_LOWER_KIV_11KM            3u
#define TM_LOWER_KIV_12KM            3u


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

/* Send CMD_READ_PARAM after boot download to verify lower board settings. */
#define TM_LOWER_SEND_READ_PARAM_BOOT   1u

/* Stop display ramp BASE duration at maximum speed (100 ms ticks).
 * 45 s at max speed → 45 × 10 = 450 ticks.
 * Actual ramp = TM_STOP_TIME_COUNT × start_speed / TM_SPEED_MAX.
 *   At 1.0 km/h: 450 × 10/80 ≈ 56 ticks = 5.6 s
 *   At 8.0 km/h: 300 × 80/80 = 300 ticks = 30.0 s */
#define TM_STOP_TIME_COUNT              300u

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
