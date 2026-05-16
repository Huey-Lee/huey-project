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

/* === 220 V motor — 参数已按「判停/蠕行 + 开环过冲」向保守侧调校，仅烧上显 ================== */
#define TM_LOWER_VOLTAGE_MAX        187u    /* 与表内 220 V 列一致；原 180 可略欠顶速 */
#define TM_LOWER_VOLTAGE_MIN        40u     /* 与表内 220 V 列一致；1 km/h 电压地板 */
#define TM_LOWER_OVER_VOLTAGE_MAX   240u
#define TM_LOWER_OVER_CURRENT_MAX   30u
#define TM_LOWER_SPEED_MAX_PARAM    100u
#define TM_LOWER_TX_SPEED_MAX_BYTE  ((uint8_t)(TM_LOWER_SPEED_MAX_PARAM))


/* HC-32_1：软起仅 CMD_STA_LEVEL(28)；220 V 大静摩擦可试 3，过冲则退回 2 */
#define TM_LOWER_STA_LEVEL          2u
/* 判停门限（2..25）：略增大易满足端压判据，利于 220 V 机不收尾、最低速蠕行 */
#define TM_LOWER_END_LEVEL          20u

/* 运行加速/停机减速：加缓减可略快，减轻再加速「顶一下」体感 */
#define TM_LOWER_ACCEL_LEVEL         1u
#define TM_LOWER_DECEL_LEVEL         3u

/* KIV 略降：减少 RUN/STOP 段 IR 在低速多顶占空，助减蠕行与过冲（台架可再细调） */
#define TM_LOWER_KIV_1KM             0u
#define TM_LOWER_KIV_2KM             1u
#define TM_LOWER_KIV_3KM             2u
#define TM_LOWER_KIV_4KM             2u
#define TM_LOWER_KIV_5KM             2u
#define TM_LOWER_KIV_6KM             2u
#define TM_LOWER_KIV_7KM             2u
#define TM_LOWER_KIV_8KM             2u
#define TM_LOWER_KIV_9KM             2u
#define TM_LOWER_KIV_10KM            2u
#define TM_LOWER_KIV_11KM            2u
#define TM_LOWER_KIV_12KM            2u


/*=============================================================================
 * [5b] 433 MHz 遥控器对码（与 HA380A-BLDC_1080U10-4500 一致）
 *===========================================================================*/
#define TM_RF_PAIRING_ENABLE            1u
#define TM_RF_PAIR_WINDOW_10MS          600u
#define TM_RF_PAIR_HOLD_10MS            300u


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
 *   At 8.0 km/h: 300 × 80/80 = 300 ticks = 30.0 s 
 *   At 10.0 km/h: 300 × 100/1000 = 400 ticks = 40.0 s */
#define TM_STOP_TIME_COUNT              390u

/* 显示速度已降到 0（进入「最低速→下位确认完全停」阶段）后，仅在该阶段累计计时；
 * 超过仍未收到下位 CMD_STOP_OVER（彻底停止）则报 E01，不得无确认回待机。100 ms = 1 tick。 */
#define TM_STOP_FULL_STOP_WAIT_MAX      80u   /* 例：80×100ms=8s；需盖住下位续流+断继+UART（见 motor.c ~1.5s） */

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
