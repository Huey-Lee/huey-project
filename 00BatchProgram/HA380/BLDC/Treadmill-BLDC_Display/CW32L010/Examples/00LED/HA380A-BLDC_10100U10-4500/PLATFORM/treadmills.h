/*
 * treadmills.h  -  Single configuration file for the treadmill display board
 *
 * THIS IS THE ONLY FILE YOU NEED TO MODIFY.
 * All display behaviour is defined here. After editing, rebuild and flash;
 * set TM_UART_BAUD to match the lower board if you change baud.
 *
 * Speed units: 0.1 km/h integers.  g_TM.speed / UI use TM_SPEED_MIN..MAX（显示 1.0..10.0 km/h）.
 *   TM_SPEED_MIN = 10  ->  1.0 km/h（显示与按键下限）
 *   TM_SPEED_MAX = 100 -> 10.0 km/h（显示与按键上限）
 * UART 档位 FC_GEAR Data1 为下发表 TM_DOWNLINK_SPEED_MIN..MAX（1.0..8.0 km/h），与显示线性对应。
 * Downlink (上显→下控): 17 字节帧 AA BB 0x0A FC SFC Data×8 CS_H CS_L EE FF
 *   档位 FC=0x03 SFC=0x00：目标档位在 Data1（大端），与 TM_DOWNLINK_* 一致（下发表，0.1 km/h）
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
#define FW_VER_C    0u    /* 下控版本无新协议统一上报位时作 BOOT 占位 */


/*=============================================================================
 * [2] Communication
 *     [BOTH BOARDS] TM_UART_BAUD must match the lower board UART baud.
 *     帧格式与 00YS626_1060U1.0 一致；下控须解析 0x09 上行并按表回应运行/故障等。
 *     链路：上控周期发 FC_HEARTBEAT(01/00) 心跳，下控应回包（同心跳镜像或运行/故障帧）；
 *     任一合法下控帧清零超时计数，连续 TM_LINK_LOST_TIMEOUT_SEC 秒无收包且已 LowerReady → E01。
 *===========================================================================*/
#define TM_UART_BAUD                4800u


/*=============================================================================
 * [3] Speed range  (unit: 0.1 km/h)
 *     显示/按键目标：TM_SPEED_MIN..TM_SPEED_MAX（1.0..10.0 km/h，步进 0.1）。
 *     下控下发：TM_DOWNLINK_SPEED_MIN..MAX（1.0..8.0 km/h），与显示线性对应：
 *       显示 1.0 ↔ 下发 1.0；显示 10.0 ↔ 下发 8.0；中间均匀插值；下发不低于 1.0（10）。
 *     Hold step = TM_SPEED_STEP × TM_SPEED_HOLD_STEP_MULT
 *===========================================================================*/
#define TM_SPEED_MIN                10u     /* 1.0 km/h — display / UI minimum */
#define TM_SPEED_MAX                100u    /* 10.0 km/h — display / UI maximum */
#define TM_SPEED_STEP               1u      /* 0.1 km/h per tap   */
#define TM_SPEED_HOLD_STEP_MULT     5u      /* × 5 when held       */

#define TM_DOWNLINK_SPEED_MIN       10u     /* 1.0 km/h — minimum sent to lower */
#define TM_DOWNLINK_SPEED_MAX       80u     /* 8.0 km/h — maximum sent to lower */


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
 * [5] Timing & boot sequencing
 *     上电门控后仅发两次标准停机 (FC_CONTROL 停)，无其它下参。
 *===========================================================================*/

/* Boot: all segments on (100 ms ticks). 20 = 2.0 s; no keys during this. */
#define TM_BOOT_FULL_TICKS              20u

/* 250 ms ticks to wait before starting boot UART burst (lower board boot). */
#define TM_LOWER_CFG_250MS_GATE         1u   /* 250 ms */

/* Additional 100 ms ticks after gate before first UART frame in burst. */
#define TM_LOWER_CFG_POST_GATE_100MS    2u   /* 200 ms */

/* Stop display ramp BASE duration at maximum speed (100 ms ticks).
 * 45 s at max speed → 45 × 10 = 450 ticks.
 * Actual ramp = TM_STOP_TIME_COUNT × start_speed / TM_SPEED_MAX.
 *   At 1.0 km/h: 450 × 10/80 ≈ 56 ticks = 5.6 s
 *   At 8.0 km/h: 300 × 80/80 = 300 ticks = 30.0 s 
 *   At 10.0 km/h: 300 × 100/1000 = 400 ticks = 40.0 s */
#define TM_STOP_TIME_COUNT              390u

/* Safety fallback: STOPPING 且下位反馈速已为 0 时，phase2 最长等待（100 ms ticks） */
#define TM_STOP_SAFETY_TIMEOUT          30u

/* 下控 FC_HEARTBEAT(01/00) data[2] 常用值（与下位协议一致时可改） */
#define TM_LOWER_STATUS_RUNNING         9u
#define TM_LOWER_STATUS_IDLE            10u /* 待机/等待启动 */

/* 反馈速已为 0 而状态尚未切到待机时，仅时间宽限（100ms 拍）；界面不闪 0.1 */
#define TM_STOP_SPD0_GRACE_TICKS        15u

/* 与下控失联判据：Task_1000ms 中累加，单位秒；≥此值报 E01（通信类）。 */
#define TM_LINK_LOST_TIMEOUT_SEC        5u

/* 发启动/停机后，在此时间内（×100ms）若未见到下控状态/速度预期变化，则仅再补发一次 */
#define TM_LOWER_CMD_ACK_WAIT_TICKS    10u

/* Error alarm beep count.  Safety-key error always uses 255 (continuous). */
#define TM_ERROR_BEEP_COUNT             10u

/* 433 遥控器对码（1=开启，0=关闭编译进工程）
 * 开启时：上电后 TM_RF_PAIR_WINDOW_10MS（6s）内长按「速度加」(K_ID_UP) 累计 TM_RF_PAIR_HOLD_10MS（3s），
 * 听到双短鸣后写入 Flash；此后仅接受已存储地址的遥控器；未对码前任一合法地址键值均可操作。
 * 存储占用 Flash 末页（见 plat_rf_store.c），请确认固件体积未覆盖该页。 */
#define TM_RF_PAIRING_ENABLE            0
#define TM_RF_PAIR_WINDOW_10MS          600u   /* 6 s */
#define TM_RF_PAIR_HOLD_10MS            300u   /* 3 s 连续按住加键（10ms 节拍累加） */


/*=============================================================================
 * [6] State machine types  — do not edit
 *===========================================================================*/
typedef enum {
    TM_STATE_BOOT = 0,
    TM_STATE_IDLE,
    TM_STATE_SETTING,
    TM_STATE_COUNTDOWN,
    TM_STATE_RUNNING,
    TM_STATE_STOPPING,
    TM_STATE_TEST,      /* 下控发 AA BB 09 FF FF（收帧内 CID=09 FC=FF SFC=FF）进入 */
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

    uint16_t        speed;      /* 目标/显示速度，0.1km/h，运行中为 TM_SPEED_MIN..MAX；0=停机显示 */
    uint32_t        time;       /* elapsed / remaining seconds   */
    float           dist;       /* elapsed / remaining metres    */
    float           cal;        /* elapsed / remaining milli-kcal*/

    uint16_t        timer_100ms;/* general-purpose countdown timer (100 ms ticks) */
    uint8_t         steady_timer;/* UI blink inhibit after key press (100 ms ticks)*/
    uint16_t        cycle_timer; /* display auto-cycle counter (100 ms ticks)      */
    _Bool           safety_key;  /* safety-key present flag                        */
    uint16_t        error_code;  /* last error code (0 = none)                     */
    uint8_t         lower_c_bcd; /* 下控版本（旧 CMD_ACK）；新协议暂无则保持 0 */
} TM_Control_t;

extern TM_Control_t g_TM;


/*=============================================================================
 * [7] Public API
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
