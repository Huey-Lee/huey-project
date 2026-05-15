#ifndef __TREADMILLS_H
#define __TREADMILLS_H

#include "main.h"

/* Boot: show version (U/C) if defined; else full-on then idle */
//#define BOOT_SHOW_VERSION

/* FW version */
#define FW_VER_U    10   /* U 1.0 */
#define FW_VER_C    1    /* C 0.1 */


/* ---- 可调参数（统一修改入口） ---- */

/* 时间目标：单位秒，UI 以 0:mm 显示，设置范围 5~99 分钟 */
#define TM_TIME_MIN           300u
#define TM_TIME_MAX          5940u
#define TM_TIME_STEP           60u
#define TM_TIME_SET          1800u    /* 默认 30 分钟 */

/* 路程目标：内部单位米，UI 显示 xx.x km */
#define TM_DIST_MIN        1000.0f    /* 1.0 km */
#define TM_DIST_MAX       99000.0f    /* 99.0 km */
#define TM_DIST_STEP       1000.0f    /* 0.1 km / 步 */
#define TM_DIST_SET        5000.0f    /* 默认 5.0 km */

/* 卡路里目标：内部单位 0.001 kcal，UI 显示 20~990 */
#define TM_CAL_MIN        20000.0f
#define TM_CAL_MAX       990000.0f
#define TM_CAL_STEP       10000.0f
#define TM_CAL_SET       100000.0f    /* 默认 100 kcal */

/* 速度：面板 0.6～6.2 km/h，内部 6～62（十分之一 km/h）；线侧档位 1～10（对齐 ALR-C60T 换算）*/
#define TM_SPEED_MIN              6u   /* 0.6 km/h */
#define TM_SPEED_MAX             62u   /* 6.2 km/h */
#define TM_GEAR_WIRE_MIN          1u
#define TM_GEAR_WIRE_MAX         10u
#define TM_SPEED_STEP             1u

/* 长按加速倍率 */
#define TM_TIME_HOLD_STEP_MULT   1u
#define TM_DIST_HOLD_STEP_MULT   1u
#define TM_CAL_HOLD_STEP_MULT    1u
#define TM_SPEED_HOLD_STEP_MULT  1u

/* 等下控应答待机(run_status=10) 最长 5s，超时仍回待机以防丢包 */
#define TM_LOWER_STANDBY_WAIT_MAX  50u

/* 停机斜坡上限（×100ms，例160=16s）。实际斜坡=min(本值, max(下限, init×TM_STOP_TICKS_PER_TENTH))，低速更快 */
#define TM_STOPPING_DURATION_100MS  160u
#define TM_STOP_TICKS_PER_TENTH      8u   /* 每 0.1km/h 占 8×100ms（1.0km/h≈8s） */
#define TM_STOP_MIN_RAMP_100MS      40u   /* 斜坡最短 4s，避免高档位过陡 */

/* 运行中下控速度与上显设定比对周期（×100ms），不一致则重发当前显示速度 */
#define TM_SPEED_RESEND_PERIOD_100MS  5u

/* 下控运行信息 Data1：RPM 大端；换算为上显 0.1km/h（按辊径/减速比改分母） */
#define TM_LOWER_RPM_TENTH_NUM     1u
#define TM_LOWER_RPM_TENTH_DEN    100u

/* 热量换算：kcal/km */
#define CAL_PER               70u

/* Program 模式：12 个程序，每程序 10 段 × 3 分钟 = 30 分钟 */
#define TM_PROG_COUNT          12u    /* 程序数 P01-P12 */
#define TM_PROG_SEG_COUNT      10u    /* 每程序段数 */
#define TM_PROG_SEG_SEC       180u    /* 每段 3 分钟（秒） */
#define TM_PROG_TOTAL_SEC    1800u    /* 程序总时长 30 分钟（秒） */

#ifndef TM_BOOT_FULLON_MS
#define TM_BOOT_FULLON_MS            1500u
#endif
#define TM_BOOT_VERSION_EACH_MS      800u
#ifndef TM_SETTING_IDLE_EXIT_100MS
#define TM_SETTING_IDLE_EXIT_100MS  600u
#endif
#ifndef TM_ERROR_REPORT_ENABLE
#define TM_ERROR_REPORT_ENABLE       0
#endif


/* ---- 枚举类型 ---- */

typedef enum {
    TM_STATE_BOOT = 0,
    TM_STATE_IDLE,
    TM_STATE_SETTING,
    TM_STATE_COUNTDOWN,
    TM_STATE_RUNNING,
    TM_STATE_STOPPING,
    TM_STATE_PAUSED,
    TM_STATE_ERROR
} TM_State_t;

typedef enum {
    DISP_SPEED   = 0,  /* 速度显示/设置 */
    DISP_TIME    = 1,  /* 时间显示/设置 */
    DISP_DIST    = 2,  /* 路程显示/设置 */
    DISP_CAL     = 3,  /* 卡路里显示/设置 */
    DISP_PROGRAM = 4,  /* P01-P12 程序模式选择 */
    DISP_NONE    = 5
} TM_DispIndex_t;

typedef enum {
    TARGET_NONE    = 0,
    TARGET_TIME    = 1,
    TARGET_DIST    = 2,
    TARGET_CAL     = 3,
    TARGET_PROGRAM = 4   /* 预设程序锻炼 */
} TargetMode_t;

typedef enum {
    BOOT_FULL,
    BOOT_VER_U,
    BOOT_VER_C,
    BOOT_DONE
} BootPhase_t;


/* ---- 主控数据结构 ---- */

typedef struct {
    TM_State_t      state;
    BootPhase_t     boot_phase;
    TargetMode_t    target_mode;
    TM_DispIndex_t  disp_index;

    uint16_t        speed;          /* 当前速度，0.1 km/h（6～62≈0.6～6.2km/h） */
    uint32_t        time;           /* 时间（秒）：TARGET_TIME 时倒计时，否则正计时 */
    float           dist;           /* 路程（米） */
    float           cal;            /* 卡路里（0.001 kcal） */

    uint16_t        timer_100ms;    /* 通用 100ms 计数器 */
    uint8_t         steady_timer;   /* 按键防抖/稳定延时 */
    uint16_t        cycle_timer;    /* 非 TM1640 轮显计时 */
    uint16_t        error_code;     /* 下位机故障码等 */
    _Bool           pending_lower_standby; /* 已发停机：等下控 run_status=10 再清零回待机 */
    uint16_t        lower_standby_wait_100ms;

    _Bool           resume_from_pause;  /* 自暂停恢复：321 后勿重采目标里程/时间初值 */

    /* Program 模式专用字段 */
    uint8_t         prog_index;         /* 0..11 → P01..P12 */
    uint8_t         prog_seg;           /* 0..9  当前段序号 */
    uint16_t        seg_elapsed_sec;    /* 当前段已运行秒数 */
} TM_Control_t;

extern TM_Control_t g_TM;

/* 安全锁 PB04：由 plat_safetylock 去抖后调用，勿在别处直接调 */
#define TM_ERR_SAFETY_LOCK   17u
void Treadmill_OnSafetyLockOpened(void);
void Treadmill_OnSafetyLockClosed(void);

void Treadmill_Init(void);
void Treadmill_Manager_100ms(void);
bool Treadmill_On_Event(uint8_t key_id, uint8_t evt);

#endif /* __TREADMILLS_H */
