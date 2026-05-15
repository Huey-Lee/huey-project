#ifndef __TREADMILLS_H
#define __TREADMILLS_H

#include "main.h"

/* 开机是否显示版本号片段 */
#ifndef BOOT_SHOW_VERSION
#define BOOT_SHOW_VERSION  0
#endif

/* --------- 参数范围（与用户界面约定一致） --------- */
#define TM_TIME_MIN      60       /* 秒：最小目标时间 */
#define TM_TIME_MAX      5940     /* 99 分 */
#define TM_TIME_STEP     60       /* 每次调节 1 分钟 */
#define TM_TIME_SET      1800

#define TM_DIST_MIN      1000.0f  /* 0.1 km 为单位，即 1000 = 1.0 km */
#define TM_DIST_MAX      99000.0f
#define TM_DIST_STEP     1000.0f
#define TM_DIST_SET      1000.0f

#define TM_CAL_MIN       10000.0f
#define TM_CAL_MAX       990000.0f
#define TM_CAL_STEP      10000.0f
#define TM_CAL_SET       50000.0f

#define TM_SPEED_MIN     10
#define TM_SPEED_MAX     120
#define TM_SPEED_STEP    1

#define TM_PROG_NUM           8u
#define TM_PROG_SEGMENTS      10u
#define TM_PROG_SEC_PER_SEG   (3u * 60u)
#define TM_PROG_TOTAL_SEC     ((uint32_t)TM_PROG_SEGMENTS * TM_PROG_SEC_PER_SEG)

/* 程式待机或设定闲置超过此时长（×100ms）自动回待机清零 */
#define TM_UI_IDLE_RETURN_TICKS  (3u * 60u * 10u)

#define CAL_PER          70.0f   /* kcal/km 估算 */

#define TM_ERR_SAFETY_LOCK  17u   /* 安全锁（磁控）断开 */

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
    DISP_HR = 0,   /* 运行轮显首项：心率（无数据时 000） */
    DISP_SPEED,
    DISP_TIME,
    DISP_DIST,
    DISP_CAL,
    DISP_NONE
} TM_DispIndex_t;

#define TM_RUN_DISP_CYCLE_LEN  5u

typedef enum {
    TARGET_NONE = 0,
    TARGET_TIME,
    TARGET_DIST,
    TARGET_CAL,
    TARGET_PROGRAM
} TargetMode_t;

typedef enum {
    BOOT_FULL,
    BOOT_VER_U,
    BOOT_VER_C,
    BOOT_DONE
} BootPhase_t;

typedef struct {
    TM_State_t     state;
    BootPhase_t    boot_phase;
    TargetMode_t   target_mode;
    TM_DispIndex_t disp_index;

    uint16_t       speed;
    uint16_t       hr_bpm;
    uint32_t       time;
    float          dist;
    float          cal;

    uint16_t       timer_100ms;
    uint8_t        steady_timer;
    uint16_t       cycle_timer;
    _Bool          safety_key;
    uint16_t       error_code;
    uint8_t        prog_id;
    uint8_t        prog_segment;
    uint16_t       prog_seg_timer;
    uint16_t       ui_idle_return_100ms;
} TM_Control_t;

extern TM_Control_t g_TM;

void Treadmill_Init(void);
void Treadmill_Manager_100ms(void);
void Treadmill_On_Event(uint8_t key_id, uint8_t evt);

/** 按键板 Keylink：实时目标速（0.1 km/h）；运行中会下发下位 */
void Treadmill_Keylink_SetSpeed0p1(uint16_t speed_0p1);
/** 运行中快捷键设速并发 FC_GEAR */
void Treadmill_Keylink_ApplyHotkey(uint16_t speed_0p1);
/** 按键板 Keylink 0x20：心率 BPM */
void Treadmill_Keylink_SetHeartRate(uint8_t bpm);
#endif
