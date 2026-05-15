#ifndef __TREADMILLS_H
#define __TREADMILLS_H

#include "main.h"

/** 速度上下限（0.1 km/h），与显示板目标速一致，供 Keylink 0x30 同步等共用 */
#define TM_SPEED_0P1_MAX  120u
#define TM_SPEED_0P1_MIN  10u

/**
 * 按键板状态 + 运行量测：供 Keylink 心跳、蓝牙 V2.3 运动数据包、显示同步
 */
typedef enum {
    TM_STATE_BOOT = 0,
    TM_STATE_IDLE,
    TM_STATE_RUNNING
} TM_State_t;

typedef struct {
    TM_State_t state;
    uint16_t   step_rate;   /* SFC01 下位步频（显示/蓝牙心率独立为 hr_bpm） */
    uint16_t   timer_100ms; /* BOOT 切 IDLE；走时基 */
    uint16_t   speed_0p1;   /* 实时速度 0.1 km/h */
    uint32_t   dist_mm;     /* 累计距离 mm（运行中积分） */
    uint16_t   kcal;        /* 估算千卡 */
    uint32_t   time_s;      /* 运动时间 s */
    uint8_t    hr_bpm;      /* 心率（无传感器可用步频截断代替） */
    uint8_t    app_ctrl_ok; /* 蓝牙 APP 已应答请求控制 */
} TM_Control_t;

extern TM_Control_t g_TM;

void Treadmill_Init(void);
void Treadmill_Manager_100ms(void);
void Treadmill_On_Event(uint8_t key_id, uint8_t evt);

#endif
