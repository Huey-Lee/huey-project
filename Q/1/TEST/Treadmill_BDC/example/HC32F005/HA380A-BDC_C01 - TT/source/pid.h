/* pid.h - 占空比/步进边界（原速度环 PID 与串口调参已移除，宏名保留以少改调用处） */
#ifndef _PID_H
#define _PID_H

#define UK_STEP_MAX (20)
#define UK_STEP_MIN (-20)
#define UK_STEP_MAX_LOW_SPD (10)
#define UK_PWM_MAX  (540)
#define UK_PWM_MIN   (1)

/* 电压指令=adjust_max 时前馈占空上限 = UK_PWM_MAX×百分数/100（8.13 与实测母线不一致时勿用满占空） */
#ifndef MOTOR_PWM_AT_VMAX_PCENT
#define MOTOR_PWM_AT_VMAX_PCENT  (88u)
#endif
#define MOTOR_PWM_TOP_AT_VMAX  ((uint16_t)(((uint32_t)UK_PWM_MAX * (uint32_t)MOTOR_PWM_AT_VMAX_PCENT) / 100u))

#endif
