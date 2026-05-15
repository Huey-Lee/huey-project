#ifndef __SYS_SCHEDULER_H
#define __SYS_SCHEDULER_H

#include "cw32l010.h"


typedef struct {
    void (*pTask)(void);    // 任务函数指针
    uint32_t Delay;         // 初始延迟（用于平摊CPU负载）
    uint32_t Period;        // 执行周期
    uint32_t RunMe;         // 运行计数（>0执行）
} sTask;

void Scheduler_Update(void);   // 定时器中调用
void Scheduler_Dispatch(void); // main循环中调用

void Task_10ms(void);
void Task_20ms(void);
void Task_100ms(void);
void Task_250ms(void);
void Task_300ms(void);
void Task_500ms(void);


#endif
