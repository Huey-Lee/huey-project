#include "sys_scheduler.h"
#include "ui_display.h"
#include "plat_segment.h"
#include "plat_keyfun.h"
#include "treadmills.h"
#include "plat_aerolink.h"
#include "plat_safetylock.h"


void Task_10ms(void) {
    AeroLink_Handler();
    Keypad_Handler_10ms();
    SafetyLock_Poll_10ms();
}

void Task_20ms(void) {
    UI_Engine_Refresh();
}

void Task_100ms(void) {
    Treadmill_Manager_100ms();
}

void Task_250ms(void) {}
void Task_300ms(void) {}
void Task_500ms(void) {
    /* 每 500ms 向下控发心跳 FC=01 SFC=00（与下位约定一致） */
    if (g_TM.state != TM_STATE_BOOT) {
        AeroLink_Send(FC_HEARTBEAT, 0x00, (void*)0);
    }
}

#define SCH_MAX_TASKS (sizeof(SCH_tasks_G) / sizeof(sTask))

// 任务列表（Delay错开，避免同一tick多任务重叠）
static sTask SCH_tasks_G[] = {
    {Task_10ms,   3,  10,   0},
    {Task_20ms,   7,  20,   0},
    {Task_100ms,  13, 100,  0},
    {Task_250ms,  21, 250,  0},
    {Task_300ms,  37, 300,  0},
    {Task_500ms,  43,  500, 0},
};

// 任务计数更新（在1ms中断中调用）
void Scheduler_Update(void) {
    for (int i = 0; i < SCH_MAX_TASKS; i++) {
        if (SCH_tasks_G[i].Delay > 0) {
            SCH_tasks_G[i].Delay--;
        } else {
            SCH_tasks_G[i].RunMe++;
            SCH_tasks_G[i].Delay = SCH_tasks_G[i].Period - 1;
        }
    }
}

// 任务分发执行（在while(1)中调用）
void Scheduler_Dispatch(void) {
    for (int i = 0; i < SCH_MAX_TASKS; i++) {
        if (SCH_tasks_G[i].RunMe > 0) {
            SCH_tasks_G[i].pTask();
            SCH_tasks_G[i].RunMe--;
        }
    }
}
