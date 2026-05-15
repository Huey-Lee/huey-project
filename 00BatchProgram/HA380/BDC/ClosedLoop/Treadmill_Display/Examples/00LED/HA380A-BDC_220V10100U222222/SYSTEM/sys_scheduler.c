#include "sys_scheduler.h"
#include "plat_beep.h"
#include "ui_display.h"
#include "plat_segment.h"
#include "plat_keyfun.h"
#include "treadmills.h"
#include "plat_aerolink.h"


void Task_1ms(void) {
    Beep_Handler_1ms();
}

void Task_10ms(void) {
    AeroLink_Handler();
    Keypad_Handler_10ms();
}

void Task_20ms(void) {
    UI_Engine_Refresh();
}

void Task_100ms(void) {
    Treadmill_Manager_100ms();
}

void Task_250ms(void) {
    Treadmill_On250msTasks(); /* 下控心跳 + 上电发参门控 */
}
void Task_300ms(void) {}
void Task_500ms(void) {}

void Task_1000ms(void) {
    Treadmill_On1000msUartWatchdog(); /* 长时间无收包 */
}


#define SCH_MAX_TASKS (sizeof(SCH_tasks_G) / sizeof(sTask))

// 任务列表（Delay错开，避免同一tick多任务重叠）
static sTask SCH_tasks_G[] = {
    {Task_1ms,    0,  1,    0},
    {Task_10ms,   3,  10,   0},
    {Task_20ms,   7,  20,   0},
    {Task_100ms,  13, 100,  0},
    {Task_250ms,  21, 250,  0},
    {Task_300ms,  37, 300,  0},
    {Task_500ms,  43, 500,  0},
    {Task_1000ms, 59, 1000, 0},
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
