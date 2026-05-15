#include "sys_scheduler.h"
#include "plat_beep.h"
#include "ui_display.h"
#include "plat_segment.h"
#include "plat_keyfun.h"
#include "treadmills.h"
#include "plat_aerolink.h"


void Task_1ms(void) 
{
	Beep_Handler_1ms();
	AeroLink_Handler();
}

void Task_10ms(void) 
{
	Keypad_Handler_10ms();  
}

void Task_20ms(void) 
{
	UI_Engine_Refresh();
}

void Task_100ms(void) 
{
	Treadmill_Manager_100ms();
}

void Task_250ms(void) 
{

}

void Task_300ms(void) 
{

}

void Task_500ms(void) 
{

}

void Task_1000ms(void) 
{

}


#define SCH_MAX_TASKS (sizeof(SCH_tasks_G) / sizeof(sTask))

// 任务列表配置 (核心：通过不同的Delay避开同一时刻触发多个任务)
static sTask SCH_tasks_G[] = {
	{Task_1ms,	 	0,   1,   0},
    {Task_10ms,	 	3,   10,   0},
    {Task_20ms,  	7,   20,   0}, 
    {Task_100ms, 	13,   100,  0},
    {Task_250ms, 	21,   250,  0},
    {Task_300ms, 	37,   300,  0},
    {Task_500ms, 	43,  500,  0},
    {Task_1000ms,	59,  1000, 0}
};

// 更新任务计数（在 1ms 中断中运行）
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

// 分发执行任务（在 while(1) 中运行）
void Scheduler_Dispatch(void) {
    for (int i = 0; i < SCH_MAX_TASKS; i++) {
        if (SCH_tasks_G[i].RunMe > 0) {
            SCH_tasks_G[i].pTask(); // 执行具体业务
            SCH_tasks_G[i].RunMe--; /* 递减计数 */
        }
    }
}
