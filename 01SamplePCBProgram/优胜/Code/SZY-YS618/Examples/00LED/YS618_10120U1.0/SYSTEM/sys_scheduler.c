#include "sys_scheduler.h"
#include "plat_beep.h"
#include "ui_display.h"
#include "plat_segment.h"
#include "plat_keyfun.h"
#include "treadmills.h"
#include "plat_aerolink.h"
#include "plat_keylink_rx.h"


void Task_1ms(void) 
{
	Beep_Handler_1ms();
	KeylinkRx_Handler_1ms();
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
	KeylinkRx_OnTimer100ms();
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

// ???????????? (?????????????Delay??????????????????)
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

// ??????????????? 1ms ????????????
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

// ???????????? while(1) ????????
void Scheduler_Dispatch(void) {
    for (int i = 0; i < SCH_MAX_TASKS; i++) {
        if (SCH_tasks_G[i].RunMe > 0) {
            SCH_tasks_G[i].pTask(); // ??????????
            SCH_tasks_G[i].RunMe--; /* ??????? */
        }
    }
}
