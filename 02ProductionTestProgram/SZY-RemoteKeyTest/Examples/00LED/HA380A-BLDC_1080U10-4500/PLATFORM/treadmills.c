/* Demo 工程占位：保留 treadmills.h 中的类型与 RF 对码宏，业务已移至 app_remote_demo */
#include "treadmills.h"

TM_Control_t g_TM;

void Treadmill_Init(void) {}

void Treadmill_Manager_100ms(void) {}

void Treadmill_On250msTasks(void) {}

void Treadmill_On1000msUartWatchdog(void) {}

bool Treadmill_On_Event(uint8_t key_id, uint8_t evt)
{
    (void)key_id;
    (void)evt;
    return false;
}

_Bool Treadmill_ConsumeKeyBeepSuppress(void)
{
    return 0;
}

_Bool Treadmill_CmdTestDisplayActive(void)
{
    return 0;
}

_Bool Treadmill_LowerBoardReady(void)
{
    return 0;
}
