/* bdc_init.c — 参考工程 MAIN_INIT：默认参数与限幅同步 */

#include "motor.h"
#include "motor_drive.h"

int MAIN_INIT(int argc)
{
    (void)argc;
    ctr_init();
    motor_drive_boot_sync_limits();
    return 0;
}
