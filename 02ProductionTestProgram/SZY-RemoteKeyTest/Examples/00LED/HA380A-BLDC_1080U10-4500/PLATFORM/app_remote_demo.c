/* HA380 演示：仅 433 遥控器 + TM1640 一位数码管显示键值 */
#include "app_remote_demo.h"
#include "ui_display.h"
#include "plat_beep.h"

/*
 * plat_keyfun Map_RF 映射（与当前遥控器码表一致）：
 *   0x2E -> K_ID_UP    → 显示 1（亮/+）
 *   0x3D -> K_ID_MODE  → 显示 2（M）
 *   0x5B -> K_ID_ONOFF → 显示 3
 *   0x4C -> K_ID_DN    → 显示 4（-）
 */

void App_RemoteDemo_Init(void)
{
    UI_ClearAll();
    UI_SetNumber(0u, 0u, 1u, 0u, ZERO_SHOW);
    UI_MarkDirty();
}

void App_RemoteDemo_OnKey(KeyID_t id, KeyEvt_t evt)
{
    uint16_t digit;

    if (evt != K_EVT_PRESS) {
        return;
    }
    switch (id) {
        case K_ID_UP:
            digit = 1u;
            break;
        case K_ID_MODE:
            digit = 2u;
            break;
        case K_ID_ONOFF:
            digit = 3u;
            break;
        case K_ID_DN:
            digit = 4u;
            break;
        default:
            return;
    }
    Beep_Play(BEEP_CLICK);
    UI_ClearAll();
    UI_SetNumber(0u, digit, 1u, 0u, ZERO_SHOW);
    UI_MarkDirty();
}
