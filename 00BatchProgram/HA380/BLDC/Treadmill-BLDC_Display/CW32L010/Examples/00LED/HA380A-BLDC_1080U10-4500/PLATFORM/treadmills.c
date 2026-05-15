/*
 * treadmills.c  -  Treadmill application logic
 *
 * Module layout:
 *   §1  Includes & module-level state
 *   §2  Communication — send primitives
 *   §3  Boot UART burst（仅双停）
 *   §4  Communication — link watchdog & frame dispatch
 *   §5  Error handling
 *   §6  Key input & target adjustment
 *   §7  Running state — countdown / run / stop / statistics
 *   §8  Display rendering
 *   §9  Public API
 */

#include "treadmills.h"
#include "ui_display.h"
#include "plat_segment.h"
#include "plat_beep.h"
#include "plat_keyfun.h"
#include "plat_aerolink.h"

/*=============================================================================
 * §1  Includes & module-level state
 *===========================================================================*/

TM_Control_t g_TM;

/* 下控进测：校验通过后若帧头为 AA BB 09 FF FF（CID=收方09，FC/SFC=FF）即 TM_STATE_TEST。
 * 短按遥控：一半→另一半→全亮→…；全部键事件在测试态内吞掉。 */
static uint8_t s_cmd_test_sel;

/* Accumulated distance for the current run session (metres). */
static float    s_accum_dist;
/* Stop animation state. */
static uint16_t s_stop_start_spd;    /* speed captured at STOPPING entry   */
static uint16_t s_stop_ramp_cnt;     /* proportional ramp length (ticks)   */
static uint16_t s_stop_disp_step;    /* 每 100 ms 显示递减量（0.1 km/h），Enter_Stopping 内按 ramp 均摊 */
static uint16_t s_stop_wait_cnt;     /* ticks waiting for CMD_STOP_OVER    */
static bool     s_stop_motor_done;   /* motor confirmed stopped (flag)      */
/* 开停进减速在 Enter 内已响，抑制 Keypad 对同一次 PRESS 的重复蜂鸣 */
static _Bool    s_suppress_keypad_beep;

/* 下控 FC01/00 中 Data1 大端档位（0.1 km/h）；data[2]=运行状态 */
static uint16_t s_lower_spd_fbk;
static uint8_t  s_lower_status_fbk;
/* 限制最大、最小挡位：接收下控上发的最大/最小挡位（0.1 km/h），约束显示板目标速度与按键调节范围。
 * Data3/Data4 全 0 表示本帧未携带边界，保持当前有效值；上电初值＝TM_SPEED_MAX / TM_SPEED_MIN。 */
static uint16_t s_spd_eff_max;
static uint16_t s_spd_eff_min;
/* 启动：待「已跑」确认（状态9 或 反馈速≥起步），超时补发一轮 RUN+档位 仅一次 */
static uint8_t  s_start_ack_pending;
static uint8_t  s_start_ack_wait_100ms;
static uint8_t  s_start_retried;
/* 停机：待反馈速归零，超时补发 Send_Stop 仅一次 */
static uint8_t  s_stop_ack_pending;
static uint8_t  s_stop_ack_wait_100ms;
static uint8_t  s_stop_retried;
/* 反馈速已为 0 至回待机之间：状态非待机时的宽限计数（100ms） */
static uint8_t  s_stop_spd0_grace_100ms;

/* Forward declarations for functions defined later in this file. */
static void Data_Reset(void);
static void Enter_Stopping(void);
static void Stop_FinishToIdle(void);
static void Display_Update(void);


/*=============================================================================
 * §2  Communication — send primitives
 *===========================================================================*/

static void Send_Ack0(void)
{
    AeroLink_Send(FC_HEARTBEAT, 0x00u, (uint8_t *)0);
}

/* 档位：Data1 = 目标档位 16 位，协议约定 10 = 1.0（与 TM 内部 0.1 km/h 一致） */
static void Send_GearWord(uint16_t spd01)
{
    uint8_t d[8] = {0};

    d[0] = (uint8_t)(spd01 >> 8);
    d[1] = (uint8_t)(spd01 & 0xFFu);
    AeroLink_Send(FC_GEAR, 0x00u, d);
}

static void Send_Speed(uint16_t spd)
{
    Send_GearWord(spd);
}

static void Send_Stop(void)
{
    AeroLink_Send(FC_CONTROL, 0x01u, (uint8_t *)0);
    AeroLink_Send(FC_CONTROL, 0x01u, (uint8_t *)0);
}

static void Send_Run(uint16_t speed01)
{
    uint8_t i;

    for (i = 0u; i < 3u; i++) {
        AeroLink_Send(FC_CONTROL, 0x00u, (uint8_t *)0);
    }
    for (i = 0u; i < 3u; i++) {
        Send_GearWord(speed01);
    }
}


/*=============================================================================
 * §3  Boot UART burst
 *     门控后连续发两次标准停机 (FC_CONTROL 停)，与旧工程「两帧停机」意图一致。
 *===========================================================================*/

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t d0;
    uint8_t d1;
} CfgRow_t;

static const CfgRow_t s_cfg_table[] = {
    { CMD_STATUS, 1u, STATUS_MOTOR_STOP, 0u },
    { CMD_STATUS, 1u, STATUS_MOTOR_STOP, 0u },
};
enum { CFG_TABLE_ROWS = 2 };
#define CFG_TABLE_LEN  ((uint8_t)CFG_TABLE_ROWS)

static uint8_t  s_cfg_step;
static uint16_t s_cfg_delay_ticks;
static uint8_t  s_cfg_gate_250ms;
/* Boot full-on phase (100 ms steps); < TM_BOOT_FULL_TICKS → block keys. */
static uint16_t s_boot_100ms;

/* Returns true once all config rows have been sent and gate delays elapsed. */
_Bool Treadmill_LowerBoardReady(void)
{
    return (s_cfg_gate_250ms >= TM_LOWER_CFG_250MS_GATE &&
            s_cfg_delay_ticks == 0u &&
            s_cfg_step >= CFG_TABLE_LEN) ? 1 : 0;
}

static void CfgRow_Send(const CfgRow_t *r)
{
    (void)r;
    /* 与旧逻辑一致：表内每行对应一帧「停」，非 Send_Stop 的双发 */
    AeroLink_Send(FC_CONTROL, 0x01u, (uint8_t *)0);
}

/* 100 ms: in BOOT/IDLE/SETTING; sends all rows in one burst when gate+delay elapsed. */
static void BootCfg_Poll(void)
{
    if (s_cfg_gate_250ms < TM_LOWER_CFG_250MS_GATE) return;
    if (s_cfg_step >= CFG_TABLE_LEN) return;

    if (s_cfg_delay_ticks > 0u) {
        s_cfg_delay_ticks--;
        return;
    }

    while (s_cfg_step < CFG_TABLE_LEN) {
        CfgRow_Send(&s_cfg_table[s_cfg_step++]);
    }
}


/*=============================================================================
 * §4  Communication — link watchdog & frame dispatch
 *===========================================================================*/

/* 1 s UART idle watchdog：无下控有效帧累加秒数，见 TM_LINK_LOST_TIMEOUT_SEC。 */
static uint8_t s_uart_idle_sec;
static uint8_t s_uart_rx_pulse;

/* 每 250 ms：上发 FC_HEARTBEAT(01/00,数据全0)。E01 仅由 Task_1000ms 按「连续无收包秒数」判定。 */
void Treadmill_On250msTasks(void)
{
    /* Advance lower-board gate from power-on (includes BOOT 全显) */
    if (s_cfg_gate_250ms < TM_LOWER_CFG_250MS_GATE) {
        s_cfg_gate_250ms++;
        if (s_cfg_gate_250ms == TM_LOWER_CFG_250MS_GATE) {
            s_cfg_delay_ticks = TM_LOWER_CFG_POST_GATE_100MS;
        }
    }

    if (g_TM.state == TM_STATE_ERROR) {
        return;
    }
    if (g_TM.state == TM_STATE_BOOT) {
        return; /* 全显期间不抢总线 */
    }
    if (s_cfg_step < CFG_TABLE_LEN && s_cfg_gate_250ms >= TM_LOWER_CFG_250MS_GATE) {
        return;
    }

    Send_Ack0();
}

/* 每 1 s：自下控上次有效帧起满 TM_LINK_LOST_TIMEOUT_SEC 秒 → E01（与老工程 er01=通信一致）。 */
void Treadmill_On1000msUartWatchdog(void)
{
    if (g_TM.state == TM_STATE_BOOT) {
        return;
    }

    if (s_uart_rx_pulse) {
        s_uart_rx_pulse = 0u;
        s_uart_idle_sec = 0u;
    } else {
        if (s_uart_idle_sec < 250u) {
            s_uart_idle_sec++;
        }
    }

    if (s_uart_idle_sec >= TM_LINK_LOST_TIMEOUT_SEC &&
        Treadmill_LowerBoardReady() &&
        g_TM.state != TM_STATE_ERROR) {
        g_TM.state      = TM_STATE_ERROR;
        g_TM.error_code = 1u;
        Beep_Play(BEEP_ALARM);
    }
}

/* 下控 FC01/SFC01 一级/二级故障 → 上显 E 码（与协议对照表一致） */
static uint8_t MapLowerFaultToDisplayCode(uint16_t err_lv1, uint8_t err_lv2)
{
    /* 二级：1=串口丢失→E01；2=过温→E02 */
    if (err_lv2 & 0x01u) {
        return 1u;
    }
    /* 一级：1,2,4,8 母线/12V 电压类→E05 */
    if (err_lv1 & (1u | 2u | 4u | 8u)) {
        return 5u;
    }
    /* 16,32,64 过流/堵转→E03 */
    if (err_lv1 & (16u | 32u | 64u)) {
        return 3u;
    }
    /* 128,512 速度过冲/MOS→E08 */
    if (err_lv1 & (128u | 512u)) {
        return 8u;
    }
    /* 256,1024 反转/缺相→E02；二级过温亦→E02 */
    if ((err_lv1 & (256u | 1024u)) != 0u || (err_lv2 & 0x02u) != 0u) {
        return 2u;
    }
    return 0u;
}

/* 将 g_TM.speed 约束在下控允许的挡位范围内；运行中超限则补发档位。停机递减段不干预。 */
static void Clamp_DisplaySpeedToLowerLimits(void)
{
    if (g_TM.state == TM_STATE_STOPPING) {
        return;
    }
    if (g_TM.speed > s_spd_eff_max) {
        g_TM.speed = s_spd_eff_max;
        UI_MarkDirty();
        if (g_TM.state == TM_STATE_RUNNING) {
            Send_Speed(g_TM.speed);
        }
        return;
    }
    if (g_TM.speed != 0u && g_TM.speed < s_spd_eff_min) {
        g_TM.speed = s_spd_eff_min;
        UI_MarkDirty();
        if (g_TM.state == TM_STATE_RUNNING) {
            Send_Speed(g_TM.speed);
        }
    }
}

/* max_gear / min_gear：运行信息 FC01/00 的 Data3/Data4（大端），或故障 FC01/01 的 Data3/Data4（协议最大/最小速度）。
 * 二者全 0：本帧不更新边界（兼容旧下控）；任一非 0 则解析，缺省的一侧用编译时 TM_SPEED_MIN/MAX。 */
static void Lower_ApplyReportedGearLimits(uint16_t max_gear, uint16_t min_gear)
{
    uint16_t hi;
    uint16_t lo;

    if (max_gear == 0u && min_gear == 0u) {
        return;
    }

    hi = (max_gear != 0u) ? max_gear : TM_SPEED_MAX;
    lo = (min_gear != 0u) ? min_gear : TM_SPEED_MIN;
    if (hi > TM_SPEED_MAX) {
        hi = TM_SPEED_MAX;
    }
    if (lo < TM_SPEED_MIN) {
        lo = TM_SPEED_MIN;
    }
    if (lo > hi) {
        return;
    }
    s_spd_eff_max = hi;
    s_spd_eff_min = lo;
    Clamp_DisplaySpeedToLowerLimits();
}

/* 下控「进测」：线序 AA BB 09 FF FF，与 17 字节帧中 CID=0x09、FC=0xFF、SFC=0xFF 一致（校验由链路层完成） */
static _Bool prv_uart_rx_is_factory_test(const AeroFrame_t *f)
{
    return (f->cid == TREAD_CID_RECV && f->fc == 0xFFu && f->sfc == 0xFFu);
}

/* UART frame received callback（校验通过后进入；任一下控帧均视为链路存活）。 */
void AeroLink_OnFrameReceived(AeroFrame_t *f)
{
    s_uart_rx_pulse = 1u;

    /* 故障信息：FC 0x01 / SFC 0x01，Data1 一级、Data2 二级；Data3/Data4 大端为最大/最小速度（与档位数制一致时复用） */
    if (f->fc == FC_HEARTBEAT && f->sfc == 0x01u) {
        uint16_t err_lv1 = ((uint16_t)f->data[0] << 8) | (uint16_t)f->data[1];
        uint8_t  err_lv2 = f->data[3];
        uint16_t lmx     = ((uint16_t)f->data[4] << 8) | (uint16_t)f->data[5];
        uint16_t lmn     = ((uint16_t)f->data[6] << 8) | (uint16_t)f->data[7];

        Lower_ApplyReportedGearLimits(lmx, lmn);

        uint8_t  code     = MapLowerFaultToDisplayCode(err_lv1, err_lv2);

        if (code != 0u) {
            /* 产测期间：不报下控故障 E 码（仍靠 1s 看门狗报通信 E01） */
            if (g_TM.state == TM_STATE_TEST) {
                return;
            }
            bool was_error = (g_TM.state == TM_STATE_ERROR);

            g_TM.state = TM_STATE_ERROR;
            if (g_TM.error_code == 0u) {
                g_TM.error_code = code;
            }
            if (!was_error) {
                Beep_Play(BEEP_ALARM);
            }
        }
        return;
    }

    /* 运行信息：FC 0x01 / SFC 0x00，Data1=档位反馈（大端），Data2=运行状态；Data3/Data4=最大/最小挡位（大端） */
    if (f->fc == FC_HEARTBEAT && f->sfc == 0x00u) {
        uint16_t spd_word = ((uint16_t)f->data[0] << 8) | (uint16_t)f->data[1];
        uint8_t  status   = f->data[2];
        uint16_t gmax     = ((uint16_t)f->data[4] << 8) | (uint16_t)f->data[5];
        uint16_t gmin     = ((uint16_t)f->data[6] << 8) | (uint16_t)f->data[7];

        s_lower_spd_fbk    = spd_word;
        s_lower_status_fbk = status;

        Lower_ApplyReportedGearLimits(gmax, gmin);

        if (g_TM.state == TM_STATE_BOOT || g_TM.state == TM_STATE_ERROR) {
            return;
        }
        if (g_TM.state == TM_STATE_TEST) {
            return;
        }

        switch (status) {
            case TM_LOWER_STATUS_RUNNING:
                /* 倒计时阶段不下控抢跑进入 RUNNING，否则 3-2-1/倒计时显示错乱 */
                if (g_TM.state != TM_STATE_RUNNING &&
                    g_TM.state != TM_STATE_STOPPING &&
                    g_TM.state != TM_STATE_COUNTDOWN) {
                    g_TM.state = TM_STATE_RUNNING;
                    s_start_ack_pending     = 0u;
                    s_start_retried         = 0u;
                    s_start_ack_wait_100ms  = 0u;
                }
                break;
            case TM_LOWER_STATUS_IDLE:
                /* 避免 status 先到、速度反馈仍非零时误清（显示已停电机仍转） */
                if (s_lower_spd_fbk > 0u) {
                    break;
                }
                if (g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_STOPPING) {
                    Stop_FinishToIdle();
                } else if (g_TM.state == TM_STATE_COUNTDOWN) {
                    /* 下控停：速度视为 0，退出倒计时回待机 */
                    g_TM.timer_100ms = 0u;
                    s_lower_spd_fbk  = 0u;
                    Data_Reset();
                    g_TM.state = TM_STATE_IDLE;
                    UI_MarkDirty();
                }
                break;
            case 3u:
                if (g_TM.state != TM_STATE_TEST) {
                    g_TM.state = TM_STATE_ERROR;
                }
                break;
            default: break;
        }
        return;
    }

    if (prv_uart_rx_is_factory_test(f)) {
        if (g_TM.state == TM_STATE_BOOT) {
            g_TM.timer_100ms = 0u;
            s_boot_100ms     = 0u;
        }
        if (g_TM.state != TM_STATE_TEST) {
            Beep_Play(BEEP_CLICK);
            g_TM.state = TM_STATE_TEST;
            s_cmd_test_sel = 0u;
        }
        UI_MarkDirty();
        return;
    }
}


/*=============================================================================
 * §5  Error handling
 *===========================================================================*/

static uint8_t s_err_stop_sent;   /* stop command sent flag for current error */

/* On first entry to ERROR state: send stop and reset the flag. */
static void Error_OnEntry(void)
{
    s_err_stop_sent = 0u;
}

static void Error_Poll(void)
{
    if (s_err_stop_sent == 0u) {
        Send_Stop();
        s_err_stop_sent = 1u;
    }
}


/*=============================================================================
 * §6  Key input & target adjustment
 *===========================================================================*/

/* Returns the effective step multiplier (minimum 1). */
static uint32_t Step_Mult(bool is_hold, uint32_t hold_mult)
{
    uint32_t m = is_hold ? hold_mult : 1u;
    return (m == 0u) ? 1u : m;
}

/* Adjusts the currently displayed target by dir (+1 / -1). Clamps to min/max. */
static void Adjust_Target(int8_t dir, bool is_hold)
{
    g_TM.steady_timer = 20u;
    UI_MarkDirty();

    switch (g_TM.disp_index) {

        case DISP_SPEED: {
            uint32_t delta = (uint32_t)TM_SPEED_STEP *
                             Step_Mult(is_hold, TM_SPEED_HOLD_STEP_MULT);
            if (dir > 0) {
                if (g_TM.speed >= s_spd_eff_max) break; /* 已在上限，Keypad 侧可再响一声 */
                uint32_t ns = (uint32_t)g_TM.speed + delta;
                g_TM.speed  = (ns > s_spd_eff_max) ? s_spd_eff_max : (uint16_t)ns;
                /* 蜂鸣在 Send_Speed 之前：避免 UART 阻塞导致节拍打不满。
                 * 短按每次有效加减响；长按仅到达最高/最低档时各提示一声。 */
                if (!is_hold || g_TM.speed == s_spd_eff_max) {
                    Beep_Play(BEEP_CLICK);
                    s_suppress_keypad_beep = 1;
                }
            } else {
                if (g_TM.speed <= s_spd_eff_min) break;
                int32_t ns = (int32_t)g_TM.speed - (int32_t)delta;
                g_TM.speed = (ns < s_spd_eff_min) ? s_spd_eff_min : (uint16_t)ns;
                if (!is_hold || g_TM.speed == s_spd_eff_min) {
                    Beep_Play(BEEP_CLICK);
                    s_suppress_keypad_beep = 1;
                }
            }
            if (g_TM.state == TM_STATE_RUNNING) Send_Speed(g_TM.speed);
            break;
        }

        case DISP_TIME: {
            uint32_t delta = (uint32_t)TM_TIME_STEP *
                             Step_Mult(is_hold, TM_TIME_HOLD_STEP_MULT);
            if (dir > 0) {
                if (g_TM.time >= TM_TIME_MAX) { g_TM.time = TM_TIME_MIN; break; }
                uint32_t ns = g_TM.time + delta;
                g_TM.time   = (ns > TM_TIME_MAX) ? TM_TIME_MAX : ns;
            } else {
                if (g_TM.time <= TM_TIME_MIN) { g_TM.time = TM_TIME_MAX; break; }
                g_TM.time = (g_TM.time - TM_TIME_MIN < delta) ? TM_TIME_MIN
                            : g_TM.time - delta;
            }
            break;
        }

        case DISP_DIST: {
            float delta = TM_DIST_STEP * (float)Step_Mult(is_hold, TM_DIST_HOLD_STEP_MULT);
            if (dir > 0) {
                if (g_TM.dist >= TM_DIST_MAX) { g_TM.dist = TM_DIST_MIN; break; }
                float nd = g_TM.dist + delta;
                g_TM.dist = (nd > TM_DIST_MAX) ? TM_DIST_MAX : nd;
            } else {
                if (g_TM.dist <= TM_DIST_MIN) { g_TM.dist = TM_DIST_MAX; break; }
                float nd = g_TM.dist - delta;
                g_TM.dist = (nd < TM_DIST_MIN) ? TM_DIST_MIN : nd;
            }
            break;
        }

        case DISP_CAL: {
            float delta = TM_CAL_STEP * (float)Step_Mult(is_hold, TM_CAL_HOLD_STEP_MULT);
            if (dir > 0) {
                if (g_TM.cal >= TM_CAL_MAX) { g_TM.cal = TM_CAL_MIN; break; }
                float nc = g_TM.cal + delta;
                g_TM.cal = (nc > TM_CAL_MAX) ? TM_CAL_MAX : nc;
            } else {
                if (g_TM.cal <= TM_CAL_MIN) { g_TM.cal = TM_CAL_MAX; break; }
                float nc = g_TM.cal - delta;
                g_TM.cal = (nc < TM_CAL_MIN) ? TM_CAL_MIN : nc;
            }
            break;
        }

        default: break;
    }
}

_Bool Treadmill_CmdTestDisplayActive(void)
{
    return (g_TM.state == TM_STATE_TEST) ? 1 : 0;
}

/* Key event dispatcher.  Returns true if the event was consumed. */
bool Treadmill_On_Event(uint8_t key_id, uint8_t evt)
{
    /* 出厂测：吞掉 PRESS/HOLD/RELEASE，禁止遥控进入倒计时/设置/运行等其它界面 */
    if (g_TM.state == TM_STATE_TEST) {
        (void)key_id;
        if (evt == K_EVT_PRESS) {
            if (s_cmd_test_sel == 0u) {
                s_cmd_test_sel = 1u;
            } else if (s_cmd_test_sel >= 3u) {
                s_cmd_test_sel = 1u;
            } else {
                s_cmd_test_sel++;
            }
            UI_MarkDirty();
            Display_Update();
            UI_Engine_Refresh();
        }
        return true;
    }

    if (evt == K_EVT_RELEASE) return false;
    if ((key_id == K_ID_MODE || key_id == K_ID_ONOFF) && evt == K_EVT_HOLD) return false;

    /* BOOT: 全显、可选版本行 — 整段不响应按键，结束后由 Render_Boot 进 IDLE。 */
    if (g_TM.state == TM_STATE_BOOT) {
        return true;
    }

    switch (g_TM.state) {

        case TM_STATE_IDLE:
            if (key_id == K_ID_UP || key_id == K_ID_DN) return false;
            if (key_id == K_ID_ONOFF) {
                if (!Treadmill_LowerBoardReady()) { return true; }
                g_TM.state       = TM_STATE_COUNTDOWN;
                g_TM.timer_100ms = 30u;
            } else if (key_id == K_ID_MODE) {
                g_TM.state       = TM_STATE_SETTING;
                g_TM.disp_index  = DISP_TIME;
                g_TM.target_mode = TARGET_TIME;
                g_TM.time        = TM_TIME_SET;
            }
            break;

        case TM_STATE_SETTING:
            if (key_id == K_ID_MODE) {
                if      (g_TM.disp_index == DISP_TIME) {
                    g_TM.time = 0u;
                    g_TM.disp_index = DISP_DIST;  g_TM.target_mode = TARGET_DIST;
                    g_TM.dist = TM_DIST_SET;
                } else if (g_TM.disp_index == DISP_DIST) {
                    g_TM.dist = 0.0f;
                    g_TM.disp_index = DISP_CAL;   g_TM.target_mode = TARGET_CAL;
                    g_TM.cal  = TM_CAL_SET;
                } else {
                    /* 退出设置回待机：速度/时间/路程/卡路里全部清 0 */
                    Data_Reset();
                    g_TM.state = TM_STATE_IDLE;
                    g_TM.timer_100ms = 0u;
                }
            } else if (key_id == K_ID_UP || key_id == K_ID_DN) {
                Adjust_Target(key_id == K_ID_UP ? 1 : -1, evt == K_EVT_HOLD);
            } else if (key_id == K_ID_ONOFF) {
                if (!Treadmill_LowerBoardReady()) { return true; }
                g_TM.state       = TM_STATE_COUNTDOWN;
                g_TM.timer_100ms = 30u;
            }
            break;

        case TM_STATE_COUNTDOWN:
            /* 3-2-1 期间按 ON/OFF 取消，回待机并清状态（与待机全 0 一致） */
            if (key_id == K_ID_ONOFF) {
                Data_Reset();
                g_TM.state = TM_STATE_IDLE;
                g_TM.timer_100ms = 0u;
            }
            break;

        case TM_STATE_RUNNING:
            if (key_id == K_ID_MODE) return false;
            if (key_id == K_ID_ONOFF) {
                Enter_Stopping();
                s_suppress_keypad_beep = 1; /* 与上面 Enter 内蜂鸣去重，勿在 Keypad 再响 */
            } else if (key_id == K_ID_UP || key_id == K_ID_DN) {
                g_TM.disp_index  = DISP_SPEED;
                g_TM.cycle_timer = 0u;
                Adjust_Target(key_id == K_ID_UP ? 1 : -1, evt == K_EVT_HOLD);
            }
            break;

        case TM_STATE_STOPPING:
            return false;

        default: break;
    }

    UI_MarkDirty();
    return true;
}


/*=============================================================================
 * §7  Running state — countdown / run / stop / statistics
 *===========================================================================*/

/* Enter stopping state.  timer_100ms = ramp ticks (max Phase-1 wait).
 * 显示倒计时：单调线性递减（见 Stop_Tick），步长 = 起始速度/ramp，与 TM_STOP_TIME_COUNT 比例一致。 */
static void Enter_Stopping(void)
{
    if (g_TM.state == TM_STATE_TEST) {
        return;
    }
    /* 在 Send_Stop/串口 之前响，避免主循环被占用导致蜂鸣 1ms 节拍打不满、听成「一截或很弱」 */
    Beep_Play(BEEP_CLICK);
    s_stop_start_spd  = g_TM.speed;
    uint32_t ramp     = (uint32_t)TM_STOP_TIME_COUNT * g_TM.speed / TM_SPEED_MAX;
    s_stop_ramp_cnt   = (ramp < 2u) ? 2u : (uint16_t)ramp;
    /* 与 ALR-C6 boot_time 一致：停机速度显示为单调倒计时（不跟下控上跳）。
     * 每步减量按 ramp 总拍均摊，使总时长与 TM_STOP_TIME_COUNT 比例设计一致。 */
    if (s_stop_start_spd > 0u && s_stop_ramp_cnt > 0u) {
        s_stop_disp_step = s_stop_start_spd / s_stop_ramp_cnt;
        if (s_stop_disp_step == 0u) {
            s_stop_disp_step = 1u;
        }
    } else {
        s_stop_disp_step = 1u;
    }
    s_stop_wait_cnt   = 0u;
    s_stop_motor_done = false;
    s_stop_ack_pending       = 1u;
    s_stop_retried           = 0u;
    s_stop_ack_wait_100ms    = 0u;
    s_stop_spd0_grace_100ms  = 0u;
    g_TM.timer_100ms  = s_stop_ramp_cnt;
    g_TM.state        = TM_STATE_STOPPING;
    Send_Stop();
}

static void Data_Reset(void)
{
    g_TM.speed        = 0u;
    g_TM.time         = 0u;
    g_TM.dist         = 0.0f;
    g_TM.cal          = 0.0f;
    g_TM.target_mode  = TARGET_NONE;
    g_TM.disp_index   = DISP_TIME;
    g_TM.steady_timer = 0u;
    g_TM.cycle_timer  = 0u;
    UI_ClearAll();
}

/* 停机结束：立刻清显示、回待机（不再额外等 TM_STOP_SAFETY_TIMEOUT） */
static void Stop_FinishToIdle(void)
{
    if (g_TM.state == TM_STATE_TEST) {
        return;
    }
    s_stop_motor_done        = false;
    s_stop_wait_cnt          = 0u;
    s_stop_ack_pending       = 0u;
    s_stop_retried           = 0u;
    s_stop_ack_wait_100ms    = 0u;
    s_stop_spd0_grace_100ms  = 0u;
    g_TM.timer_100ms         = 0u;
    Data_Reset();
    g_TM.state = TM_STATE_IDLE;
    UI_MarkDirty();
}

/* Countdown: 3-2-1 beep, then launch the motor. */
static void Countdown_Tick(void)
{
    if (g_TM.timer_100ms > 0u) {
        g_TM.timer_100ms--;
        if (g_TM.timer_100ms == 20u || g_TM.timer_100ms == 10u) {
            Beep_Play(BEEP_CLICK);
        }
    } else {
        g_TM.speed       = s_spd_eff_min;
        g_TM.state       = TM_STATE_RUNNING;
        g_TM.cycle_timer = 0u;
        g_TM.disp_index  = DISP_SPEED;
        s_accum_dist     = 0.0f;
        s_start_ack_pending    = 1u;
        s_start_retried        = 0u;
        s_start_ack_wait_100ms = 0u;
        if (g_TM.target_mode != TARGET_TIME) g_TM.time = 0u;
        if (g_TM.target_mode != TARGET_DIST) g_TM.dist = 0.0f;
        if (g_TM.target_mode != TARGET_CAL)  g_TM.cal  = 0.0f;
        Send_Run(g_TM.speed);
        Send_Speed(g_TM.speed);
    }
}

/* RUNNING：若刚发启动，等待下控 status=运行 或 反馈速≥起步；超时未变则补发一轮 RUN+档位（仅一次） */
static void LowerStartAck100ms(void)
{
    if (g_TM.state != TM_STATE_RUNNING || s_start_ack_pending == 0u) {
        return;
    }

    if (s_lower_status_fbk == TM_LOWER_STATUS_RUNNING || s_lower_spd_fbk >= s_spd_eff_min) {
        s_start_ack_pending    = 0u;
        s_start_retried        = 0u;
        s_start_ack_wait_100ms = 0u;
        return;
    }

    if (s_start_retried != 0u) {
        return;
			}	

    if (++s_start_ack_wait_100ms < TM_LOWER_CMD_ACK_WAIT_TICKS) {
        return;
    }

    s_start_ack_wait_100ms = 0u;
    Send_Run(g_TM.speed);
    Send_Speed(g_TM.speed);
    s_start_retried = 1u;
}

/* Running: update stats every 1 s (10 × 100 ms ticks). */
static void Run_Tick(void)
{
    static uint8_t s_sec_cnt = 0u;

    LowerStartAck100ms();

    /* Display auto-cycle every 5 s when user is not adjusting. */
#if (LED_DRIVER_TYPE != LED_DRIVER_TM1640)
    if (g_TM.steady_timer == 0u && ++g_TM.cycle_timer >= 50u) {
        g_TM.cycle_timer = 0u;
        g_TM.disp_index  = (TM_DispIndex_t)((g_TM.disp_index + 1u) % 4u);
    }
#endif

    if (++s_sec_cnt < 10u) return;
    s_sec_cnt = 0u;

    /* Distance increment for this second (m): speed[0.1km/h] / 36 = m/s */
    float dist_inc = (float)g_TM.speed / 36.0f;
    s_accum_dist  += dist_inc;

    bool stop_now = false;

    /* Time */
    if (g_TM.target_mode == TARGET_TIME) {
        if (g_TM.time > 0u) g_TM.time--;
        if (g_TM.time == 0u) stop_now = true;
    } else {
        g_TM.time++;
        if (g_TM.time >= TM_TIME_MAX) stop_now = true;
    }

    /* Distance */
    if (g_TM.target_mode == TARGET_DIST) {
        g_TM.dist -= dist_inc;
        if (g_TM.dist <= 0.0f) stop_now = true;
    } else {
        g_TM.dist = s_accum_dist;
    }

    /* Calories */
    if (g_TM.target_mode == TARGET_CAL) {
        g_TM.cal -= dist_inc * (float)CAL_PER;
        if (g_TM.cal <= 0.0f) stop_now = true;
    } else {
        g_TM.cal = s_accum_dist * (float)CAL_PER;
    }

    if (stop_now) {
        /* 蜂鸣在 Enter_Stopping 内(先于 UART)，与手动按开停同一路径、避免到顶+按键叠两声 */
        Enter_Stopping();
    }
}

/* Phase1: fbk>0 递减显示；Phase2: 反馈速=0 — 仅当下控状态=待机(TM_LOWER_STATUS_IDLE)才立刻清 0 回待机；
 * 否则宽限 TM_STOP_SPD0_GRACE_TICKS 或 TM_STOP_SAFETY_TIMEOUT，期间显示保持（可为 0），不强行抬成 0.1。 */
static void Stop_Tick(void)
{
    if (s_lower_spd_fbk > 0u) {
        s_stop_spd0_grace_100ms = 0u;
        if (s_stop_ack_pending != 0u && s_stop_retried == 0u) {
            if (++s_stop_ack_wait_100ms >= TM_LOWER_CMD_ACK_WAIT_TICKS) {
                s_stop_ack_wait_100ms = 0u;
                Send_Stop();
                s_stop_retried = 1u;
            }
        }
        uint16_t fbk_c = (s_lower_spd_fbk < s_stop_start_spd)
                             ? s_lower_spd_fbk
                             : s_stop_start_spd;
        uint16_t dec   = s_stop_disp_step;
        if (dec == 0u) {
            dec = 1u;
        }
        uint16_t next;
        if (g_TM.speed > dec) {
            next = (uint16_t)(g_TM.speed - dec);
        } else {
            next = 0u;
        }
        if (fbk_c < next) {
            next = fbk_c;
        }
        g_TM.speed = next;
        if (g_TM.timer_100ms > 0u) {
            g_TM.timer_100ms--;
        }
        UI_MarkDirty();
    } else {
        s_stop_ack_pending    = 0u;
        s_stop_retried        = 0u;
        s_stop_ack_wait_100ms = 0u;

        if (s_lower_status_fbk == TM_LOWER_STATUS_IDLE) {
            Stop_FinishToIdle();
            return;
        }

        if (++s_stop_wait_cnt >= TM_STOP_SAFETY_TIMEOUT) {
            Stop_FinishToIdle();
            return;
        }

        if (s_stop_spd0_grace_100ms < TM_STOP_SPD0_GRACE_TICKS) {
            s_stop_spd0_grace_100ms++;
            return;
        }

        Stop_FinishToIdle();
    }
}


/*=============================================================================
 * §8  Display rendering
 *===========================================================================*/

/* Indicator LED grid indices (TM1620 / TM1652). */
#define GRID_IND    4u
#define LED_TIME    0u
#define LED_SPEED   1u
#define LED_DIS     2u
#define LED_CAL     3u

/* Colon segment location. */
#define GRID_COLON  0u
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
  #define SEG_COLON_H  7u
  #define SEG_COLON_L  7u
#else
  #define SEG_COLON_H  1u
  #define SEG_COLON_L  1u
#endif

/* Converts g_TM.time to minutes for 0:mm display. */
static uint16_t Time_To_Minutes(void)
{
    uint32_t t = g_TM.time;
    /* Ceil remaining seconds when counting down. */
    if (g_TM.target_mode == TARGET_TIME &&
        (g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_STOPPING)) {
        t = (t == 0u) ? 0u : (t + 59u) / 60u;
    } else {
        t = t / 60u;
    }
    return (t > 99u) ? 99u : (uint16_t)t;
}

/* Converts g_TM.dist (metres) to tenths of km for xx.x display. */
static uint16_t Dist_To_Tenths(void)
{
    float d = (g_TM.dist < 0.0f) ? 0.0f : g_TM.dist;
    uint32_t dk = (uint32_t)(d / 100.0f);   /* 100 m = 0.1 km */
    return (uint16_t)((dk > 990u) ? 990u : dk);
}

/* ---- TM1640 full-screen renderer (4 simultaneous zones) ---- */
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)

#define Z_TIME   0u
#define Z_CAL    3u
#define Z_SPEED  6u
#define Z_DIST   9u

static void Render_TM1640(void)
{
    static TM_State_t s_prev = TM_STATE_BOOT;
    if (g_TM.state != s_prev) { UI_ClearAll(); s_prev = g_TM.state; }

    UI_SetBlinkTime(1000u);
    bool blink = (g_TM.state == TM_STATE_SETTING && g_TM.steady_timer == 0u);
    for (uint8_t i = 0u; i < 3u; i++) {
        UI_SetGridBlinkMask(Z_TIME  + i, blink && (g_TM.disp_index == DISP_TIME));
        UI_SetGridBlinkMask(Z_CAL   + i, blink && (g_TM.disp_index == DISP_CAL));
        UI_SetGridBlinkMask(Z_SPEED + i, blink && (g_TM.disp_index == DISP_SPEED));
        UI_SetGridBlinkMask(Z_DIST  + i, blink && (g_TM.disp_index == DISP_DIST));
    }

    switch (g_TM.state) {
        case TM_STATE_COUNTDOWN: {
            UI_ClearAll();
            uint8_t cd = (uint8_t)((g_TM.timer_100ms + 9u) / 10u);
            if (cd < 1u) {
                cd = 1u;
            }
            UI_SetNumber(Z_SPEED + 1u, cd, 1, 0, ZERO_HIDE);
            break;
        }
        case TM_STATE_IDLE:
        case TM_STATE_SETTING:
        case TM_STATE_RUNNING:
        case TM_STATE_STOPPING:
            UI_SetNumber(Z_TIME,  Time_To_Minutes(),                     3, 0, ZERO_SHOW);
            UI_SetNumber(Z_CAL,   (uint16_t)(g_TM.cal / 1000.0f),       3, 0, ZERO_SHOW);
            UI_SetNumber(Z_SPEED, g_TM.speed,                            3, 2, ZERO_SHOW);
            UI_SetNumber(Z_DIST,  Dist_To_Tenths(),                      3, 2, ZERO_SHOW);
            UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_BLINK);
            UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_BLINK);
            break;
        case TM_STATE_ERROR:
            UI_SetRaw(Z_SPEED, SEG_RAW(SEG_E));
            UI_SetNumber(Z_SPEED + 1u, g_TM.error_code, 2, 0, ZERO_SHOW);
            break;
        default: break;
    }
}

/* 出厂测灯：0=全显(仅数码段，同开机全显)；1=一半；2=另一半；3=全亮(+四灯+冒号)。 */
static void Render_CmdTest_Tm1640(void)
{
#define TCT_ZT   0u
#define TCT_ZC   3u
#define TCT_ZS   6u
#define TCT_ZD   9u
#define TCT_IGR  11u
    uint8_t s;
    uint8_t i;
    UI_SetBlinkTime(0u);
    UI_ClearAll();
    for (s = 0u; s < 3u; s++) {
        UI_SetGridBlinkMask(TCT_ZT + s, 0u);
        UI_SetGridBlinkMask(TCT_ZC + s, 0u);
        UI_SetGridBlinkMask(TCT_ZS + s, 0u);
        UI_SetGridBlinkMask(TCT_ZD + s, 0u);
    }
    for (s = 0u; s < 6u; s++) {
        UI_SetBitMode(TCT_IGR, s, DISP_OFF);
    }
    if (s_cmd_test_sel == 0u) {
        for (i = 0u; i < (uint8_t)UI_GRID_MAX; i++) {
            UI_SetRaw(i, 0xFFu);
        }
    } else if (s_cmd_test_sel == 3u) {
        for (i = 0u; i < (uint8_t)UI_GRID_MAX; i++) {
            UI_SetRaw(i, 0xFFu);
        }
        UI_SetBitMode(TCT_IGR, UI_IND_TIME_BIT, DISP_ON);
        UI_SetBitMode(TCT_IGR, UI_IND_SPEED_BIT, DISP_ON);
        UI_SetBitMode(TCT_IGR, UI_IND_DIS_BIT, DISP_ON);
        UI_SetBitMode(TCT_IGR, UI_IND_CAL_BIT, DISP_ON);
        UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_ON);
        UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_ON);
    } else if (s_cmd_test_sel == 1u) {
        for (i = 0u; i < 3u; i++) {
            UI_SetRaw((uint8_t)(TCT_ZT + i), 0xFFu);
            UI_SetRaw((uint8_t)(TCT_ZC + i), 0xFFu);
            UI_SetRaw((uint8_t)(TCT_ZS + i), 0u);
            UI_SetRaw((uint8_t)(TCT_ZD + i), 0u);
        }
    } else {
        for (i = 0u; i < 3u; i++) {
            UI_SetRaw((uint8_t)(TCT_ZT + i), 0u);
            UI_SetRaw((uint8_t)(TCT_ZC + i), 0u);
            UI_SetRaw((uint8_t)(TCT_ZS + i), 0xFFu);
            UI_SetRaw((uint8_t)(TCT_ZD + i), 0xFFu);
        }
        UI_SetBitMode(TCT_IGR, UI_IND_SPEED_BIT, DISP_ON);
        UI_SetBitMode(TCT_IGR, UI_IND_DIS_BIT, DISP_ON);
    }
#undef TCT_ZT
#undef TCT_ZC
#undef TCT_ZS
#undef TCT_ZD
#undef TCT_IGR
}
#endif /* LED_DRIVER_TM1640 */

/* ---- Single-zone renderer (TM1620 / TM1652) ---- */
static void Render_SingleZone(void)
{
    static TM_State_t     s_prev_st   = TM_STATE_BOOT;
    static TM_DispIndex_t s_prev_disp = DISP_SPEED;
    if (g_TM.state != s_prev_st || g_TM.disp_index != s_prev_disp) {
        UI_ClearAll();
        s_prev_st   = g_TM.state;
        s_prev_disp = g_TM.disp_index;
    }

    for (uint8_t i = 0u; i < 6u; i++) UI_SetBitMode(GRID_IND, i, DISP_OFF);

    bool blink = (g_TM.state == TM_STATE_SETTING && g_TM.steady_timer == 0u);
    UI_SetBlinkTime(1000u);
    for (uint8_t i = 0u; i < 4u; i++) UI_SetGridBlinkMask(i, blink);

    switch (g_TM.state) {

        case TM_STATE_IDLE:
            UI_SetNumber(0, 0, 4, 0, ZERO_SHOW);
            UI_SetBitMode(GRID_IND,   LED_TIME,    DISP_ON);
            UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_BLINK);
            UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_BLINK);
            break;

        case TM_STATE_COUNTDOWN: {
            uint8_t cd = (uint8_t)((g_TM.timer_100ms + 9u) / 10u);
            if (cd < 1u) {
                cd = 1u;
            }
            UI_SetNumber(2, cd, 1, 0, ZERO_HIDE);
            break;
        }

        case TM_STATE_STOPPING: {
            uint16_t spd = g_TM.speed;
            if (spd < 10u) UI_SetNumber(1, spd, 2, 2, ZERO_SHOW);
            else            UI_SetNumber(0, spd, 3, 2, ZERO_HIDE);
            UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
            break;
        }

        case TM_STATE_SETTING:
        case TM_STATE_RUNNING:
            switch (g_TM.disp_index) {
                case DISP_SPEED: {
                    uint16_t spd = g_TM.speed;
                    if (spd < 10u) UI_SetNumber(1, spd, 2, 2, ZERO_SHOW);
                    else            UI_SetNumber(0, spd, 3, 2, ZERO_HIDE);
                    UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
                    break;
                }
                case DISP_TIME:
                    UI_SetNumber(0, Time_To_Minutes(), 4, 0, ZERO_SHOW);
                    UI_SetBitMode(GRID_IND,   LED_TIME,    DISP_ON);
                    UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_BLINK);
                    UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_BLINK);
                    break;
                case DISP_DIST:
                    UI_SetNumber(0, Dist_To_Tenths(), 3, 2, ZERO_SHOW);
                    UI_SetBitMode(GRID_IND, LED_DIS, DISP_ON);
                    UI_SetBitMode(1, 7, blink ? DISP_BLINK : DISP_ON);
                    break;
                case DISP_CAL:
                    UI_SetNumber(0, (uint32_t)(g_TM.cal / 1000.0f), 4, 0, ZERO_HIDE);
                    UI_SetBitMode(GRID_IND, LED_CAL, DISP_ON);
                    break;
                default: break;
            }
            break;

        case TM_STATE_ERROR:
            UI_SetRaw(0, SEG_RAW(SEG_E));
            UI_SetNumber(1, g_TM.error_code, 2, 0, ZERO_SHOW);
            UI_SetRaw(3, 0u);
            break;

        default: break;
    }
}

#if (LED_DRIVER_TYPE != LED_DRIVER_TM1640)
/* TM1620/TM1652：0=全显(仅段)；1=一半(时/卡)；2=另一半(速/里)；3=全亮(+四灯+冒号) */
static void Render_CmdTest_Single(void)
{
    uint8_t i;
    UI_SetBlinkTime(0u);
    UI_ClearAll();
    for (i = 0u; i < 4u; i++) {
        UI_SetGridBlinkMask(i, 0u);
    }
    for (i = 0u; i < 6u; i++) {
        UI_SetBitMode(GRID_IND, i, DISP_OFF);
    }
    if (s_cmd_test_sel == 0u) {
        for (i = 0u; i < (uint8_t)UI_GRID_MAX; i++) {
            UI_SetRaw(i, 0xFFu);
        }
    } else if (s_cmd_test_sel == 3u) {
        for (i = 0u; i < (uint8_t)UI_GRID_MAX; i++) {
            UI_SetRaw(i, 0xFFu);
        }
        UI_SetBitMode(GRID_IND, LED_TIME, DISP_ON);
        UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
        UI_SetBitMode(GRID_IND, LED_DIS, DISP_ON);
        UI_SetBitMode(GRID_IND, LED_CAL, DISP_ON);
        UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_ON);
        UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_ON);
    } else if (s_cmd_test_sel == 1u) {
        for (i = 0u; i < (uint8_t)UI_GRID_MAX; i++) {
            UI_SetRaw(i, 0u);
        }
        UI_SetBitMode(GRID_IND, LED_TIME, DISP_ON);
        UI_SetBitMode(GRID_IND, LED_CAL, DISP_ON);
    } else {
        for (i = 0u; i < (uint8_t)UI_GRID_MAX; i++) {
            UI_SetRaw(i, 0u);
        }
        UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
        UI_SetBitMode(GRID_IND, LED_DIS, DISP_ON);
    }
}
#endif

/* ---- Boot display (shared by all LED driver types) ---- */
static void Render_Boot(void)
{
    if (s_boot_100ms == 0u) {
        Beep_Play(BEEP_CLICK);
    }

    if (s_boot_100ms < TM_BOOT_FULL_TICKS) {
        for (uint8_t i = 0u; i < UI_GRID_MAX; i++) {
            UI_SetRaw(i, 0xFFu);
        }
        s_boot_100ms++;
        return;
    }

#ifdef BOOT_SHOW_VERSION
    if (s_boot_100ms < (TM_BOOT_FULL_TICKS + 5u)) {
        if (s_boot_100ms == TM_BOOT_FULL_TICKS) {
            UI_ClearAll();
        }
        UI_SetRaw(0, SEG_RAW(SEG_U));
        UI_SetNumber(1, FW_VER_U, 2, 2, ZERO_SHOW);
        s_boot_100ms++;
        return;
    }
    if (s_boot_100ms < (TM_BOOT_FULL_TICKS + 15u)) {
        if (s_boot_100ms == (TM_BOOT_FULL_TICKS + 5u)) {
            UI_ClearAll();
        }
        UI_SetRaw(0, SEG_RAW(SEG_C));
        {
            uint16_t cv = (g_TM.lower_c_bcd != 0u) ? g_TM.lower_c_bcd : FW_VER_C;
            UI_SetNumber(1, cv, 2, 2, ZERO_SHOW);
        }
        s_boot_100ms++;
        return;
    }
#endif

    g_TM.state        = TM_STATE_IDLE;
    g_TM.timer_100ms  = 0u;
    s_boot_100ms      = 0u;
}

/* Top-level display dispatcher called every 100 ms. */
static void Display_Update(void)
{
    /* 测灯优先于开机全显(2s)，否则测灯在 BOOT 阶段被全显盖掉，2s 后像「自动切界面」 */
    if (g_TM.state == TM_STATE_TEST) {
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
        Render_CmdTest_Tm1640();
#else
        Render_CmdTest_Single();
#endif
        return;
    }
    if (g_TM.state == TM_STATE_BOOT) { Render_Boot(); return; }

#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
    Render_TM1640();
#else
    Render_SingleZone();
#endif
}


/*=============================================================================
 * §9  Public API
 *===========================================================================*/

void Treadmill_Init(void)
{
    g_TM.state        = TM_STATE_BOOT;
    g_TM.safety_key   = true;
    g_TM.lower_c_bcd  = 0u;
    g_TM.error_code   = 0u;
    g_TM.timer_100ms  = 0u;
    s_boot_100ms      = 0u;
    s_lower_spd_fbk      = 0u;
    s_lower_status_fbk   = 0u;
    s_spd_eff_max        = TM_SPEED_MAX;
    s_spd_eff_min        = TM_SPEED_MIN;
    s_start_ack_pending  = 0u;
    s_start_retried      = 0u;
    s_start_ack_wait_100ms = 0u;
    s_stop_ack_pending   = 0u;
    s_stop_retried       = 0u;
    s_stop_ack_wait_100ms  = 0u;
    s_stop_spd0_grace_100ms = 0u;

    s_cfg_step         = 0u;
    s_cfg_delay_ticks  = 0u;
    s_cfg_gate_250ms   = 0u;
    s_uart_idle_sec    = 0u;
    s_uart_rx_pulse    = 0u;
    s_err_stop_sent    = 0u;
    s_accum_dist       = 0.0f;
    s_stop_start_spd   = 0u;
    s_stop_ramp_cnt    = 0u;
    s_stop_disp_step   = 1u;
    s_stop_wait_cnt    = 0u;
    s_stop_motor_done  = false;

    Data_Reset();
    s_cmd_test_sel = 0u;
    s_suppress_keypad_beep  = 0;
}

_Bool Treadmill_ConsumeKeyBeepSuppress(void)
{
    if (s_suppress_keypad_beep) {
        s_suppress_keypad_beep = 0;
        return 1;
    }
    return 0;
}

/* Main 100 ms periodic task: error → comm → boot-cfg → run states → display. */
void Treadmill_Manager_100ms(void)
{
    /* Blink inhibit timer */
    if (g_TM.steady_timer > 0u && --g_TM.steady_timer == 0u) UI_MarkDirty();

    /* ---- Error state ---- */
    if (g_TM.state == TM_STATE_ERROR) {
        static TM_State_t s_prev = TM_STATE_BOOT;
        if (s_prev != TM_STATE_ERROR) Error_OnEntry();
        s_prev = TM_STATE_ERROR;
        Error_Poll();
        Display_Update();
        return;
    }
    { static TM_State_t s_prev = TM_STATE_BOOT; s_prev = g_TM.state; (void)s_prev; }

    /* ---- Boot parameter download（产测不参与下发） ---- */
    if (g_TM.state == TM_STATE_BOOT ||
        g_TM.state == TM_STATE_IDLE ||
        g_TM.state == TM_STATE_SETTING) {
        BootCfg_Poll();
    }

    /* ---- 产测：仅刷新测灯，不跑其它状态逻辑 ---- */
    if (g_TM.state == TM_STATE_TEST) {
        Display_Update();
        return;
    }

    /* ---- Per-state logic ---- */
    switch (g_TM.state) {
        case TM_STATE_COUNTDOWN: Countdown_Tick(); break;
        case TM_STATE_RUNNING:   Run_Tick();       break;
        case TM_STATE_STOPPING:  Stop_Tick();      break;
        default: break;
    }

    Display_Update();
}
