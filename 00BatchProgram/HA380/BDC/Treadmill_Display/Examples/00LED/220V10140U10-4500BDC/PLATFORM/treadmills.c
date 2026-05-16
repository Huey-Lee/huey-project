/*
 * treadmills.c  -  Treadmill application logic
 *
 * Module layout:
 *   §1  Includes & module-level state
 *   §2  Communication — send primitives
 *   §3  Communication — boot parameter download
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

/* CMD_TEST: lower board factory/pattern; exit by key, no time auto-exit. */
static _Bool   s_cmd_test_active;
static uint8_t s_cmd_test_sel;   /* 0..2 */
/* 1=ignore UART CMD_TEST (line often spams; without this, 下一包即拉回测灯) */
static _Bool   s_cmd_test_reject_lower;

/* Running-state comm frame counter; reset on every valid received frame. */
static uint16_t s_comm_timeout_cnt;

/* Accumulated distance for the current run session (metres). */
static float    s_accum_dist;
/* Stop animation state. */
static uint16_t s_stop_start_spd;    /* speed captured at STOPPING entry   */
static uint16_t s_stop_ramp_cnt;     /* proportional ramp length (ticks)   */
static uint16_t s_stop_wait_cnt;     /* ticks waiting for CMD_STOP_OVER    */
static bool     s_stop_motor_done;   /* motor confirmed stopped (flag)      */
/* 开停进减速在 Enter 内已响，抑制 Keypad 对同一次 PRESS 的重复蜂鸣 */
static _Bool    s_suppress_keypad_beep;

/* Forward declarations for functions defined later in this file. */
static void Data_Reset(void);
static void Enter_Stopping(void);
static void LowerBoard_MotorStopped_ToIdle(void);
static void Display_Update(void);


/*=============================================================================
 * §2  Communication — send primitives
 *===========================================================================*/

static void Send_Ack0(void)
{
    uint8_t z = 0u;
    AeroLink_Send(CMD_ACK, 1u, &z);
}

static void Send_Speed(uint16_t spd)
{
    uint8_t b = (spd > 255u) ? 255u : (uint8_t)spd;
    AeroLink_Send(CMD_SPEED, 1u, &b);
}

static void Send_Stop(void)
{
    uint8_t b = STATUS_MOTOR_STOP;
    AeroLink_Send(CMD_STATUS, 1u, &b);
    AeroLink_Send(CMD_STATUS, 1u, &b);
}

static void Send_Run(uint8_t speed_wire)
{
    uint8_t pair[2] = { STATUS_MOTOR_RUN, speed_wire };
    for (uint8_t i = 0u; i < 3u; i++) {
        AeroLink_Send(CMD_STATUS, 2u, pair);
    }
}


/*=============================================================================
 * §3  Boot parameter download
 *     s_cfg_table: after 250ms gate + post-gate, sent in BOOT (during 2s 全显) in one burst;
 *     keys are ignored until TM_STATE_IDLE (all BOOT states including optional version).
 *===========================================================================*/

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t d0;
    uint8_t d1;
} CfgRow_t;

static const CfgRow_t s_cfg_table[] = {
    { CMD_STATUS,               1u, STATUS_MOTOR_STOP,          0u },
    { CMD_STATUS,               1u, STATUS_MOTOR_STOP,          0u },
    { CMD_TREADMILLS_SPEED_MAX, 1u, TM_LOWER_TX_SPEED_MAX_BYTE, 0u },
    { CMD_VOLTAGE_MAX,          1u, TM_LOWER_VOLTAGE_MAX,       0u },
    { CMD_VOLTAGE_MIN,          1u, TM_LOWER_VOLTAGE_MIN,       0u },
    { CMD_OVER_CURRENT_MAX,     1u, TM_LOWER_OVER_CURRENT_MAX,   0u },
    { CMD_OVER_VOLTAGE_MAX,     1u, TM_LOWER_OVER_VOLTAGE_MAX,   0u },
    { CMD_STA_LEVEL,            1u, TM_LOWER_STA_LEVEL,          0u },
    { CMD_END_LEVEL,            1u, TM_LOWER_END_LEVEL,          0u },
    { CMD_ACCELERATED_LEVEL,    1u, TM_LOWER_ACCEL_LEVEL,        0u },
    { CMD_DECELERATION_LEVEL,   1u, TM_LOWER_DECEL_LEVEL,        0u },
    { CMD_KIV_1KM,              1u, TM_LOWER_KIV_1KM,           0u },
    { CMD_KIV_2KM,              1u, TM_LOWER_KIV_2KM,           0u },
    { CMD_KIV_3KM,              1u, TM_LOWER_KIV_3KM,           0u },
    { CMD_KIV_4KM,              1u, TM_LOWER_KIV_4KM,           0u },
    { CMD_KIV_5KM,              1u, TM_LOWER_KIV_5KM,           0u },
    { CMD_KIV_6KM,              1u, TM_LOWER_KIV_6KM,           0u },
    { CMD_KIV_7KM,              1u, TM_LOWER_KIV_7KM,           0u },
    { CMD_KIV_8KM,              1u, TM_LOWER_KIV_8KM,           0u },
    { CMD_KIV_9KM,              1u, TM_LOWER_KIV_9KM,           0u },
    { CMD_KIV_10KM,             1u, TM_LOWER_KIV_10KM,          0u },
    { CMD_KIV_11KM,             1u, TM_LOWER_KIV_11KM,          0u },
    { CMD_KIV_12KM,             1u, TM_LOWER_KIV_12KM,          0u },
};
/* Explicit count avoids Clang "incomplete type" if any row fails to parse. */
enum { CFG_TABLE_ROWS = 23 };
#define CFG_TABLE_LEN  ((uint8_t)CFG_TABLE_ROWS)

static uint8_t  s_cfg_step;          /* next row index to send   */
static uint16_t s_cfg_delay_ticks;   /* 100 ms post-gate counter */
static uint8_t  s_cfg_gate_250ms;    /* 250 ms gate counter      */
#if (TM_LOWER_SEND_READ_PARAM_BOOT)
static uint8_t  s_read_param_sent;
#endif
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
    if (r->len == 1u) {
        AeroLink_Send(r->cmd, 1u, &r->d0);
    } else {
        uint8_t pair[2] = { r->d0, r->d1 };
        AeroLink_Send(r->cmd, 2u, pair);
    }
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

#if (TM_LOWER_SEND_READ_PARAM_BOOT)
    if (s_read_param_sent == 0u) {
        /* 7F 08 01 00 02 00 7E: LEN=1, DATA=0, CHK=(8+1+0)%7=0x02（上显 plat_aerolink 对 READ_PARAM 发 %7；下控也接受异或 0x09） */
        uint8_t z = 0u;
        AeroLink_Send(CMD_READ_PARAM, 1u, &z);
        s_read_param_sent = 1u;
    }
#endif
}


/*=============================================================================
 * §4  Communication — link watchdog & frame dispatch
 *===========================================================================*/

/* 250 ms link-check state machine. */
static uint8_t s_lchk_sm;
static uint8_t s_lchk_ack_rx;
static uint8_t s_lchk_timeout;
static uint8_t s_lchk_err;
#define LCHK_ERR_MAX  10u

/* 1 s UART idle watchdog. */
static uint8_t s_uart_idle_sec;
static uint8_t s_uart_rx_pulse;

/* Called every 250 ms to send CMD_ACK heartbeat and check for echo.
 * Triggers error after LCHK_ERR_MAX consecutive missed responses during run. */
void Treadmill_On250msTasks(void)
{
    /* Advance lower-board gate from power-on (includes BOOT 全显) */
    if (s_cfg_gate_250ms < TM_LOWER_CFG_250MS_GATE) {
        s_cfg_gate_250ms++;
        if (s_cfg_gate_250ms == TM_LOWER_CFG_250MS_GATE) {
            s_cfg_delay_ticks = TM_LOWER_CFG_POST_GATE_100MS;
        }
    }

    /* Suppress heartbeat during config download to avoid frame collision. */
    if (g_TM.state == TM_STATE_ERROR) { s_lchk_sm = 2u; return; }
    if (g_TM.state == TM_STATE_BOOT) return; /* no ACK poll during 全显 */
    if (s_cfg_step < CFG_TABLE_LEN && s_cfg_gate_250ms >= TM_LOWER_CFG_250MS_GATE) return;

    switch (s_lchk_sm) {
        case 0u:                       /* send ACK, wait for echo */
            s_lchk_ack_rx  = 0u;
            s_lchk_timeout = 0u;
            Send_Ack0();
            s_lchk_sm = 1u;
            break;

        case 1u:                       /* check echo */
            s_lchk_timeout++;
            if (s_lchk_ack_rx) {
                s_lchk_ack_rx = 0u;
                s_lchk_err    = 0u;
                s_lchk_sm     = 0u;
            } else {
                Send_Ack0();
                if (s_lchk_timeout > 1u) {
                    s_lchk_timeout = 0u;
                    if (++s_lchk_err > LCHK_ERR_MAX) {
                        if (g_TM.state == TM_STATE_RUNNING ||
                            g_TM.state == TM_STATE_STOPPING) {
                            /* Error entry handled in §5 */
                            g_TM.state      = TM_STATE_ERROR;
                            g_TM.error_code = 1u;
                            Beep_Play(BEEP_ALARM);
                            s_lchk_sm = 2u;
                        } else {
                            s_lchk_err = 0u;
                            s_lchk_sm  = 0u;
                        }
                    } else {
                        s_lchk_sm = 0u;
                    }
                }
            }
            break;

        default: break;
    }
}

/* Called every 1 s.  Triggers er01 if no frame received for > 5 s during run. */
void Treadmill_On1000msUartWatchdog(void)
{
    if (g_TM.state == TM_STATE_BOOT) return;

    if (s_uart_rx_pulse) {
        s_uart_rx_pulse = 0u;
        s_uart_idle_sec = 0u;
    } else {
        if (s_uart_idle_sec < 250u) s_uart_idle_sec++;
    }

    if (s_uart_idle_sec > 5u &&
        Treadmill_LowerBoardReady() &&
        g_TM.state != TM_STATE_ERROR) {
        g_TM.state      = TM_STATE_ERROR;
        g_TM.error_code = 1u;
        Beep_Play(BEEP_ALARM);
    }
}

/* UART frame received callback (called from AeroLink_Handler). */
void AeroLink_OnFrameReceived(AeroRecvFrame_t *f)
{
    s_comm_timeout_cnt = 0u;
    s_lchk_ack_rx      = 1u;
    s_uart_rx_pulse    = 1u;

    switch (f->cmd) {

        case CMD_ACK:
            /* byte 0 = lower board firmware version; keep first valid value. */
            if (f->len >= 1u && f->data[0] != 0u && g_TM.lower_c_bcd == 0u) {
                g_TM.lower_c_bcd = f->data[0];
            }
            break;

        case CMD_ERROR:
            if (f->len >= 1u && f->data[0] != 0u) {
                bool was_error = (g_TM.state == TM_STATE_ERROR);
                g_TM.state = TM_STATE_ERROR;
                if (g_TM.error_code == 0u) {
                    g_TM.error_code = (f->data[0] > 99u) ? 99u : f->data[0];
                }
                /* Beep once per error entry; CMD_ERROR may arrive repeatedly. */
                if (!was_error) {
                    Beep_Play(BEEP_ALARM);
                }
            }
            break;

        case CMD_STOP_OVER:
            if (g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_STOPPING) {
                /* 下控已停：上控不再继续显示减速倒计时，直接清速度并回待机。 */
                LowerBoard_MotorStopped_ToIdle();
            }
            break;

        case CMD_STATUS: {
            if (g_TM.state == TM_STATE_BOOT || g_TM.state == TM_STATE_ERROR) break;
            if (f->len == 0u) break;
            uint8_t status = (f->len >= 3u) ? f->data[2] : f->data[0];
            switch (status) {
                case 9u:   /* motor running */
                    if (g_TM.state != TM_STATE_RUNNING &&
                        g_TM.state != TM_STATE_STOPPING) {
                        g_TM.state = TM_STATE_RUNNING;
                    }
                    break;
                case 10u:  /* motor stopped — same as CMD_STOP_OVER */
                    if (g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_STOPPING) {
                        LowerBoard_MotorStopped_ToIdle();
                    }
                    break;
                case 3u:   /* motor fault */
                    g_TM.state = TM_STATE_ERROR;
                    break;
                default: break;
            }
            break;
        }

        case CMD_TEST:
            if (f->len < 1u) break;
            if (s_cmd_test_reject_lower) {
                /* 上控已用按键退测灯；下位机仍连发时不再抢回，否则与 Display_Update/影子缓冲无关 */
                break;
            }
            Beep_Play(BEEP_CLICK);
            s_cmd_test_sel  = f->data[0];
            if (s_cmd_test_sel > 2u) s_cmd_test_sel = 0u;
            s_cmd_test_active = true;
            if (g_TM.state == TM_STATE_BOOT) {
                /* 避免 Display 在 BOOT 2s 内先全显，约 2s 后才到测灯/待机，被误认为「到点自动退测」 */
                g_TM.state       = TM_STATE_IDLE;
                g_TM.timer_100ms = 0u;
                s_boot_100ms     = 0u;
            }
            UI_MarkDirty();
            break;

        default: break;
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
                if (g_TM.speed >= TM_SPEED_MAX) break;
                uint32_t ns = (uint32_t)g_TM.speed + delta;
                g_TM.speed  = (ns > TM_SPEED_MAX) ? TM_SPEED_MAX : (uint16_t)ns;
            } else {
                if (g_TM.speed <= TM_SPEED_MIN) break;
                int32_t ns = (int32_t)g_TM.speed - (int32_t)delta;
                g_TM.speed = (ns < TM_SPEED_MIN) ? TM_SPEED_MIN : (uint16_t)ns;
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
    return s_cmd_test_active ? 1 : 0;
}

/* Key event dispatcher.  Returns true if the event was consumed. */
bool Treadmill_On_Event(uint8_t key_id, uint8_t evt)
{
    if (evt == K_EVT_RELEASE) return false;
    /* 测灯：仅遥控短按退出（无定时）；下位机连发 CMD_TEST 用 s_cmd_test_reject_lower 屏蔽 */
    if (Treadmill_CmdTestDisplayActive()) {
        if (evt == K_EVT_PRESS &&
            (key_id == K_ID_ONOFF || key_id == K_ID_MODE)) {
            s_cmd_test_active       = false;
            s_cmd_test_reject_lower = 1;
            Data_Reset();
            g_TM.state       = TM_STATE_IDLE;
            g_TM.timer_100ms = 0u;
            Display_Update();
            UI_Engine_Refresh();
        }
        return true;
    }
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

/* Enter stopping state.  Ramp length is proportional to current speed so
 * the display closely tracks the motor's actual deceleration at any speed.
 * Ramp ticks = TM_STOP_TIME_COUNT × speed / TM_SPEED_MAX (min 2 ticks). */
static void Enter_Stopping(void)
{
    /* 在 Send_Stop/串口 之前响，避免主循环被占用导致蜂鸣 1ms 节拍打不满、听成「一截或很弱」 */
    Beep_Play(BEEP_CLICK);
    s_stop_start_spd  = g_TM.speed;
    uint32_t ramp     = (uint32_t)TM_STOP_TIME_COUNT * g_TM.speed / TM_SPEED_MAX;
    s_stop_ramp_cnt   = (ramp < 2u) ? 2u : (uint16_t)ramp;
    s_stop_wait_cnt   = 0u;
    s_stop_motor_done = false;
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

/* 下位机报电机已停：清显示减速计时状态并回待机（与下控实际停机一致）。 */
static void LowerBoard_MotorStopped_ToIdle(void)
{
    s_stop_motor_done = false;
    s_stop_wait_cnt   = 0u;
    g_TM.timer_100ms  = 0u;
    Data_Reset();
    g_TM.state = TM_STATE_IDLE;
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
        g_TM.speed       = TM_SPEED_MIN;
        g_TM.state       = TM_STATE_RUNNING;
        g_TM.cycle_timer = 0u;
        g_TM.disp_index  = DISP_SPEED;
        s_accum_dist     = 0.0f;
        if (g_TM.target_mode != TARGET_TIME) g_TM.time = 0u;
        if (g_TM.target_mode != TARGET_DIST) g_TM.dist = 0.0f;
        if (g_TM.target_mode != TARGET_CAL)  g_TM.cal  = 0.0f;
        uint8_t sw = (uint8_t)((g_TM.speed > 255u) ? 255u : g_TM.speed);
        Send_Run(sw);
        Send_Speed(g_TM.speed);
    }
}

/* Running: update stats every 1 s (10 × 100 ms ticks). */
static void Run_Tick(void)
{
    static uint8_t s_sec_cnt = 0u;

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

/* Phase 1: proportional ramp mirrors motor decel — display stays in sync.
 * Phase 2: speed shows 0, wait up to TM_STOP_SAFETY_TIMEOUT (3 s) for
 *          CMD_STOP_OVER; then force IDLE as safety fallback. */
static void Stop_Tick(void)
{
    if (g_TM.timer_100ms > 0u) {
        /* Phase 1 — proportional ramp. */
        g_TM.timer_100ms--;
        g_TM.speed = (s_stop_ramp_cnt > 0u)
                     ? (uint16_t)((uint32_t)s_stop_start_spd *
                                   g_TM.timer_100ms / s_stop_ramp_cnt)
                     : 0u;
        UI_MarkDirty();
    } else {
        /* Phase 2 — ramp finished, display at 0.
         * Go to IDLE immediately if motor already confirmed stopped,
         * otherwise wait up to TM_STOP_SAFETY_TIMEOUT (3 s) as fallback. */
        g_TM.speed = 0u;
        UI_MarkDirty();
        if (s_stop_motor_done || ++s_stop_wait_cnt >= TM_STOP_SAFETY_TIMEOUT) {
            Data_Reset();
            g_TM.state = TM_STATE_IDLE;
        }
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
            if (cd < 1u) cd = 1u;
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

/* 下位 CMD_TEST：0=时间+卡路里区段全亮(仅数码)、速度+路程+功能图标灯全灭；1=反；2=全板段+四灯+冒号。 */
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
        for (i = 0u; i < 3u; i++) {
            UI_SetRaw((uint8_t)(TCT_ZT + i), 0xFFu);
            UI_SetRaw((uint8_t)(TCT_ZC + i), 0xFFu);
            UI_SetRaw((uint8_t)(TCT_ZS + i), 0u);
            UI_SetRaw((uint8_t)(TCT_ZD + i), 0u);
        }
    } else if (s_cmd_test_sel == 1u) {
        for (i = 0u; i < 3u; i++) {
            UI_SetRaw((uint8_t)(TCT_ZT + i), 0u);
            UI_SetRaw((uint8_t)(TCT_ZC + i), 0u);
            UI_SetRaw((uint8_t)(TCT_ZS + i), 0xFFu);
            UI_SetRaw((uint8_t)(TCT_ZD + i), 0xFFu);
        }
        UI_SetBitMode(TCT_IGR, UI_IND_SPEED_BIT, DISP_ON);
        UI_SetBitMode(TCT_IGR, UI_IND_DIS_BIT, DISP_ON);
    } else {
        for (i = 0u; i < (uint8_t)UI_GRID_MAX; i++) {
            UI_SetRaw(i, 0xFFu);
        }
        UI_SetBitMode(TCT_IGR, UI_IND_TIME_BIT, DISP_ON);
        UI_SetBitMode(TCT_IGR, UI_IND_SPEED_BIT, DISP_ON);
        UI_SetBitMode(TCT_IGR, UI_IND_DIS_BIT, DISP_ON);
        UI_SetBitMode(TCT_IGR, UI_IND_CAL_BIT, DISP_ON);
        UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_ON);
        UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_ON);
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
            if (cd < 1u) cd = 1u;
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
/* TM1620/TM1652 单窗：0=仅时/卡功能灯+数码全灭、速/里灯灭；1=反；2=全段+四灯+冒号全亮 */
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
            UI_SetRaw(i, 0u);
        }
        UI_SetBitMode(GRID_IND, LED_TIME, DISP_ON);
        UI_SetBitMode(GRID_IND, LED_CAL, DISP_ON);
    } else if (s_cmd_test_sel == 1u) {
        for (i = 0u; i < (uint8_t)UI_GRID_MAX; i++) {
            UI_SetRaw(i, 0u);
        }
        UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
        UI_SetBitMode(GRID_IND, LED_DIS, DISP_ON);
    } else {
        for (i = 0u; i < (uint8_t)UI_GRID_MAX; i++) {
            UI_SetRaw(i, 0xFFu);
        }
        UI_SetBitMode(GRID_IND, LED_TIME, DISP_ON);
        UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
        UI_SetBitMode(GRID_IND, LED_DIS, DISP_ON);
        UI_SetBitMode(GRID_IND, LED_CAL, DISP_ON);
        UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_ON);
        UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_ON);
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
    if (s_cmd_test_active) {
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

    s_cfg_step         = 0u;
    s_cfg_delay_ticks  = 0u;
    s_cfg_gate_250ms   = 0u;
    s_lchk_sm          = 0u;
    s_lchk_ack_rx      = 0u;
    s_lchk_timeout     = 0u;
    s_lchk_err         = 0u;
    s_uart_idle_sec    = 0u;
    s_uart_rx_pulse    = 0u;
    s_comm_timeout_cnt = 0u;
    s_err_stop_sent    = 0u;
    s_accum_dist       = 0.0f;
    s_stop_start_spd   = 0u;
    s_stop_ramp_cnt    = 0u;
    s_stop_wait_cnt    = 0u;
    s_stop_motor_done  = false;

#if (TM_LOWER_SEND_READ_PARAM_BOOT)
    s_read_param_sent  = 0u;
#endif

    Data_Reset();
    s_cmd_test_active       = false;
    s_cmd_test_reject_lower = 0;
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

    /* ---- Boot parameter download (during BOOT 全显 + after IDLE) ---- */
    if (g_TM.state == TM_STATE_BOOT ||
        g_TM.state == TM_STATE_IDLE ||
        g_TM.state == TM_STATE_SETTING) {
        BootCfg_Poll();
    }

    /* ---- Communication timeout (running / stopping only) ---- */
    if (g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_STOPPING) {
        if (++s_comm_timeout_cnt >= TM_COMM_TIMEOUT_COUNT) {
            s_comm_timeout_cnt = 0u;
            g_TM.state         = TM_STATE_ERROR;
            g_TM.error_code    = 1u;   /* er01: communication lost */
            Beep_Play(BEEP_ALARM);
        }
    } else {
        s_comm_timeout_cnt = 0u;
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
