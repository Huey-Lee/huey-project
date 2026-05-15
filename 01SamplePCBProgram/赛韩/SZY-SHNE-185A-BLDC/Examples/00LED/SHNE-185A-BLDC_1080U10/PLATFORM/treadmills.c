/* Treadmill: UI/state, keys, lower-board link */
#include "treadmills.h"
#include "ui_display.h"
#include "plat_segment.h"
#include "plat_beep.h"
#include "plat_keyfun.h"
#include "plat_aerolink.h"

#define COMM_TIMEOUT_COUNT  50  /* no valid frame: 50*100ms = 5s */

TM_Control_t g_TM;

#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
/* 圈灯共 12 步，须先于按键等代码；与下方 c_marquee 一致 */
#define TM_MARQUEE_COUNT    12u
static void prv_marquee_set(uint8_t n);
static uint8_t  s_tm1640_shared_cal;   /* 0=路程，1=卡路里 */
static uint16_t s_tm1640_swap_100ms;   /* 与 steady_timer/cycle_timer 脱钩，保证每 5s 切换 */

/* 跑马灯状态 */
static uint8_t  s_mq_on   = 0u;       /* 当前点亮步数 0..TM_MARQUEE_COUNT */
static uint16_t s_mq_tick = 0u;       /* 步进计时（×100ms） */
static int8_t   s_mq_dir  = 1;        /* 空闲呼吸方向 +1/-1 */
/* 用户按停：右橙冻结为「按停前」；减速 STOPPING 时左蓝随 g_TM.speed 递减至 0 */
static uint8_t  s_tm1640_outer_bars_frozen;
static uint8_t  s_tm1640_frozen_n_cal;
static uint8_t  s_tm1640_user_stop_snap_done;
#endif

static uint16_t g_comm_timeout_cnt = 0; /* no-frame counter, cleared on RX */
/* 下控 Data1 换算后的 0.1km/h（运行中用于与上显比对并重发档位） */
static uint16_t s_lower_spd_rx_tenth;
/* 下控运行信息 Data3/Data4：最大/最小档(1～10，大端 uint16)；换算为显示用 0.1km/h 后限制 g_TM.speed */
static uint16_t s_lower_speed_max_tenth = TM_SPEED_MAX;
static uint16_t s_lower_speed_min_tenth = TM_SPEED_MIN;
static uint16_t s_stop_initial_spd; /* 进入 STOPPING 时的速度(0.1km/h)，用于线性减速 */
static uint16_t s_stop_ramp_dur;    /* 本段停机斜坡总 tick（与 timer_100ms 初值一致） */
static int32_t  s_stop_ramp_err;    /* Bresenham 累加：在整段 dur 内均匀减 init 次 0.1 */
static uint8_t  s_lower_last_run_status = 10u; /* 下控最近 run_status；10=待机 */
static uint8_t  s_cd_run_resend_100ms;         /* 321 阶段启动命令重发间隔计数 */
static uint8_t  s_stop_cmd_resend_100ms;       /* STOP 重发间隔计数 */
/* 卡目标模式：进入 RUNNING 瞬间的剩余目标(0.001kcal)，用于侧条「已消耗」= 初值−当前 g_TM.cal */
static float    s_tm_cal_goal_mcal = 0.0f;
/* 时间目标 / P 程序：进入 RUNNING 时目标秒数；12 颗跑马灯按剩余比例亮（如 12min→每灯约 1min，60min→每灯约 5min；P01~P12 固定 30min） */
static uint32_t s_run_time_goal_sec = 0u;
/* 业务层已播限位音时，跳过本次 Keypad 的 BEEP_CLICK（与 HA380A 停机去重同理） */
static uint8_t  s_suppress_keypad_beep;
/* 用户按启停减速：结束后进 PAUSED；自动练完进 IDLE */
static uint8_t  s_stop_for_pause;
/* 自暂停 321 后进 RUNNING：勿重采 s_run_time_goal_sec / s_tm_cal_goal_mcal（仍为本次锻炼初值） */
static uint8_t  s_run_entry_skip_goal;
/* 设置界面无按键累计（×100ms），达 TM_SETTING_IDLE_EXIT_100MS 回待机 */
static uint16_t s_setting_idle_100ms;

/* 停机斜坡 tick：低速 init×k 缩短，高速顶格 TM_STOPPING_DURATION_100MS */
static uint16_t prv_stop_ramp_duration_ticks(uint16_t init_tenth) {
    uint32_t cap = (uint32_t)TM_STOPPING_DURATION_100MS;
    uint32_t lin = (uint32_t)init_tenth * (uint32_t)TM_STOP_TICKS_PER_TENTH;
    uint32_t mnr = (uint32_t)TM_STOP_MIN_RAMP_100MS;
    uint32_t eff = lin;
    if (eff < mnr) eff = mnr;
    if (eff > cap) eff = cap;
    if (eff < 1u) eff = 1u;
    return (uint16_t)eff;
}

static uint16_t prv_speed_max_eff(void) { return s_lower_speed_max_tenth; }
static uint16_t prv_speed_min_eff(void) { return s_lower_speed_min_tenth; }

/* 将上显/指令速度限制在下控允许范围内（0 不抬升到 min，便于待机） */
static void prv_clamp_speed_to_lower(void)
{
    uint16_t smax = prv_speed_max_eff();
    uint16_t smin = prv_speed_min_eff();
    if (g_TM.speed > smax) g_TM.speed = smax;
    /* STOPPING 线性减档会低于 smin；若再抬到 min 会与 RX 钳位打架 */
    if (g_TM.state != TM_STATE_STOPPING) {
        if (g_TM.speed > 0u && g_TM.speed < smin) g_TM.speed = smin;
    }
}

/* 显示十分之一 km/h → 线缆侧档位 1～10（与 ALR 系 0.6～6.2 对 1～10 线性对应） */
static uint8_t prv_speed_tenth_to_gear_wire(uint16_t tenth)
{
    uint32_t span_u;
    if (tenth == 0u)
        return 0u;
    if (tenth <= TM_SPEED_MIN)
        return (uint8_t)TM_GEAR_WIRE_MIN;
    if (tenth >= TM_SPEED_MAX)
        return (uint8_t)TM_GEAR_WIRE_MAX;
    span_u = (uint32_t)TM_SPEED_MAX - (uint32_t)TM_SPEED_MIN;
    return (uint8_t)((uint32_t)TM_GEAR_WIRE_MIN +
                     (uint32_t)(tenth - TM_SPEED_MIN) * (TM_GEAR_WIRE_MAX - TM_GEAR_WIRE_MIN) /
                         span_u);
}

/* 下控回报档 1～10 → 显示用十分之一 km/h（与上面互逆的整数划分） */
static uint16_t prv_gear_wire_to_speed_tenth(uint16_t gear)
{
    uint32_t span_u;
    if (gear <= TM_GEAR_WIRE_MIN)
        return TM_SPEED_MIN;
    if (gear >= TM_GEAR_WIRE_MAX)
        return TM_SPEED_MAX;
    span_u = (uint32_t)TM_SPEED_MAX - (uint32_t)TM_SPEED_MIN;
    return (uint16_t)(TM_SPEED_MIN + (uint32_t)(gear - TM_GEAR_WIRE_MIN) * span_u /
                                    (TM_GEAR_WIRE_MAX - TM_GEAR_WIRE_MIN));
}

/* TM1620/TM1652: indicator LEDs on logic grid GRID_IND */
#define GRID_IND    4
#define LED_SPEED   1
#define LED_TIME    0
#define LED_DIS     2
#define LED_CAL     3

/* Colon: TM1640 时间在逻辑格 3..6，冒号在位5(逻辑格4)；TM1620/1652 在格 0 */
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
#define GRID_COLON    4
#else
#define GRID_COLON    0
#endif
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
#define SEG_COLON_H   7  /* _SEG_DP -> UI_SEG_DP_HW */
#define SEG_COLON_L   7
#else
#define SEG_COLON_H   1
#define SEG_COLON_L   1
#endif


/* Time zone: minutes 0..99 for 0:mm (ceil remaining sec when target=time and running). */
static uint16_t TM_Time_Minutes_For_Display(void) {
    uint32_t t = g_TM.time;
    if (g_TM.target_mode == TARGET_TIME &&
        (g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_STOPPING ||
         g_TM.state == TM_STATE_PAUSED)) {
        if (t == 0u) return 0;
        uint32_t m = (t + 59u) / 60u;
        return (m > 99u) ? 99u : (uint16_t)m;
    }
    if (g_TM.target_mode == TARGET_TIME &&
        (g_TM.state == TM_STATE_SETTING || g_TM.state == TM_STATE_IDLE)) {
        uint32_t m = t / 60u;
        return (m > 99u) ? 99u : (uint16_t)m;
    }
    {
        uint32_t m = t / 60u;
        return (m > 99u) ? 99u : (uint16_t)m;
    }
}

/* TM1640 时间区 MM:SS：g_TM.time 为秒，显示上限 99:59 */
static uint16_t TM_Time_MMSS_For_Display(void) {
    uint32_t sec = g_TM.time;
    if (sec > 5999u) sec = 5999u;
    uint32_t mm = sec / 60u;
    uint32_t ss = sec % 60u;
    if (mm > 99u) {
        mm = 99u;
        ss = 59u;
    }
    return (uint16_t)(mm * 100u + ss);
}

/* Distance: g_TM.dist meters -> UI tenths of km (10 = 1.0 km, 50 = 5.0 km, 990 = 99.0 km). */
static uint16_t TM_Dist_Tenths_For_Display(void) {
    float d = g_TM.dist;
    if (d < 0.0f) d = 0.0f;
    uint32_t dk = (uint32_t)(d / 100.0f); /* 100 m == 0.1 km, truncate */
    if (dk > 990u) dk = 990u;
    return (uint16_t)dk;
}


static void Send_Speed_To_Lower(void) {
    uint8_t data[8] = {0};
    uint16_t s = g_TM.speed;
    /* 停转：0；运行：十分之一 km/h → 线侧 1～10 档 */
    data[1] = prv_speed_tenth_to_gear_wire(s);
    AeroLink_Send(FC_GEAR, 0x00, data);
}

/* 回待机/上电：时间速度清零；路程/卡路里清零（待机轮显 0.0 / 0；进各设置项时另赋 TM_*_SET） */
static void Treadmill_Reset_Data(void) {
    g_TM.time             = 0;
    g_TM.dist             = 0.0f;
    g_TM.cal              = 0.0f;
    g_TM.speed            = 0;
    g_TM.target_mode      = TARGET_NONE;
    g_TM.disp_index       = DISP_TIME;
    g_TM.steady_timer     = 0;
    g_TM.cycle_timer      = 0;
    g_TM.prog_index       = 0;
    g_TM.prog_seg         = 0;
    g_TM.seg_elapsed_sec  = 0;
    g_TM.pending_lower_standby     = false;
    g_TM.lower_standby_wait_100ms  = 0;
    g_TM.timer_100ms               = 0;
    g_TM.resume_from_pause         = false;
    s_stop_for_pause               = 0u;
    s_run_entry_skip_goal          = 0u;
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
    s_tm1640_outer_bars_frozen   = 0u;
    s_tm1640_user_stop_snap_done = 0u;
#endif
    s_stop_ramp_err                = 0;
    s_stop_ramp_dur                = 0;
    s_stop_cmd_resend_100ms        = 0;
    s_tm_cal_goal_mcal             = 0.0f;
    s_run_time_goal_sec            = 0u;
    s_suppress_keypad_beep         = 0u;
    s_setting_idle_100ms           = 0u;
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
    s_tm1640_shared_cal  = 0;
    s_tm1640_swap_100ms  = 0;
    s_mq_on   = 0u;
    s_mq_tick = 0u;
    s_mq_dir  = 1;
#endif
    UI_ClearAll();
}

void Treadmill_Init(void) {
    g_TM.state      = TM_STATE_BOOT;
    g_TM.boot_phase = BOOT_FULL;
    s_suppress_keypad_beep = 0u;
    Treadmill_Reset_Data();
}

_Bool Treadmill_ConsumeKeyBeepSuppress(void)
{
    if (s_suppress_keypad_beep) {
        s_suppress_keypad_beep = 0u;
        return 1;
    }
    return 0;
}

/* ----------------------------------------------------------------
 * Program 速度表：12 程序 × 10 段，内部 0.1 km/h（6～62≈0.6～6.2 km/h）
 * 每段 3 分钟，共 30 分钟
 * ---------------------------------------------------------------- */
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
/* 由旧表 1.0～10.0 km/h 线性缩到 0.6～6.2（6～62） */
static const uint16_t c_prog_speed[TM_PROG_COUNT][TM_PROG_SEG_COUNT] = {
    /* seg:  1    2    3    4    5    6    7    8    9   10  */
    /* P01 */{13,  13,  62,  34,  48,  48,  48,  48,  48,  13},
    /* P02 */{13,  13,  34,  34,  48,  48,  48,  48,  13,  13},
    /* P03 */{ 6,  13,  34,  62,  62,  62,  62,   6,  13,   6},
    /* P04 */{13,  34,  62,  62,  62,  62,  62,  34,  13,  13},
    /* P05 */{ 6,   6,  34,  34,  62,  62,  48,  34,  13,   6},
    /* P06 */{ 6,   6,  34,  62,  62,  62,  48,  62,  13,   6},
    /* P07 */{ 6,   6,  34,  34,  48,  62,  48,  48,  13,   6},
    /* P08 */{13,  13,  48,  48,  62,  62,  62,  13,  13,   6},
    /* P09 */{ 6,  34,  34,  48,  48,  62,  62,  48,  34,   6},
    /* P10 */{13,  48,  62,  34,  62,  62,  48,  62,  13,   6},
    /* P11 */{ 6,  13,  34,  48,  62,  62,  62,  48,  13,  13},
    /* P12 */{ 6,   6,  13,   6,  13,  13,  13,   6,  13,   6},
};
#endif /* LED_DRIVER_TM1640 */

/* hold multiplier: 0 -> 1 to avoid zero step */
static uint32_t TM_Step_Mult(bool is_hold, uint32_t hold_mult) {
    uint32_t m = is_hold ? hold_mult : 1u;
    return (m == 0u) ? 1u : m;
}

/* dir +/-1; short vs hold uses TM_*_HOLD_STEP_MULT
 * 运行中：速度第一次加到顶或减到底时 BEEP_LONG 一声（短按连加或长按连加均可）；
 * 已在顶/底再按不再响；短按触发时限位音与按键 CLICK 去重（长按本就不发 CLICK）。 */
static void TM_Adjust_Process(int8_t dir, bool is_hold) {
    g_TM.steady_timer = 20;
    UI_MarkDirty();

    switch (g_TM.disp_index) {
        case DISP_SPEED: {
            uint16_t speed_before = g_TM.speed;
            uint32_t mult = TM_Step_Mult(is_hold, (uint32_t)TM_SPEED_HOLD_STEP_MULT);
            uint32_t delta = (uint32_t)TM_SPEED_STEP * mult;
            if (dir > 0) {
                if (g_TM.speed < prv_speed_max_eff()) {
                    uint32_t ns = (uint32_t)g_TM.speed + delta;
                    uint16_t smax = prv_speed_max_eff();
                    g_TM.speed = (ns > smax) ? smax : (uint16_t)ns;
                }
            } else {
                if (g_TM.speed > prv_speed_min_eff()) {
                    int32_t ns = (int32_t)g_TM.speed - (int32_t)delta;
                    uint16_t smin = prv_speed_min_eff();
                    g_TM.speed = (ns < (int32_t)smin) ? smin : (uint16_t)ns;
                }
            }
            if (g_TM.state == TM_STATE_RUNNING && g_TM.speed != speed_before)
                Send_Speed_To_Lower();
            if (g_TM.state == TM_STATE_RUNNING) {
                bool hit_cap = false;
                if (dir > 0)
                    hit_cap =
                        (speed_before < prv_speed_max_eff() &&
                         g_TM.speed >= prv_speed_max_eff());
                else
                    hit_cap =
                        (speed_before > prv_speed_min_eff() &&
                         g_TM.speed <= prv_speed_min_eff());
                if (hit_cap) {
                    Beep_Play(BEEP_LONG);
                    if (!is_hold) s_suppress_keypad_beep = 1u;
                }
            }
            break;
        }
        case DISP_TIME: {
            uint32_t mult = TM_Step_Mult(is_hold, (uint32_t)TM_TIME_HOLD_STEP_MULT);
            uint32_t delta = (uint32_t)TM_TIME_STEP * mult;
            if (dir > 0) {
                if (g_TM.time >= TM_TIME_MAX) g_TM.time = TM_TIME_MIN;
                else {
                    uint64_t ns = (uint64_t)g_TM.time + (uint64_t)delta;
                    if (ns > (uint64_t)TM_TIME_MAX) { g_TM.time = TM_TIME_MAX; }
                    else g_TM.time = (uint32_t)ns;
                }
            } else {
                if (g_TM.time <= TM_TIME_MIN) g_TM.time = TM_TIME_MAX;
                else {
                    if (g_TM.time - TM_TIME_MIN < delta) { g_TM.time = TM_TIME_MIN; }
                    else g_TM.time -= delta;
                }
            }
            break;
        }
        case DISP_DIST: {
            uint32_t mult = TM_Step_Mult(is_hold, (uint32_t)TM_DIST_HOLD_STEP_MULT);
            float delta = TM_DIST_STEP * (float)mult;
            if (dir > 0) {
                if (g_TM.dist >= TM_DIST_MAX) g_TM.dist = TM_DIST_MIN;
                else {
                    float nd = g_TM.dist + delta;
                    if (nd > TM_DIST_MAX) { g_TM.dist = TM_DIST_MAX; }
                    else g_TM.dist = nd;
                }
            } else {
                if (g_TM.dist <= TM_DIST_MIN) g_TM.dist = TM_DIST_MAX;
                else {
                    float nd = g_TM.dist - delta;
                    if (nd < TM_DIST_MIN) { g_TM.dist = TM_DIST_MIN; }
                    else g_TM.dist = nd;
                }
            }
            break;
        }
        case DISP_CAL: {
            uint32_t mult = TM_Step_Mult(is_hold, (uint32_t)TM_CAL_HOLD_STEP_MULT);
            float delta = TM_CAL_STEP * (float)mult;
            if (dir > 0) {
                if (g_TM.cal >= TM_CAL_MAX) g_TM.cal = TM_CAL_MIN;
                else {
                    float nc = g_TM.cal + delta;
                    if (nc > TM_CAL_MAX) { g_TM.cal = TM_CAL_MAX; }
                    else g_TM.cal = nc;
                }
            } else {
                if (g_TM.cal <= TM_CAL_MIN) g_TM.cal = TM_CAL_MAX;
                else {
                    float nc = g_TM.cal - delta;
                    if (nc < TM_CAL_MIN) { g_TM.cal = TM_CAL_MIN; }
                    else g_TM.cal = nc;
                }
            }
            break;
        }
        default: break;
    }
}

/* true=handled, false=ignored */
bool Treadmill_On_Event(uint8_t key_id, uint8_t evt) {
    if (evt == K_EVT_RELEASE) return false;
    if ((key_id == K_ID_MODE || key_id == K_ID_ONOFF) && evt == K_EVT_HOLD) return false;

    /* 报错界面：所有键无效（蜂鸣由 Keypad 层在 ERROR 时不再调用） */
    if (g_TM.state == TM_STATE_ERROR)
        return false;

    if (g_TM.state == TM_STATE_BOOT) {
        if (key_id == K_ID_ONOFF || key_id == K_ID_MODE) {
            g_TM.state = TM_STATE_IDLE;
            g_TM.timer_100ms = 0;
            Treadmill_Reset_Data();
            UI_MarkDirty();
            return true;
        }
        return false;
    }

    switch (g_TM.state) {
        case TM_STATE_IDLE:
            if (key_id == K_ID_UP || key_id == K_ID_DN) return false;
            if (key_id == K_ID_ONOFF) {
                Treadmill_Reset_Data();
                g_TM.state = TM_STATE_COUNTDOWN;
                g_TM.timer_100ms         = 30;
                s_cd_run_resend_100ms    = 0;
            } else if (key_id == K_ID_MODE) {
                g_TM.state = TM_STATE_SETTING;
                Treadmill_Reset_Data();
                g_TM.disp_index  = DISP_TIME;
                g_TM.target_mode = TARGET_TIME;
                g_TM.time        = TM_TIME_SET;
                s_setting_idle_100ms = 0u;
            }
            break;

        case TM_STATE_SETTING:
            s_setting_idle_100ms = 0u;
            if (key_id == K_ID_MODE) {
                /* 切换设置项：离开当前项时清零，确保只有一个目标有效 */
                if (g_TM.disp_index == DISP_TIME) {
                    g_TM.time        = 0;               /* 清零时间目标 */
                    g_TM.disp_index  = DISP_DIST;
                    g_TM.target_mode = TARGET_DIST;
                    g_TM.dist        = TM_DIST_SET;     /* 路程预设默认值 */
                } else if (g_TM.disp_index == DISP_DIST) {
                    g_TM.dist        = 0.0f;            /* 清零路程目标 */
                    g_TM.disp_index  = DISP_CAL;
                    g_TM.target_mode = TARGET_CAL;
                    g_TM.cal         = TM_CAL_SET;      /* 卡路里预设默认值 */
                } else if (g_TM.disp_index == DISP_CAL) {
                    g_TM.cal         = 0.0f;            /* 清零卡路里目标 */
                    g_TM.disp_index  = DISP_PROGRAM;
                    g_TM.target_mode = TARGET_PROGRAM;
                    g_TM.prog_index  = 0;               /* 从 P01 开始 */
                } else if (g_TM.disp_index == DISP_PROGRAM) {
                    /* P01..P12 循环，P12 之后回到待机 */
                    if (g_TM.prog_index < (uint8_t)(TM_PROG_COUNT - 1u)) {
                        g_TM.prog_index++;
                    } else {
                        g_TM.state = TM_STATE_IDLE;
                        Treadmill_Reset_Data();
                    }
                } else {
                    g_TM.state = TM_STATE_IDLE;
                    Treadmill_Reset_Data();
                }
            } else if (key_id == K_ID_UP || key_id == K_ID_DN) {
                /* DISP_PROGRAM 无需手动调整（程序速度固定） */
                if (g_TM.disp_index != DISP_PROGRAM)
                    TM_Adjust_Process(key_id == K_ID_UP ? 1 : -1, (evt == K_EVT_HOLD));
            } else if (key_id == K_ID_ONOFF) {
                if (g_TM.disp_index == DISP_PROGRAM) {
                    /* 启动 P 程序：分段从 0；RUNNING 后 g_TM.time=剩余秒(30:00 倒计)，在 321 结束处置 TM_PROG_TOTAL_SEC */
                    g_TM.prog_seg        = 0;
                    g_TM.seg_elapsed_sec = 0;
                    g_TM.time            = 0;
                    g_TM.speed = c_prog_speed[g_TM.prog_index][0];
                    prv_clamp_speed_to_lower();
                }
                g_TM.state = TM_STATE_COUNTDOWN;
                g_TM.timer_100ms      = 30;
                s_cd_run_resend_100ms = 0;
            }
            break;

        case TM_STATE_COUNTDOWN:
            if (key_id == K_ID_ONOFF) {
                /* 321 期间再按启停：停电机；若自暂停恢复流程则回到 PAUSED，否则清零回待机 */
                AeroLink_Send(FC_CONTROL, FC_CTRL_SFC_STOP, (void*)0);
                if (g_TM.resume_from_pause) {
                    g_TM.resume_from_pause = false;
                    g_TM.state = TM_STATE_PAUSED;
                } else {
                    Treadmill_Reset_Data();
                    g_TM.state = TM_STATE_IDLE;
                }
            }
            break;

        case TM_STATE_RUNNING:
            if (key_id == K_ID_MODE) return false;
            if (key_id == K_ID_ONOFF) {
                s_stop_for_pause = 1u;
                s_stop_initial_spd = g_TM.speed;
                s_stop_ramp_err    = 0;
                s_stop_ramp_dur    = prv_stop_ramp_duration_ticks(s_stop_initial_spd);
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
                s_tm1640_user_stop_snap_done = 0u;
                /* 圈灯由 prv_marquee_update STOPPING 分支保持与按停前运行态一致，不强制改写 */
#endif
                g_TM.state = TM_STATE_STOPPING;
                g_TM.pending_lower_standby    = false;
                g_TM.lower_standby_wait_100ms = 0;
                g_TM.timer_100ms          = s_stop_ramp_dur;
                s_stop_cmd_resend_100ms   = 0;
                AeroLink_Send(FC_CONTROL, FC_CTRL_SFC_STOP, (void*)0);
            } else if (key_id == K_ID_UP || key_id == K_ID_DN) {
                g_TM.disp_index = DISP_SPEED; g_TM.cycle_timer = 0;
                TM_Adjust_Process(key_id == K_ID_UP ? 1 : -1, (evt == K_EVT_HOLD));
            }
            break;

        case TM_STATE_PAUSED:
            if (key_id == K_ID_UP || key_id == K_ID_DN) return false;
            if (key_id == K_ID_MODE) {
                g_TM.state = TM_STATE_SETTING;
                Treadmill_Reset_Data();
                g_TM.disp_index  = DISP_TIME;
                g_TM.target_mode = TARGET_TIME;
                g_TM.time        = TM_TIME_SET;
                s_setting_idle_100ms = 0u;
            } else if (key_id == K_ID_ONOFF) {
                g_TM.resume_from_pause = true;
                if (g_TM.target_mode == TARGET_PROGRAM) {
                    g_TM.speed = c_prog_speed[g_TM.prog_index][g_TM.prog_seg];
                } else {
                    g_TM.speed = prv_speed_min_eff();
                }
                prv_clamp_speed_to_lower();
                g_TM.state = TM_STATE_COUNTDOWN;
                g_TM.timer_100ms      = 30;
                s_cd_run_resend_100ms = 0;
            }
            break;

        case TM_STATE_STOPPING:
            return false;

        default: break;
    }
    UI_MarkDirty();
    return true;
}


/* TM1640: 逻辑格 0..9 数码；10=G10 功能灯+圈灯后半；11=G5 圈灯前半；见 c_marquee */
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)

#define TM1640_ZONE_SPEED   0
#define TM1640_ZONE_TIME    3
#define TM1640_ZONE_SHARED  7

/* 侧向 8 路灯：G1 蓝(速度)、G14 橙(卡路里)，D0..D7=板 S1..S8，亮灯顺序见 prv_tm1640_speed_cal_bars */
#define TM1640_GRID_G14_ORANGE  12u    /* UI_GRID_MAP→物理 13 */
#define TM1640_GRID_G1_BLUE     13u    /* UI_GRID_MAP→物理 0  */

/* 四灯均在 G10：速度=S1(dp)、时间=S3(e)、路程=S8(a)、卡路里=S6(b) */
#define TM1640_IND_GRID_SPEED  10
#define TM1640_IND_GRID_FUNC   10
#define TM1640_IND_SEG_SPEED   7
#define TM1640_IND_SEG_TIME    4
#define TM1640_IND_SEG_DIS     0
#define TM1640_IND_SEG_CAL     1

#define TM1640_IndSpeed(m)  UI_SetBitMode(TM1640_IND_GRID_SPEED, TM1640_IND_SEG_SPEED, (m))
#define TM1640_IndTime(m)   UI_SetBitMode(TM1640_IND_GRID_FUNC, TM1640_IND_SEG_TIME, (m))
#define TM1640_IndDis(m)    UI_SetBitMode(TM1640_IND_GRID_FUNC, TM1640_IND_SEG_DIS, (m))
#define TM1640_IndCal(m)    UI_SetBitMode(TM1640_IND_GRID_FUNC, TM1640_IND_SEG_CAL, (m))

/* 在共享区（逻辑格 7-9）显示 "P xx"（P01-P12） */
static void prv_show_program_id(uint8_t prog_idx)
{
    uint8_t num = (uint8_t)(prog_idx + 1u);   /* 1..12 */
    UI_SetRaw(TM1640_ZONE_SHARED,      SEG_RAW(SEG_P));
    UI_SetNumber(TM1640_ZONE_SHARED + 1u, num, 2u, 0u, ZERO_SHOW);
}

/* ----------------------------------------------------------------
 * 圆圈时间指示灯：仅 G5 + G10 原理图 S 序；UI_SetBitMode 的 bit 是「标准段码」a..g=0..6、DP=7。
 * G5：S1,S3,S2,S4 后为 S5→S7（与原理图 1324758 中 7/5 顺序对调），S7 段连亮两步，再 S8。
 *   段序 7,4,3,2,6,5,5,0（格 11）；S7=S7 线=段 bit5，S5=段 bit6。
 * G10-S(7542)：S7,S5,S4,S2 → 段 5,6,2,3（格 10）。勿含指示灯时间(bit4)、卡(bit1)。
 * 指示灯 G10-S(1386)：见 TM1640_IND_SEG_*。共 12 步；勿动逻辑格 9（GR13 数码）。
 * ---------------------------------------------------------------- */
#define TM_MARQUEE_IDLE_STEP  2u   /* 空闲呼吸：每 2×100ms 步进一格 */
#define TM_MARQUEE_RUN_STEP  50u   /* 运行模式：每 50×100ms=5s 步进一格 */

typedef struct { uint8_t grid; uint8_t bit; } MarqueePos_t;

static const MarqueePos_t c_marquee[TM_MARQUEE_COUNT] = {
    {11u, 7u}, {11u, 4u}, {11u, 3u}, {11u, 2u}, {11u, 6u}, {11u, 1u}, {11u, 5u}, {11u, 0u},
    {10u, 5u}, {10u, 6u}, {10u, 2u}, {10u, 3u},
};

/* 将前 n 个跑马灯点亮，其余全灭 */
static void prv_marquee_set(uint8_t n)
{
    uint8_t i;
    for (i = 0u; i < TM_MARQUEE_COUNT; i++) {
        UI_SetBitMode(c_marquee[i].grid, c_marquee[i].bit,
                      (i < n) ? DISP_ON : DISP_OFF);
    }
}

/* 每次全屏绘制后同步跑马灯（在功能灯之后，避免笔段竞争） */
static void prv_marquee_sync_after_leds(void)
{
    switch (g_TM.state) {
        case TM_STATE_IDLE:
        case TM_STATE_COUNTDOWN:
        case TM_STATE_PAUSED:
        case TM_STATE_RUNNING:
            prv_marquee_set(s_mq_on);
            break;
        case TM_STATE_STOPPING:
            prv_marquee_set(s_mq_on);
            break;
        case TM_STATE_SETTING:
        case TM_STATE_ERROR:
            prv_marquee_set(0u);
            break;
        default:
            break;
    }
}

/* 每 100ms 调用一次，按状态更新跑马灯 */
static void prv_marquee_update(void)
{
    switch (g_TM.state) {

        case TM_STATE_IDLE:
            /* 待机：呼吸 */
            if (++s_mq_tick < TM_MARQUEE_IDLE_STEP) break;
            s_mq_tick = 0u;
            if (s_mq_dir > 0) {
                if (s_mq_on < TM_MARQUEE_COUNT) s_mq_on++;
                if (s_mq_on >= TM_MARQUEE_COUNT) s_mq_dir = -1;
            } else {
                if (s_mq_on > 0u) s_mq_on--;
                if (s_mq_on == 0u) s_mq_dir = 1;
            }
            prv_marquee_set(s_mq_on);
            break;

        case TM_STATE_COUNTDOWN:
            /* 首次启动 321：呼吸；自暂停恢复 321：与 PAUSED 同比例，避免进跑时格数错乱 */
            if (g_TM.resume_from_pause) {
                if (g_TM.target_mode == TARGET_TIME && s_run_time_goal_sec > 0u) {
                    uint32_t rem = g_TM.time;
                    uint32_t gsec = s_run_time_goal_sec;
                    uint32_t n =
                        (rem * (uint32_t)TM_MARQUEE_COUNT + gsec - 1u) / gsec;
                    if (n > TM_MARQUEE_COUNT) n = TM_MARQUEE_COUNT;
                    s_mq_on   = (uint8_t)n;
                    s_mq_tick = 0u;
                    prv_marquee_set(s_mq_on);
                } else if (g_TM.target_mode == TARGET_PROGRAM) {
                    uint32_t rem = g_TM.time;
                    if (rem > TM_PROG_TOTAL_SEC) rem = TM_PROG_TOTAL_SEC;
                    uint32_t gsec = TM_PROG_TOTAL_SEC;
                    uint32_t n =
                        (rem * (uint32_t)TM_MARQUEE_COUNT + gsec - 1u) / gsec;
                    if (n > TM_MARQUEE_COUNT) n = TM_MARQUEE_COUNT;
                    s_mq_on   = (uint8_t)n;
                    s_mq_tick = 0u;
                    prv_marquee_set(s_mq_on);
                } else {
                    prv_marquee_set(s_mq_on);
                }
            } else {
                if (++s_mq_tick < TM_MARQUEE_IDLE_STEP) break;
                s_mq_tick = 0u;
                if (s_mq_dir > 0) {
                    if (s_mq_on < TM_MARQUEE_COUNT) s_mq_on++;
                    if (s_mq_on >= TM_MARQUEE_COUNT) s_mq_dir = -1;
                } else {
                    if (s_mq_on > 0u) s_mq_on--;
                    if (s_mq_on == 0u) s_mq_dir = 1;
                }
                prv_marquee_set(s_mq_on);
            }
            break;

        case TM_STATE_PAUSED:
            /* 与 RUNNING 同源：按剩余时间/P 程序占比亮灯，恢复跑步后从同一格继续 */
            if (g_TM.target_mode == TARGET_TIME && s_run_time_goal_sec > 0u) {
                uint32_t rem = g_TM.time;
                uint32_t gsec = s_run_time_goal_sec;
                uint32_t n = (rem * (uint32_t)TM_MARQUEE_COUNT + gsec - 1u) / gsec;
                if (n > TM_MARQUEE_COUNT) n = TM_MARQUEE_COUNT;
                s_mq_on = (uint8_t)n;
                s_mq_tick = 0u;
                prv_marquee_set(s_mq_on);
            } else if (g_TM.target_mode == TARGET_PROGRAM) {
                uint32_t rem = g_TM.time;
                if (rem > TM_PROG_TOTAL_SEC) rem = TM_PROG_TOTAL_SEC;
                uint32_t gsec = TM_PROG_TOTAL_SEC;
                uint32_t n =
                    (rem * (uint32_t)TM_MARQUEE_COUNT + gsec - 1u) / gsec;
                if (n > TM_MARQUEE_COUNT) n = TM_MARQUEE_COUNT;
                s_mq_on = (uint8_t)n;
                s_mq_tick = 0u;
                prv_marquee_set(s_mq_on);
            } else {
                /* 无时间圈目标：保持按停前装饰格，不呼吸步进 */
                prv_marquee_set(s_mq_on);
            }
            break;

        case TM_STATE_RUNNING:
            /* 时间目标/P 程序：12 灯=剩余时间占比（目标总秒数均分，倒计灭灯）；其它：原先 5s 一步装饰跑马 */
            if (g_TM.target_mode == TARGET_TIME && s_run_time_goal_sec > 0u) {
                uint32_t rem = g_TM.time;
                uint32_t gsec = s_run_time_goal_sec;
                uint32_t n = (rem * (uint32_t)TM_MARQUEE_COUNT + gsec - 1u) / gsec;
                if (n > TM_MARQUEE_COUNT) n = TM_MARQUEE_COUNT;
                s_mq_on = (uint8_t)n;
                s_mq_tick = 0u;
                prv_marquee_set(s_mq_on);
            } else if (g_TM.target_mode == TARGET_PROGRAM) {
                /* g_TM.time = 剩余秒 */
                uint32_t rem = g_TM.time;
                if (rem > TM_PROG_TOTAL_SEC) rem = TM_PROG_TOTAL_SEC;
                uint32_t gsec = TM_PROG_TOTAL_SEC;
                uint32_t n =
                    (rem * (uint32_t)TM_MARQUEE_COUNT + gsec - 1u) / gsec;
                if (n > TM_MARQUEE_COUNT) n = TM_MARQUEE_COUNT;
                s_mq_on = (uint8_t)n;
                s_mq_tick = 0u;
                prv_marquee_set(s_mq_on);
            } else {
                if (++s_mq_tick < TM_MARQUEE_RUN_STEP) break;
                s_mq_tick = 0u;
                if (s_mq_on < TM_MARQUEE_COUNT) {
                    s_mq_on++;
                } else {
                    s_mq_on = 0u;
                }
                prv_marquee_set(s_mq_on);
            }
            break;

        case TM_STATE_SETTING:
            /* 设置界面：跑马灯熄灭 */
            s_mq_tick = 0u;
            if (s_mq_on != 0u) {
                s_mq_on = 0u;
                prv_marquee_set(0u);
            }
            break;

        case TM_STATE_STOPPING:
            if (g_TM.timer_100ms > 0u) {
                if (s_stop_for_pause) {
                    /* 用户按停减速：与 PAUSED 同源——时间/P 按剩余秒占格；装饰模式保持按停前格不步进 */
                    if (g_TM.target_mode == TARGET_TIME && s_run_time_goal_sec > 0u) {
                        uint32_t rem = g_TM.time;
                        uint32_t gsec = s_run_time_goal_sec;
                        uint32_t n =
                            (rem * (uint32_t)TM_MARQUEE_COUNT + gsec - 1u) / gsec;
                        if (n > TM_MARQUEE_COUNT) n = TM_MARQUEE_COUNT;
                        s_mq_on   = (uint8_t)n;
                        s_mq_tick = 0u;
                        prv_marquee_set(s_mq_on);
                    } else if (g_TM.target_mode == TARGET_PROGRAM) {
                        uint32_t rem = g_TM.time;
                        if (rem > TM_PROG_TOTAL_SEC) rem = TM_PROG_TOTAL_SEC;
                        uint32_t gsec = TM_PROG_TOTAL_SEC;
                        uint32_t n =
                            (rem * (uint32_t)TM_MARQUEE_COUNT + gsec - 1u) / gsec;
                        if (n > TM_MARQUEE_COUNT) n = TM_MARQUEE_COUNT;
                        s_mq_on   = (uint8_t)n;
                        s_mq_tick = 0u;
                        prv_marquee_set(s_mq_on);
                    } else {
                        prv_marquee_set(s_mq_on);
                    }
                } else if (g_TM.target_mode == TARGET_TIME && s_run_time_goal_sec > 0u) {
                    uint32_t rem = g_TM.time;
                    uint32_t gsec = s_run_time_goal_sec;
                    uint32_t n = (rem * (uint32_t)TM_MARQUEE_COUNT + gsec - 1u) / gsec;
                    if (n > TM_MARQUEE_COUNT) n = TM_MARQUEE_COUNT;
                    s_mq_on = (uint8_t)n;
                    s_mq_tick = 0u;
                    prv_marquee_set(s_mq_on);
                } else if (g_TM.target_mode == TARGET_PROGRAM) {
                    uint32_t rem = g_TM.time;
                    if (rem > TM_PROG_TOTAL_SEC) rem = TM_PROG_TOTAL_SEC;
                    uint32_t gsec = TM_PROG_TOTAL_SEC;
                    uint32_t n =
                        (rem * (uint32_t)TM_MARQUEE_COUNT + gsec - 1u) / gsec;
                    if (n > TM_MARQUEE_COUNT) n = TM_MARQUEE_COUNT;
                    s_mq_on = (uint8_t)n;
                    s_mq_tick = 0u;
                    prv_marquee_set(s_mq_on);
                } else {
                /* 停机减速期间：无时间目标时的装饰步进 */
                if (++s_mq_tick < TM_MARQUEE_RUN_STEP) break;
                s_mq_tick = 0u;
                if (s_mq_on < TM_MARQUEE_COUNT) {
                    s_mq_on++;
                } else {
                    s_mq_on = 0u;
                }
                prv_marquee_set(s_mq_on);
                }
            }
            break;

        default:
            break;
    }
}

/* G1-S(13245687) 点亮顺序：S1,S3,S2,S4,S5,S6,S8,S7 → D 位 0,2,1,3,4,5,7,6 */
static const uint8_t s_tm1640_blue_order[8] = {0u, 2u, 1u, 3u, 4u, 5u, 7u, 6u};
/* G14-S(12345678) 顺序对应 D0..D7 */
static const uint8_t s_tm1640_orange_order[8] = {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u};

static uint8_t prv_tm1640_arc_dmask(const uint8_t *order, uint8_t n_on)
{
    uint8_t m = 0u;
    uint8_t i;
    if (n_on > 8u) n_on = 8u;
    for (i = 0u; i < n_on; i++)
        m |= (uint8_t)(1u << order[i]);
    return m;
}

/* 按停首帧：仅快照橙条(卡/累计卡)；蓝条 STOPPING 全程跟速，PAUSED 随显示为 0 */
static void prv_tm1640_snapshot_frozen_cal_now(float cal_mcal, TargetMode_t tmode,
                                               float cal_goal_mcal)
{
    uint8_t n_cal;
    if (tmode == TARGET_CAL) {
        float rem_mcal = cal_mcal;
        float goal_mcal = cal_goal_mcal;
        if (rem_mcal < 0.0f)
            rem_mcal = 0.0f;
        if (goal_mcal <= 0.0f || rem_mcal <= 0.0f) {
            n_cal = 0u;
        } else {
            n_cal = (uint8_t)((rem_mcal * 8.0f + goal_mcal - 1.0f) / goal_mcal);
            if (n_cal > 8u) n_cal = 8u;
        }
    } else {
        float    cum_mcal = cal_mcal;
        uint32_t rem_kcal_u;
        if (cum_mcal < 0.0f)
            cum_mcal = 0.0f;
        rem_kcal_u = (uint32_t)(cum_mcal / 1000.0f + 0.5f);
        if (rem_kcal_u > 999u) rem_kcal_u = 999u;
        if (rem_kcal_u == 0u)
            n_cal = 0u;
        else
            n_cal = (uint8_t)((rem_kcal_u * 8u + 998u) / 999u);
        if (n_cal > 8u) n_cal = 8u;
    }
    s_tm1640_frozen_n_cal      = n_cal;
    s_tm1640_outer_bars_frozen = 1u;
}

/* 运行/停机减速：左蓝条=速度(下控允许 min..max → 1..8 灯)；
 * 右橙条：卡目标模式 RUNNING 时 g_TM.cal=剩余(0.001kcal)，按「剩余/目标×8」向上取整亮灯
 * （设10卡则起步8盏全亮，随消耗递减至0盏）；非卡目标仍用累计卡 0..999kcal→1..8 盏。 */
static void prv_tm1640_speed_cal_bars(void)
{
    uint8_t n_spd;
    uint8_t n_cal;
    _Bool   runish = (g_TM.state == TM_STATE_RUNNING) ||
                     (g_TM.state == TM_STATE_PAUSED) ||
                     (g_TM.state == TM_STATE_STOPPING && g_TM.timer_100ms > 0u);

    UI_SetGridHwOverlay(TM1640_GRID_G1_BLUE, 0xFFu, DISP_OFF);
    UI_SetGridHwOverlay(TM1640_GRID_G14_ORANGE, 0xFFu, DISP_OFF);

    if (!runish)
        return;

    if (s_tm1640_outer_bars_frozen &&
        (g_TM.state == TM_STATE_STOPPING || g_TM.state == TM_STATE_PAUSED)) {
        n_cal = s_tm1640_frozen_n_cal;
        if (g_TM.state == TM_STATE_STOPPING) {
            uint16_t spd = g_TM.speed;
            uint16_t smin = prv_speed_min_eff();
            uint16_t smax = prv_speed_max_eff();
            if (smax <= smin) smax = (uint16_t)(smin + 1u);
            if (spd > smax) spd = smax;
            if (spd < smin) {
                n_spd = 0u;
            } else {
                n_spd = 1u + (uint8_t)(((uint32_t)(spd - smin) * 7u) / (uint32_t)(smax - smin));
                if (n_spd > 8u) n_spd = 8u;
            }
        } else {
            n_spd = 0u; /* PAUSED 与界面速度 0 一致 */
        }
    } else {

    {
        uint16_t spd = (g_TM.state == TM_STATE_PAUSED) ? 0u : g_TM.speed;
        uint16_t smin = prv_speed_min_eff();
        uint16_t smax = prv_speed_max_eff();
        if (smax <= smin) smax = (uint16_t)(smin + 1u);
        if (spd > smax) spd = smax;
        if (spd < smin) {
            n_spd = 0u;
        } else {
            n_spd = 1u + (uint8_t)(((uint32_t)(spd - smin) * 7u) / (uint32_t)(smax - smin));
            if (n_spd > 8u) n_spd = 8u;
        }
    }
    {
        if (g_TM.target_mode == TARGET_CAL) {
            /* 倒计时：cal 从设定目标减到 0；与设置界面数字同源（单位 0.001 kcal） */
            float rem_mcal = g_TM.cal;
            float goal_mcal = s_tm_cal_goal_mcal;
            if (rem_mcal < 0.0f)
                rem_mcal = 0.0f;
            if (goal_mcal <= 0.0f || rem_mcal <= 0.0f) {
                n_cal = 0u;
            } else {
                n_cal = (uint8_t)((rem_mcal * 8.0f + goal_mcal - 1.0f) / goal_mcal);
                if (n_cal > 8u) n_cal = 8u;
            }
        } else {
            float    cum_mcal;
            uint32_t rem_kcal_u;

            cum_mcal = g_TM.cal;
            if (cum_mcal < 0.0f)
                cum_mcal = 0.0f;
            rem_kcal_u = (uint32_t)(cum_mcal / 1000.0f + 0.5f);
            if (rem_kcal_u > 999u) rem_kcal_u = 999u;
            if (rem_kcal_u == 0u)
                n_cal = 0u;
            else
                n_cal = (uint8_t)((rem_kcal_u * 8u + 998u) / 999u); /* ceil：1..999 → 1..8 */
            if (n_cal > 8u) n_cal = 8u;
        }
    }

    }

    UI_SetGridHwOverlay(TM1640_GRID_G1_BLUE,
                        prv_tm1640_arc_dmask(s_tm1640_blue_order, n_spd), DISP_ON);
    UI_SetGridHwOverlay(TM1640_GRID_G14_ORANGE,
                        prv_tm1640_arc_dmask(s_tm1640_orange_order, n_cal), DISP_ON);
}

static void prv_tm1640_function_leds(void)
{
    switch (g_TM.state) {
        case TM_STATE_IDLE:
        case TM_STATE_RUNNING:
        case TM_STATE_PAUSED:
            TM1640_IndSpeed(DISP_ON);
            TM1640_IndTime(DISP_ON);
            if (s_tm1640_shared_cal) {
                TM1640_IndDis(DISP_OFF);
                TM1640_IndCal(DISP_ON);
            } else {
                TM1640_IndCal(DISP_OFF);
                TM1640_IndDis(DISP_ON);
            }
            break;

        case TM_STATE_STOPPING:
            if (g_TM.timer_100ms > 0u) {
                TM1640_IndSpeed(DISP_ON);
                TM1640_IndTime(DISP_ON);
                if (s_tm1640_shared_cal) {
                    TM1640_IndDis(DISP_OFF);
                    TM1640_IndCal(DISP_ON);
                } else {
                    TM1640_IndCal(DISP_OFF);
                    TM1640_IndDis(DISP_ON);
                }
            }
            break;

        case TM_STATE_SETTING:
            /* 设置模式：指示灯跟随当前设置项 */
            TM1640_IndSpeed(DISP_ON);
            TM1640_IndTime(DISP_ON);
            if (g_TM.disp_index == DISP_CAL) {
                TM1640_IndDis(DISP_OFF);
                TM1640_IndCal(DISP_ON);
            } else if (g_TM.disp_index == DISP_PROGRAM) {
                /* P 模式选择：共享区显示程序号，两个灯均熄灭 */
                TM1640_IndDis(DISP_OFF);
                TM1640_IndCal(DISP_OFF);
            } else {
                /* DIST / TIME / SPEED 设置显示路程灯 */
                TM1640_IndCal(DISP_OFF);
                TM1640_IndDis(DISP_ON);
            }
            break;

        case TM_STATE_COUNTDOWN:
            /* 与空闲/运行一致：时间、路程/卡路里灯保持点亮 */
            TM1640_IndSpeed(DISP_ON);
            TM1640_IndTime(DISP_ON);
            if (s_tm1640_shared_cal) {
                TM1640_IndDis(DISP_OFF);
                TM1640_IndCal(DISP_ON);
            } else {
                TM1640_IndCal(DISP_OFF);
                TM1640_IndDis(DISP_ON);
            }
            break;
        case TM_STATE_ERROR:
            TM1640_IndSpeed(DISP_OFF);
            TM1640_IndTime(DISP_OFF);
            TM1640_IndDis(DISP_OFF);
            TM1640_IndCal(DISP_OFF);
            break;
        default:
            break;
    }
}

static void TM_FullDisplay_Render(void) {
    /* 状态切换时清屏（防止跨状态残影）；
     * 同一状态内 disp_index 切换无需清屏，每帧均完整重绘各区 */
    static TM_State_t s_last_state = TM_STATE_BOOT;
    if (g_TM.state != s_last_state) {
        UI_ClearAll();
        s_last_state = g_TM.state;
    }

    UI_SetBlinkTime(1000);

    /* 设置模式下当前选中区域闪烁 */
    bool is_blink = (g_TM.state == TM_STATE_SETTING && g_TM.steady_timer == 0);
    for (uint8_t i = 0; i < 3u; i++) {
        UI_SetGridBlinkMask(TM1640_ZONE_SPEED  + i,
            is_blink && (g_TM.disp_index == DISP_SPEED));
        UI_SetGridBlinkMask(TM1640_ZONE_SHARED + i,
            is_blink && (g_TM.disp_index == DISP_CAL    ||
                         g_TM.disp_index == DISP_DIST   ||
                         g_TM.disp_index == DISP_PROGRAM));
    }
    for (uint8_t i = 0; i < 4u; i++) {
        uint8_t gtime = (uint8_t)(TM1640_ZONE_TIME + i);
        _Bool tblink = is_blink && (g_TM.disp_index == DISP_TIME);
        /* 冒号格：整格 0xFF 会把 DP 与数字一起灭；用 0x7F 只闪 a~g，DP 由末尾 SetBitMode 单独闪 */
        if (gtime == GRID_COLON && tblink)
            UI_SetGridBlinkMaskValue(gtime, 0x7Fu);
        else
            UI_SetGridBlinkMask(gtime, tblink);
    }

    switch (g_TM.state) {

        case TM_STATE_COUNTDOWN: {
            uint8_t cd = (uint8_t)((g_TM.timer_100ms + 9u) / 10u);
            if (cd < 1u) cd = 1u;
            /* 321 不重刷全黑：时间区、共享区照常显示，仅速度区中间为 3/2/1 */
            UI_SetNumber(TM1640_ZONE_TIME, TM_Time_MMSS_For_Display(), 4u, 3u, ZERO_SHOW);
            if (g_TM.disp_index == DISP_PROGRAM) {
                prv_show_program_id(g_TM.prog_index);
            } else if (s_tm1640_shared_cal) {
                UI_SetNumber(TM1640_ZONE_SHARED,
                    (uint16_t)(g_TM.cal / 1000.0f), 3u, 0u, ZERO_HIDE);
            } else {
                UI_SetNumber(TM1640_ZONE_SHARED,
                    TM_Dist_Tenths_For_Display(), 3u, 2u, ZERO_HIDE);
            }
            UI_SetRaw(TM1640_ZONE_SPEED, 0);
            UI_SetNumber(TM1640_ZONE_SPEED + 1u, cd, 1u, 0u, ZERO_HIDE);
            UI_SetRaw(TM1640_ZONE_SPEED + 2u, 0);
            break;
        }

        case TM_STATE_STOPPING:
            if (g_TM.timer_100ms > 0u) {
                /* 暂停减速：时间/路程等保持；仅速度区递减 */
                UI_SetNumber(TM1640_ZONE_SPEED, g_TM.speed, 3u, 2u, ZERO_SHOW);
                UI_SetNumber(TM1640_ZONE_TIME, TM_Time_MMSS_For_Display(), 4u, 3u, ZERO_SHOW);
                if (s_tm1640_shared_cal)
                    UI_SetNumber(TM1640_ZONE_SHARED,
                        (uint16_t)(g_TM.cal / 1000.0f), 3u, 0u, ZERO_HIDE);
                else
                    UI_SetNumber(TM1640_ZONE_SHARED,
                        TM_Dist_Tenths_For_Display(), 3u, 2u, ZERO_HIDE);
            }
            break;

        case TM_STATE_IDLE:
        case TM_STATE_SETTING:
        case TM_STATE_RUNNING:
        case TM_STATE_PAUSED: {
            /* 速度区 */
            /* 速度 0.1km/h：三位显示 xx.x；dot_pos=从右数第几位加 DP（非 GRID 号）。
             * 中间位=逻辑格1→显存 GRID10，DP→SEG1 由 ui_display.h 的 UI_SEG_DP_HW 打包 */
            UI_SetNumber(TM1640_ZONE_SPEED,
                         (g_TM.state == TM_STATE_PAUSED) ? 0u : g_TM.speed,
                         3u, 2u, ZERO_SHOW);

            /* 时间区：
             * - TIME 设置 / 运行 / 空闲：正常显示 g_TM.time
             * - 其他设置项（DIST / CAL / PROGRAM）：显示 00:00 */
            if (g_TM.state == TM_STATE_SETTING && g_TM.disp_index != DISP_TIME) {
                UI_SetNumber(TM1640_ZONE_TIME, 0u, 4u, 3u, ZERO_SHOW);
            } else {
                UI_SetNumber(TM1640_ZONE_TIME, TM_Time_MMSS_For_Display(), 4u, 3u, ZERO_SHOW);
            }

            /* 共享区（路程/卡路里/Program）*/
            if (g_TM.disp_index == DISP_PROGRAM) {
                /* P 模式设置界面：显示 P01-P12 */
                prv_show_program_id(g_TM.prog_index);
            } else if (g_TM.state == TM_STATE_SETTING &&
                       g_TM.disp_index == DISP_CAL) {
                UI_SetNumber(TM1640_ZONE_SHARED,
                    (uint16_t)(g_TM.cal / 1000.0f), 3u, 0u, ZERO_HIDE);
            } else if (g_TM.state == TM_STATE_SETTING &&
                       g_TM.disp_index == DISP_DIST) {
                UI_SetNumber(TM1640_ZONE_SHARED,
                    TM_Dist_Tenths_For_Display(), 3u, 2u, ZERO_HIDE);
            } else {
                /* 运行/空闲：5s 轮换路程与卡路里 */
                if (s_tm1640_shared_cal)
                    UI_SetNumber(TM1640_ZONE_SHARED,
                        (uint16_t)(g_TM.cal / 1000.0f), 3u, 0u, ZERO_HIDE);
                else
                    UI_SetNumber(TM1640_ZONE_SHARED,
                        TM_Dist_Tenths_For_Display(), 3u, 2u, ZERO_HIDE);
            }
            break;
        }

        case TM_STATE_ERROR:
            UI_ClearAll();
            UI_SetRaw(TM1640_ZONE_SPEED, SEG_RAW(SEG_E));
            UI_SetNumber(TM1640_ZONE_SPEED + 1u, g_TM.error_code, 2u, 0u, ZERO_SHOW);
            break;

        default: break;
    }

    /* 时间冒号：显示时间区时持续闪烁（TM1640 上 H/L 均映射为 _SEG_DP，写 H 即可） */
    if (g_TM.state == TM_STATE_IDLE || g_TM.state == TM_STATE_SETTING ||
        g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_PAUSED ||
        g_TM.state == TM_STATE_COUNTDOWN) {
        UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_BLINK);
    } else if (g_TM.state == TM_STATE_STOPPING && g_TM.timer_100ms > 0u) {
        /* 减速时时间区为剩余锻炼时间：冒号常亮不闪 */
        UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_ON);
    }

    prv_tm1640_function_leds();
    prv_marquee_sync_after_leds();
    prv_tm1640_speed_cal_bars();
}

#endif /* LED_DRIVER_TM1640 */


static void TM_UI_Dispatcher(void) {
    if (g_TM.state == TM_STATE_BOOT) {
        /* timer_100ms：每 100ms +1；全亮共 TM_BOOT_FULLON_MS/100 次 */
        const uint16_t boot_full_end =
            (uint16_t)(TM_BOOT_FULLON_MS / 100u + 1u);
        const uint16_t boot_ver1_end =
            (uint16_t)(boot_full_end + TM_BOOT_VERSION_EACH_MS / 100u);
        const uint16_t boot_ver2_end =
            (uint16_t)(boot_ver1_end + TM_BOOT_VERSION_EACH_MS / 100u);
        if (g_TM.timer_100ms == 0) Beep_Play(BEEP_CLICK);
        if (++g_TM.timer_100ms < boot_full_end) {
            for (uint8_t i = 0; i < UI_GRID_MAX; i++) UI_SetRaw(i, 0xFF);
#ifdef BOOT_SHOW_VERSION
        } else if (g_TM.timer_100ms < boot_ver1_end) {
            if (g_TM.timer_100ms == boot_full_end) UI_ClearAll();
            UI_SetRaw(0, SEG_RAW(SEG_U)); UI_SetNumber(1, FW_VER_U, 2, 2, ZERO_SHOW);
        } else if (g_TM.timer_100ms < boot_ver2_end) {
            if (g_TM.timer_100ms == boot_ver1_end) UI_ClearAll();
            UI_SetRaw(0, SEG_RAW(SEG_C)); UI_SetNumber(1, FW_VER_C, 2, 2, ZERO_SHOW);
        } else {
            g_TM.state = TM_STATE_IDLE; g_TM.timer_100ms = 0;
            Treadmill_Reset_Data(); /* 待机：速度/时间/路程/卡等全部清零 */
        }
#else
        } else {
            g_TM.state = TM_STATE_IDLE; g_TM.timer_100ms = 0;
            Treadmill_Reset_Data();
        }
#endif
        return;
    }

#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
    TM_FullDisplay_Render();
    return;
#endif

    static TM_State_t     s_last_state = TM_STATE_BOOT;
    static TM_DispIndex_t s_last_disp  = DISP_SPEED;
    if (g_TM.state != s_last_state || g_TM.disp_index != s_last_disp) {
        UI_ClearAll(); s_last_state = g_TM.state; s_last_disp = g_TM.disp_index;
    }

    for (uint8_t i = 0; i < 6; i++) UI_SetBitMode(GRID_IND, i, DISP_OFF);

    bool is_blink = (g_TM.state == TM_STATE_SETTING && g_TM.steady_timer == 0);
    UI_SetBlinkTime(1000);
    for (uint8_t i = 0; i < 4; i++) UI_SetGridBlinkMask(i, is_blink);

    switch (g_TM.state) {
        case TM_STATE_IDLE:
            UI_SetNumber(0, 0, 4, 0, ZERO_SHOW); /* 0:00 */
            UI_SetBitMode(GRID_IND,   LED_TIME,    DISP_ON);
            UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_BLINK);
            UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_BLINK);
            break;

        case TM_STATE_COUNTDOWN: {
            uint8_t cd = (g_TM.timer_100ms + 9) / 10;
            if (cd < 1) cd = 1;
            UI_SetNumber(0, TM_Time_MMSS_For_Display(), 4, 3, ZERO_SHOW);
            UI_SetNumber(2, cd, 1, 0, ZERO_HIDE);
            UI_SetBitMode(GRID_IND, LED_TIME, DISP_ON);
            UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
            UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_BLINK);
            UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_BLINK);
            break;
        }

        case TM_STATE_STOPPING: {
            if (g_TM.timer_100ms > 0) {
                UI_SetNumber(0, TM_Time_MMSS_For_Display(), 4, 3, ZERO_SHOW);
                uint16_t spd = g_TM.speed;
                if (spd < 10) UI_SetNumber(1, spd, 2, 2, ZERO_SHOW);
                else          UI_SetNumber(0, spd, 3, 2, ZERO_HIDE);
                UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
                UI_SetBitMode(GRID_IND, LED_TIME, DISP_ON);
            } else {
                uint16_t spd = g_TM.speed;
                if (spd < 10) UI_SetNumber(1, spd, 2, 2, ZERO_SHOW);
                else          UI_SetNumber(0, spd, 3, 2, ZERO_HIDE);
                UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
            }
            break;
        }

        case TM_STATE_SETTING:
        case TM_STATE_RUNNING:
        case TM_STATE_PAUSED:
            if (g_TM.disp_index == DISP_SPEED) {
                uint16_t spd = (g_TM.state == TM_STATE_PAUSED) ? 0u : g_TM.speed;
                if (spd < 10) UI_SetNumber(1, spd, 2, 2, ZERO_SHOW);
                else          UI_SetNumber(0, spd, 3, 2, ZERO_HIDE);
                UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
            } else if (g_TM.disp_index == DISP_TIME) {
                uint16_t m = TM_Time_Minutes_For_Display();
                UI_SetNumber(0, m, 4, 0, ZERO_SHOW); /* 0:mm */
                UI_SetBitMode(GRID_IND,   LED_TIME,    DISP_ON);
                UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_BLINK);
                UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_BLINK);
            } else if (g_TM.disp_index == DISP_DIST) {
                UI_SetNumber(0, TM_Dist_Tenths_For_Display(), 3, 2, ZERO_SHOW); /* km 01.0..99.0 */
                UI_SetBitMode(GRID_IND, LED_DIS, DISP_ON);
                UI_SetBitMode(1, 7, is_blink ? DISP_BLINK : DISP_ON);
            } else if (g_TM.disp_index == DISP_CAL) {
                UI_SetNumber(0, (uint32_t)(g_TM.cal / 1000.0f), 4, 0, ZERO_HIDE);
                UI_SetBitMode(GRID_IND, LED_CAL, DISP_ON);
            }
            break;

        case TM_STATE_ERROR:
            UI_SetRaw(0, SEG_RAW(SEG_E));
            UI_SetNumber(1, g_TM.error_code, 2, 0, ZERO_SHOW);
            UI_SetRaw(3, 0);
            break;

        default: break;
    }
}

/* 100ms: state machine + dist/time/cal */
void Treadmill_Manager_100ms(void) {
    static float      s_accum_dist       = 0.0f;
    static TM_State_t s_mgr_prev_state   = TM_STATE_BOOT;
    static uint8_t    s_spd_resend_100ms = 0u;

    if (s_mgr_prev_state != TM_STATE_RUNNING && g_TM.state == TM_STATE_RUNNING) {
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
        s_tm1640_outer_bars_frozen   = 0u;
        s_tm1640_user_stop_snap_done = 0u;
#endif
        s_spd_resend_100ms   = 0u;
        s_lower_spd_rx_tenth = g_TM.speed;
        if (s_run_entry_skip_goal) {
            s_run_entry_skip_goal = 0u;
        } else {
            if (g_TM.target_mode == TARGET_CAL)
                s_tm_cal_goal_mcal = g_TM.cal;
            if (g_TM.target_mode == TARGET_TIME)
                s_run_time_goal_sec = g_TM.time;
            else if (g_TM.target_mode == TARGET_PROGRAM)
                s_run_time_goal_sec = TM_PROG_TOTAL_SEC;
            else
                s_run_time_goal_sec = 0u;
        }
    }
    s_mgr_prev_state = g_TM.state;

    if (g_TM.steady_timer > 0 && --g_TM.steady_timer == 0) UI_MarkDirty();

    if (g_TM.state == TM_STATE_SETTING) {
        if (++s_setting_idle_100ms >= TM_SETTING_IDLE_EXIT_100MS) {
            s_setting_idle_100ms = 0u;
            Treadmill_Reset_Data();
            g_TM.state = TM_STATE_IDLE;
            UI_MarkDirty();
        }
    } else {
        s_setting_idle_100ms = 0u;
    }

    /* 等下控待机：超时强制回待机（与 AeroLink run_status=10 二选一）。
     * 若注释掉此段，取消倒计时/停机后仅依赖串口 10：无应答时会冻结界面、按键失效。 */
    if (g_TM.pending_lower_standby) {
        if (++g_TM.lower_standby_wait_100ms >= TM_LOWER_STANDBY_WAIT_MAX) {
            g_TM.pending_lower_standby    = false;
            g_TM.lower_standby_wait_100ms = 0;
            g_TM.state = TM_STATE_IDLE;
            Treadmill_Reset_Data();
            UI_MarkDirty();
        }
    }

#if TM_ERROR_REPORT_ENABLE
    if (g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_STOPPING) {
        if (++g_comm_timeout_cnt >= COMM_TIMEOUT_COUNT) {
            g_comm_timeout_cnt = 0;
            g_TM.state      = TM_STATE_ERROR;
            g_TM.error_code = TM_ERR_COMM_TIMEOUT;
            if (!Beep_IsBusy())
                Beep_Play(BEEP_ALARM);
        }
    } else {
        g_comm_timeout_cnt = 0;
    }
#else
    g_comm_timeout_cnt = 0;
#endif

    if (g_TM.state == TM_STATE_COUNTDOWN) {
        if (!g_TM.pending_lower_standby) {
            if (g_TM.timer_100ms > 0) {
                g_TM.timer_100ms--;
                if (g_TM.timer_100ms == 20 || g_TM.timer_100ms == 10) Beep_Play(BEEP_CLICK);
                /* 显示「1」时先发 RUN_START；其后若下控未报运行(9)则周期重发 */
                if (g_TM.timer_100ms == 10) {
                    AeroLink_Send(FC_CONTROL, FC_CTRL_SFC_RUN_START, (void*)0);
                    Send_Speed_To_Lower();
                    s_cd_run_resend_100ms = 0;
                } else if (g_TM.timer_100ms < 10) {
                    if (s_lower_last_run_status != 9u) {
                        if (++s_cd_run_resend_100ms >= 3u) {
                            s_cd_run_resend_100ms = 0;
                            AeroLink_Send(FC_CONTROL, FC_CTRL_SFC_RUN_START, (void*)0);
                            Send_Speed_To_Lower();
                        }
                    } else {
                        s_cd_run_resend_100ms = 0;
                    }
                }
            } else {
                _Bool const resuming = g_TM.resume_from_pause;
                g_TM.resume_from_pause = false;
                /* P 程序：首次 321 前已在设置界面写好速度；自暂停恢复时在 PAUSED→321 已写段速 */
                if (g_TM.target_mode != TARGET_PROGRAM)
                    g_TM.speed = prv_speed_min_eff();
                else if (resuming) {
                    g_TM.speed = c_prog_speed[g_TM.prog_index][g_TM.prog_seg];
                    prv_clamp_speed_to_lower();
                }
                g_TM.state       = TM_STATE_RUNNING;
                g_TM.cycle_timer = 0;
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
                s_tm1640_swap_100ms = 0;
                /* 自暂停恢复：保留跑马灯与累计路程基准 */
                if (!resuming) {
                    s_mq_on   = 0u;
                    s_mq_tick = 0u;
                    s_mq_dir  = 1;
                    prv_marquee_set(0u);
                }
#endif
                g_TM.disp_index  = DISP_SPEED;
                if (!resuming) {
                    s_accum_dist = 0.0f;
                    if (g_TM.target_mode == TARGET_TIME) {
                        /* 保持预设倒计时秒数 */
                    } else if (g_TM.target_mode == TARGET_PROGRAM) {
                        g_TM.time = TM_PROG_TOTAL_SEC; /* 30 分钟倒计 */
                    } else {
                        g_TM.time = 0;
                    }
                    if (g_TM.target_mode != TARGET_DIST) g_TM.dist = 0.0f;
                    if (g_TM.target_mode != TARGET_CAL)  g_TM.cal  = 0.0f;
                }
                if (resuming)
                    s_run_entry_skip_goal = 1u;
                if (s_lower_last_run_status != 9u) {
                    AeroLink_Send(FC_CONTROL, FC_CTRL_SFC_RUN_START, (void*)0);
                }
                Send_Speed_To_Lower();
            }
        }
        /* pending_lower_standby：冻结倒计时显示，等下控 10 或超时 */
    }

    if (g_TM.state == TM_STATE_STOPPING) {
        if (g_TM.timer_100ms > 0) {
#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
            if (s_stop_for_pause && !s_tm1640_user_stop_snap_done) {
                prv_tm1640_snapshot_frozen_cal_now(
                    g_TM.cal, g_TM.target_mode, s_tm_cal_goal_mcal);
                s_tm1640_user_stop_snap_done = 1u;
            }
#endif
            uint16_t prev_spd = g_TM.speed;

            /* 下控已回「等待启动」(7)：电机侧已停，上显速度直接清 0 */
            if (s_lower_last_run_status == 7u) {
                if (g_TM.speed > 0u) {
                    g_TM.speed = 0u;
                    Send_Speed_To_Lower();
                }
                s_stop_ramp_err = 0;
            } else {
                uint32_t dur   = (uint32_t)s_stop_ramp_dur;
                int32_t  dur_i = (dur > 0u) ? (int32_t)dur : 1;

                /* Bresenham：每 tick 累加 init，每满 dur 减一档；dur 与进入停机时一致 */
                s_stop_ramp_err += (int32_t)s_stop_initial_spd;
                while (s_stop_ramp_err >= dur_i && g_TM.speed > 0) {
                    g_TM.speed--;
                    s_stop_ramp_err -= dur_i;
                }
                if (g_TM.speed != prev_spd)
                    Send_Speed_To_Lower();

                /* 下控仍报运行(9)时周期补发 STOP，避免偶发停不住 */
                if (s_lower_last_run_status == 9u) {
                    if (++s_stop_cmd_resend_100ms >= 5u) {
                        s_stop_cmd_resend_100ms = 0;
                        AeroLink_Send(FC_CONTROL, FC_CTRL_SFC_STOP, (void*)0);
                    }
                } else {
                    s_stop_cmd_resend_100ms = 0;
                }
            }

            g_TM.timer_100ms--;
            /* 速度先到 0 或倒计时结束：清 0 回待机（不等剩余 tick） */
            if (g_TM.speed == 0u || g_TM.timer_100ms == 0) {
                g_TM.speed = 0u;
                Send_Speed_To_Lower();
                if (s_stop_for_pause) {
                    s_stop_for_pause = 0u;
                    g_TM.state = TM_STATE_PAUSED;
                } else {
                    Treadmill_Reset_Data();
                    g_TM.state = TM_STATE_IDLE;
                }
                UI_MarkDirty();
            } else {
                UI_MarkDirty();
            }
        }
    }

    /* 运行中速度已为 0：与停机结束一致，回待机并清零时间/路程/卡路里/速度 */
    if (g_TM.state == TM_STATE_RUNNING && g_TM.speed == 0u) {
        Send_Speed_To_Lower();
        Treadmill_Reset_Data();
        g_TM.state = TM_STATE_IDLE;
        UI_MarkDirty();
    }

    if (g_TM.state == TM_STATE_RUNNING) {
        if (++s_spd_resend_100ms >= TM_SPEED_RESEND_PERIOD_100MS) {
            s_spd_resend_100ms = 0u;
            if (g_TM.speed != s_lower_spd_rx_tenth)
                Send_Speed_To_Lower();
        }
        static uint8_t sec_cnt = 0;
        if (++sec_cnt >= 10) {
            sec_cnt = 0;
            bool  trigger_stop = false;
            float dist_inc     = (float)g_TM.speed / 36.0f; /* distance delta per 1s tick, m */

            s_accum_dist += dist_inc;

            if (g_TM.target_mode == TARGET_TIME) {
                if (g_TM.time > 0) g_TM.time--;
                if (g_TM.time == 0) trigger_stop = true;
            } else if (g_TM.target_mode == TARGET_PROGRAM) {
                if (g_TM.time > 0) g_TM.time--;
                if (g_TM.time == 0) trigger_stop = true;
                if (!trigger_stop) {
                    uint32_t elapsed = TM_PROG_TOTAL_SEC - g_TM.time;
                    uint8_t want_seg = (uint8_t)(elapsed / (uint32_t)TM_PROG_SEG_SEC);
                    if (want_seg >= (uint8_t)TM_PROG_SEG_COUNT)
                        want_seg = (uint8_t)(TM_PROG_SEG_COUNT - 1u);
                    if (want_seg != g_TM.prog_seg) {
                        g_TM.prog_seg = want_seg;
                        g_TM.speed =
                            c_prog_speed[g_TM.prog_index][g_TM.prog_seg];
                        prv_clamp_speed_to_lower();
                        Send_Speed_To_Lower();
                    }
                }
            } else {
                g_TM.time++;
                if (g_TM.time >= TM_TIME_MAX) trigger_stop = true;
            }
            if (g_TM.target_mode == TARGET_DIST) {
                g_TM.dist -= dist_inc;
                if (g_TM.dist <= 0.0f) trigger_stop = true;
            } else {
                g_TM.dist = s_accum_dist;
            }
            if (g_TM.target_mode == TARGET_CAL) {
                g_TM.cal -= dist_inc * (float)CAL_PER;
                if (g_TM.cal <= 0.0f) trigger_stop = true;
            } else {
                g_TM.cal = s_accum_dist * (float)CAL_PER;
            }

            if (trigger_stop) {
                s_stop_for_pause = 0u;
                s_stop_initial_spd = g_TM.speed;
                s_stop_ramp_err    = 0;
                s_stop_ramp_dur    = prv_stop_ramp_duration_ticks(s_stop_initial_spd);
                g_TM.state = TM_STATE_STOPPING;
                g_TM.pending_lower_standby    = false;
                g_TM.lower_standby_wait_100ms = 0;
                g_TM.timer_100ms        = s_stop_ramp_dur;
                s_stop_cmd_resend_100ms = 0;
                Beep_Play(BEEP_CLICK);
                AeroLink_Send(FC_CONTROL, FC_CTRL_SFC_STOP, (void*)0);
            }
        }
#if (LED_DRIVER_TYPE != LED_DRIVER_TM1640)
        if (g_TM.steady_timer == 0 && ++g_TM.cycle_timer >= 50) {
            g_TM.cycle_timer = 0;
            g_TM.disp_index  = (TM_DispIndex_t)((g_TM.disp_index + 1) % 4);
        }
#endif
    }

#if (LED_DRIVER_TYPE == LED_DRIVER_TM1640)
    /* 路程/卡路里：每 50×100ms=5s 切换；不用 cycle_timer/steady_timer，避免调速后长时间不翻页 */
    {
        TM_State_t st = g_TM.state;
        if (st == TM_STATE_IDLE || st == TM_STATE_SETTING || st == TM_STATE_RUNNING ||
            st == TM_STATE_PAUSED ||
            st == TM_STATE_COUNTDOWN ||
            (st == TM_STATE_STOPPING && g_TM.timer_100ms > 0)) {
            /* P 设置界面固定显示程序号，不做轮换；运行/空闲状态正常5s轮换 */
            if (g_TM.disp_index == DISP_PROGRAM) {
                s_tm1640_swap_100ms = 0;
            } else {
                if (++s_tm1640_swap_100ms >= 50u) {
                    s_tm1640_swap_100ms = 0;
                    s_tm1640_shared_cal ^= 1u;
                    UI_MarkDirty();
                }
            }
        } else {
            s_tm1640_swap_100ms = 0;
        }
    }

    /* 跑马灯更新 */
    prv_marquee_update();
#endif

    TM_UI_Dispatcher();
}

/* 运行信息 Data2：协议为 Hi/Lo；值为 0/3/7/9/10（7=等待启动）。兼容单字节落在 [2] 或 [3]、以及 Lo/Hi 颠倒 */
static uint8_t prv_run_status_from_rx(const AeroRecvFrame_t *f) {
    uint16_t be = ((uint16_t)f->data[2] << 8) | f->data[3];
    if (be <= 10u) return (uint8_t)be;
    uint16_t le = ((uint16_t)f->data[3] << 8) | f->data[2];
    if (le <= 10u) return (uint8_t)le;
    if (f->data[2] <= 10u) return f->data[2];
    if (f->data[3] <= 10u) return f->data[3];
    return f->data[2];
}

void Treadmill_OnSafetyLockOpened(void) {
    /* 协议「安全锁急停」：FC=02 SFC=02，8 数据字节 0；与 AeroLink_Send 一致 */
    AeroLink_Send(FC_CONTROL, FC_CTRL_SFC_SAFETY_ESTOP, (void*)0);
    Treadmill_Reset_Data();
#if TM_ERROR_REPORT_ENABLE
    g_TM.state      = TM_STATE_ERROR;
    g_TM.error_code = TM_ERR_SAFETY_LOCK;
    Beep_Stop();
    Beep_Play(BEEP_LONG);
#else
    g_TM.state = TM_STATE_IDLE;
#endif
    UI_MarkDirty();
}

void Treadmill_OnSafetyLockClosed(void) {
    /* 仅清除本机安全锁故障，不误清下控上报的其它 ERR */
    if (g_TM.state == TM_STATE_ERROR && g_TM.error_code == TM_ERR_SAFETY_LOCK) {
        Treadmill_Reset_Data();
        g_TM.state      = TM_STATE_IDLE;
        g_TM.error_code = 0u;
        Beep_Stop();
        Beep_Play(BEEP_LONG);
        UI_MarkDirty();
    }
}

/* RX: run 0x01/0x00, fault 0x01/0x01 */
void AeroLink_OnFrameReceived(AeroRecvFrame_t *f) {

    g_comm_timeout_cnt = 0;

    if (f->fc == 0x01 && f->sfc == 0x00) {
        uint8_t run_status = prv_run_status_from_rx(f);
        s_lower_last_run_status = run_status;

        /* Data3/Data4：最大/最小物理档，大端 16bit，值域 1～10（全 0=本帧未带）→ 换算为显示 0.6～6.2 并限制设定 */
        {
            uint16_t max_gear = ((uint16_t)f->data[4] << 8) | f->data[5];
            uint16_t min_gear = ((uint16_t)f->data[6] << 8) | f->data[7];
            if ((max_gear != 0u || min_gear != 0u) &&
                max_gear >= (uint16_t)TM_GEAR_WIRE_MIN &&
                min_gear >= (uint16_t)TM_GEAR_WIRE_MIN &&
                max_gear <= (uint16_t)TM_GEAR_WIRE_MAX &&
                min_gear <= (uint16_t)TM_GEAR_WIRE_MAX) {
                if (max_gear < min_gear) {
                    uint16_t sw = max_gear;
                    max_gear      = min_gear;
                    min_gear      = sw;
                }
                s_lower_speed_max_tenth = prv_gear_wire_to_speed_tenth(max_gear);
                s_lower_speed_min_tenth = prv_gear_wire_to_speed_tenth(min_gear);
                prv_clamp_speed_to_lower();
                if (g_TM.state == TM_STATE_RUNNING ||
                    g_TM.state == TM_STATE_STOPPING ||
                    g_TM.state == TM_STATE_COUNTDOWN)
                    Send_Speed_To_Lower();
                UI_MarkDirty();
            }
        }

        if (g_TM.state == TM_STATE_BOOT || g_TM.state == TM_STATE_ERROR) return;

        /* Data1：RPM→0.1km/h；空闲/暂停/设置不写；运行/321 上显主导；STOPPING 线性减速由上控发档位，不写回 g_TM.speed */
        if (g_TM.state != TM_STATE_SETTING && g_TM.state != TM_STATE_IDLE &&
            g_TM.state != TM_STATE_PAUSED) {
            uint16_t rpm = ((uint16_t)f->data[0] << 8) | f->data[1];
            uint32_t spd = ((uint32_t)rpm * (uint32_t)TM_LOWER_RPM_TENTH_NUM +
                            (uint32_t)TM_LOWER_RPM_TENTH_DEN / 2u) /
                           (uint32_t)TM_LOWER_RPM_TENTH_DEN;
            uint16_t smax_rx = prv_speed_max_eff();
            if (spd > (uint32_t)smax_rx) spd = (uint32_t)smax_rx;
            s_lower_spd_rx_tenth = (uint16_t)spd;
        }

        switch (run_status) {
            case 7:
                /* 下控「等待启动」：斜坡停机时勿据此清屏，否则易在 1.0→0.9 间误回待机 */
                break;
            case 9:
                /* 倒计时未结束时不跟下控强行切 RUNNING，否则本机尚未发 RUN_START，状态与协议不一致 */
                if (g_TM.state != TM_STATE_RUNNING && g_TM.state != TM_STATE_STOPPING &&
                    g_TM.state != TM_STATE_COUNTDOWN && g_TM.state != TM_STATE_PAUSED) {
                    g_TM.state = TM_STATE_RUNNING;
                }
                break;
            case 10:
                /* 下控已待机：仅在有「等下控应答」标记时回待机（勿在 RUNNING 时误收 10 打断启动） */
                if (g_TM.pending_lower_standby) {
                    g_TM.pending_lower_standby    = false;
                    g_TM.lower_standby_wait_100ms = 0;
                    g_TM.state = TM_STATE_IDLE;
                    Treadmill_Reset_Data();
                    UI_ClearAll();
                    UI_MarkDirty();
                }
                break;
            case 3:
#if TM_ERROR_REPORT_ENABLE
                g_TM.state = TM_STATE_ERROR;
#endif
                break;
            default: break;
        }
    }
    else if (f->fc == 0x01 && f->sfc == 0x01) {
        uint16_t err_lv1 = ((uint16_t)f->data[0] << 8) | f->data[1];
        uint8_t  err_lv2 = f->data[2];

        if (err_lv1 != 0 || err_lv2 != 0) {
#if TM_ERROR_REPORT_ENABLE
            g_TM.state = TM_STATE_ERROR;
            for (uint8_t i = 0; i < 13; i++) {
                if (err_lv1 & (1u << i)) { g_TM.error_code = i + 1; break; }
            }
            if (err_lv2 & (1 << 0)) g_TM.error_code = TM_ERR_COMM_TIMEOUT;
            if (err_lv2 & (1 << 1)) g_TM.error_code = 15;
            if (!Beep_IsBusy())
                Beep_Play(BEEP_ALARM);
#endif
        }
    }
}
