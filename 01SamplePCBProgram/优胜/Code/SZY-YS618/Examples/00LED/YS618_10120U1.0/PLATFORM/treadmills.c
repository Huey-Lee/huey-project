#include "treadmills.h"
#include "ui_display.h"
#include "plat_segment.h"
#include "plat_beep.h"
#include "plat_keyfun.h"
#include "plat_aerolink.h"
#include "plat_keylink_rx.h"
#include "bsp_gpio.h"


#define TM_STOP_TIME_COUNT    100    /* STOPPING：超时兜底，约 10s */
#define COMM_WATCHDOG_MAX     50     /* E14：下位通信约 5s（50×100ms）未收到 AeroLink 帧 */
#define SPEED_SYNC_TIMEOUT    20    /* 速度同步超时 20×100ms */
#define LOWER_STATUS_WAIT_START  7u  /* 待机 / 等待启动 */
#define TM_PROG_GEAR_DELAY_TICKS 5u /* 程式：先发 TM_SPEED_MIN，500ms 后再发档位目标速 */

/** P01~P08：速度为 0.1km/h 单位，与 g_TM.speed 一致；每段 3min，共 10 段 */
static const uint16_t s_prog_speeds[TM_PROG_NUM][TM_PROG_SEGMENTS] = {
    { 10, 20, 30, 40, 50, 50, 40, 30, 20, 10 },
    { 10, 20, 20, 30, 30, 40, 50, 50, 30, 20 },
    { 10, 20, 50, 20, 50, 30, 50, 40, 60, 20 },
    { 10, 30, 50, 30, 50, 60, 30, 60, 40, 20 },
    { 20, 30, 40, 50, 60, 50, 60, 60, 70, 30 },
    { 20, 30, 50, 70, 70, 60, 80, 80, 50, 30 },
    { 30, 80, 40, 90, 50, 100, 60, 110, 90, 40 },
    { 30, 60, 90, 120, 100, 50, 90, 120, 90, 50 },
};

#define TM_PROG_100MS_PER_SEG  ((uint16_t)(TM_PROG_SEC_PER_SEG * 10u))

TM_Control_t g_TM;

static uint16_t s_watchdog_timer = 0;
static uint16_t s_heartbeat_tick = 0;
static uint16_t s_fbk_speed = 0;     /* 下位机反馈速度 */
static uint8_t  s_lower_status = 0;  /* FC 状态字节 */
static uint8_t  s_cmd_retry_cnt = 0; /* 运行中重发计数（每 100ms 可加） */
static uint16_t s_disp_speed_smooth = 0; /* 停机减速显：半误差跟 s_fbk_speed（同 YS617）；运行速度页仍显 g_TM.speed */
static uint8_t  s_prog_gear_boost_rem = 0; /* 程式起步待发高档：剩余节拍（仅在上一周期已为 RUNNING 时递减） */
static _Bool     s_prog_gear_boost_armed = false;
static TM_State_t s_mgr_prev_exit_stm = TM_STATE_BOOT; /* 上一 Manager 周期结束时的 TM 状态 */
/** 非程式刚进入 RUNNING 后若干拍内：忽略 Keylink 0x30 把目标速从 1.0 拉到更高（防按键板残留 8.0 等） */
static uint8_t  s_keylink_ignore_high_speed_100ms = 0;

/* 指示灯所在逻辑格 GRID_IND=4 */
#define GRID_IND       4   

#define LED_SPEED      4
#define LED_TIME       2
#define LED_DIS        5
#define LED_CAL        6
#define LED_HR         1

#define GRID_COLON     4   
#define SEG_COLON_H    7   // SEG6
#define SEG_COLON_L    0   // SEG5

#define SAFETY_LOCK_DEBOUNCE_HIST_MASK  3u   /* 连续 2 次 100ms 采样一致才翻转 */
#define SAFETY_ES_BURST_ON_ENTER        5u   /* 刚进入 E17：急停 5 字节帧连发次数 */
#define SAFETY_ES_REPEAT_TICKS          1u   /* 故障保持：每 N×100ms 再发一轮 */
#define SAFETY_ES_BURST_EACH_PERIOD     3u   /* 每轮连发次数（多次发送提高下位可靠收到概率） */

static void prv_safety_send_es_burst(uint8_t count)
{
    while (count != 0u) {
        AeroLink_SendEmergencyStopRaw();
        count--;
    }
}

static uint8_t s_safety_hist;
static uint8_t s_safety_fault_latched;
static uint8_t s_safety_es_repeat;
static uint8_t s_safety_trigger_armed;


/* 切入停机：显速从目标与反馈取大起步，再半误差跟下位（YS617 停机倒计时） */
static void prv_smooth_snap_to_fbk(void)
{
    uint16_t v = g_TM.speed;
    if (v < s_fbk_speed)
        v = s_fbk_speed;
    if (v > TM_SPEED_MAX)
        v = TM_SPEED_MAX;
    s_disp_speed_smooth = v;
}

/* 与 YS617 一致：半误差向 s_fbk_speed 靠拢 */
static void TM_DispSpeedSmooth_Step(void)
{
    int32_t err = (int32_t)s_fbk_speed - (int32_t)s_disp_speed_smooth;
    if (err == 0)
        return;

    int32_t step = err / 2;
    if (step == 0)
        step = (err > 0) ? 1 : -1;

    int32_t v = (int32_t)s_disp_speed_smooth + step;
    if (v < 0)
        v = 0;
    if (v > (int32_t)TM_SPEED_MAX)
        v = TM_SPEED_MAX;
    if ((uint16_t)v != s_disp_speed_smooth) {
        s_disp_speed_smooth = (uint16_t)v;
        UI_MarkDirty();
    }
}

/* 下发目标速度到下位（0.1 km/h） */
static void Send_Speed_To_Lower(void) {
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(g_TM.speed >> 8);
    data[1] = (uint8_t)(g_TM.speed & 0xFF);
    AeroLink_Send(FC_GEAR, 0x00, data);
    KeylinkRx_SendTargetSpeed0p1((uint8_t)g_TM.speed);
}

/* 速度数码：运行速度页显目标速；停机显顺滑减速（跟下位反馈，YS617） */
static uint16_t TM_SpeedForUiDigits(void)
{
    if (g_TM.state == TM_STATE_STOPPING) {
        return s_disp_speed_smooth;
    }
    if (g_TM.state == TM_STATE_RUNNING && g_TM.disp_index == DISP_SPEED) {
        uint16_t v = g_TM.speed;
        if (v < TM_SPEED_MIN)
            v = TM_SPEED_MIN;
        return v;
    }
    return g_TM.speed;
}

/** 程式由倒计时或下位提前拉回 RUNNING 时：首档目标先置 TM_SPEED_MIN，再在 Manager 里延时 TM_PROG_GEAR_DELAY_TICKS 后发首段表速 */
static void prv_program_arm_start_gear_ramp(void)
{
    g_TM.time             = TM_PROG_TOTAL_SEC;
    g_TM.prog_segment     = 0;
    g_TM.prog_seg_timer   = 0;
    g_TM.speed            = TM_SPEED_MIN;
    s_prog_gear_boost_armed = true;
    s_prog_gear_boost_rem   = TM_PROG_GEAR_DELAY_TICKS;
}

/* 待机/复位：清零运行相关数据并重绘 */
static void Treadmill_Reset_Data(void) {
    g_TM.time	  = 0;
    g_TM.dist     = 0.0f;
    g_TM.cal      = 0.0f;
    g_TM.speed    = 0;
    g_TM.hr_bpm   = 0;
    
    g_TM.target_mode  = TARGET_NONE;
    g_TM.disp_index   = DISP_TIME;
    g_TM.steady_timer = 0;
    g_TM.cycle_timer  = 0;
    s_fbk_speed       = 0;
    s_lower_status    = 0;
    s_cmd_retry_cnt   = 0;
    s_disp_speed_smooth   = 0;
    s_watchdog_timer      = 0;
    KeylinkRx_ResetWatchdog();
    s_prog_gear_boost_rem = 0;
    s_prog_gear_boost_armed = false;
    s_keylink_ignore_high_speed_100ms = 0;
    g_TM.prog_id          = 0;
    g_TM.prog_segment     = 0;
    g_TM.prog_seg_timer   = 0;
    g_TM.ui_idle_return_100ms = 0;
    UI_ClearAll(); 
}

void Treadmill_Init(void) {
    g_TM.state      = TM_STATE_BOOT; 
    g_TM.boot_phase = BOOT_FULL;
    g_TM.safety_key = true; 
    Treadmill_Reset_Data(); 
}

/* 设定或运行中上下键调节当前显示项 */
static void TM_Adjust_Process(int8_t dir) {
    g_TM.steady_timer = 20; 
    UI_MarkDirty();
    bool is_limit = false;
	
    switch (g_TM.disp_index) {
        case DISP_SPEED:
            if (dir > 0) {
                if (g_TM.speed >= TM_SPEED_MAX) is_limit = true;
                else g_TM.speed += TM_SPEED_STEP;
            } else {
                if (g_TM.speed <= TM_SPEED_MIN) is_limit = true;
                else g_TM.speed -= TM_SPEED_STEP;
            }
            if (g_TM.state == TM_STATE_RUNNING) {
                if (g_TM.target_mode == TARGET_PROGRAM)
                    s_prog_gear_boost_armed = false;
                Send_Speed_To_Lower();
            }						
            break;
        case DISP_TIME:
            if (dir > 0) g_TM.time = (g_TM.time >= TM_TIME_MAX) ? TM_TIME_MIN : (g_TM.time + TM_TIME_STEP);
            else         g_TM.time = (g_TM.time <= TM_TIME_MIN) ? TM_TIME_MAX : (g_TM.time - TM_TIME_STEP);
            break;
        case DISP_DIST:
            if (dir > 0) g_TM.dist = (g_TM.dist >= TM_DIST_MAX) ? TM_DIST_MIN : (g_TM.dist + TM_DIST_STEP);
            else         g_TM.dist = (g_TM.dist <= TM_DIST_MIN) ? TM_DIST_MAX : (g_TM.dist - TM_DIST_STEP);
            break;
        case DISP_CAL:
            if (dir > 0) g_TM.cal = (g_TM.cal >= TM_CAL_MAX) ? TM_CAL_MIN : (g_TM.cal + TM_CAL_STEP);
            else         g_TM.cal = (g_TM.cal <= TM_CAL_MIN) ? TM_CAL_MAX : (g_TM.cal - TM_CAL_STEP);
            break;
        default: break;
    }
    if (is_limit) Beep_Play(BEEP_LIMIT);
}

void Treadmill_On_Event(uint8_t key_id, uint8_t evt) {
    if (evt == K_EVT_RELEASE) return;

    /* E17：仅能通过安全锁恢复，不响应任何按键（含 Error 态 ON/OFF 历史逻辑） */
    if (g_TM.state == TM_STATE_ERROR && g_TM.error_code == TM_ERR_SAFETY_LOCK) {
        (void)key_id;
        return;
    }

    if (g_TM.state != TM_STATE_BOOT && g_TM.state != TM_STATE_ERROR) {
        if ((g_TM.state == TM_STATE_IDLE && g_TM.target_mode == TARGET_PROGRAM) ||
            g_TM.state == TM_STATE_SETTING) {
            g_TM.ui_idle_return_100ms = 0;
        }
    }
	
    if (g_TM.state == TM_STATE_ERROR) {
        if (key_id == K_ID_ONOFF && evt == K_EVT_PRESS) {
            if (g_TM.safety_key && s_watchdog_timer < COMM_WATCHDOG_MAX) {
                g_TM.state = TM_STATE_IDLE;
                Treadmill_Reset_Data();
            }
        }
        return;
    }
		
    if ((key_id == K_ID_MODE || key_id == K_ID_ONOFF) && evt == K_EVT_HOLD) return;
    if ((key_id == K_ID_3 || key_id == K_ID_6 || key_id == K_ID_9 || key_id == K_ID_12 || key_id == K_ID_P) && evt == K_EVT_HOLD) return;
    
    if (g_TM.state == TM_STATE_BOOT) {
        if (key_id == K_ID_ONOFF || key_id == K_ID_MODE) {
            g_TM.state = TM_STATE_IDLE;
            g_TM.timer_100ms = 0; 
            Treadmill_Reset_Data();
            UI_MarkDirty();
        }
        return; 
    } 
    
    switch (g_TM.state) {
        case TM_STATE_IDLE:
            if (key_id == K_ID_UP) return;
            if (key_id == K_ID_DN) {
                if (g_TM.target_mode == TARGET_PROGRAM) {
                    g_TM.target_mode = TARGET_NONE;
                    g_TM.prog_id = 0;
                    UI_MarkDirty();
                }
                return;
            }
            if (key_id == K_ID_3 || key_id == K_ID_6 || key_id == K_ID_9 || key_id == K_ID_12) return;
            if (key_id == K_ID_P && evt == K_EVT_PRESS) {
                if (g_TM.target_mode != TARGET_PROGRAM) {
                    g_TM.target_mode = TARGET_PROGRAM;
                    g_TM.prog_id = 0;
                } else if (g_TM.prog_id + 1u >= TM_PROG_NUM) {
                    g_TM.target_mode = TARGET_NONE;
                    g_TM.prog_id = 0;
                } else {
                    g_TM.prog_id++;
                }
                break;
            }
            if (key_id == K_ID_ONOFF) {
                if (g_TM.target_mode != TARGET_PROGRAM) {
                    Treadmill_Reset_Data();
                } else {
                    g_TM.time = 0;
                    g_TM.dist = 0.0f;
                    g_TM.cal  = 0.0f;
                    g_TM.speed = 0;
                    g_TM.prog_segment   = 0;
                    g_TM.prog_seg_timer   = 0;
                    g_TM.cycle_timer      = 0;
                    g_TM.steady_timer     = 0;
                    g_TM.disp_index       = DISP_TIME;
                }
                g_TM.state = TM_STATE_COUNTDOWN;
                g_TM.timer_100ms = 30;
            }
            else if (key_id == K_ID_MODE) {
                g_TM.state = TM_STATE_SETTING;
                Treadmill_Reset_Data();
                g_TM.disp_index = DISP_TIME;
                g_TM.target_mode = TARGET_TIME;
                g_TM.time = TM_TIME_SET; 
            }
            break;

        case TM_STATE_SETTING:
            if (key_id == K_ID_MODE) {
                if (g_TM.disp_index == DISP_TIME) {
                    g_TM.disp_index = DISP_DIST; g_TM.target_mode = TARGET_DIST;
                    g_TM.dist = TM_DIST_SET;     
                } else if (g_TM.disp_index == DISP_DIST) {
                    g_TM.disp_index = DISP_CAL; g_TM.target_mode = TARGET_CAL;
                    g_TM.cal = TM_CAL_SET;     
                } else {
                    g_TM.state = TM_STATE_IDLE; Treadmill_Reset_Data();
                }
            } 
            else if (key_id == K_ID_UP || key_id == K_ID_DN) TM_Adjust_Process(key_id == K_ID_UP ? 1 : -1);
            else if (key_id == K_ID_ONOFF) { g_TM.state = TM_STATE_COUNTDOWN; g_TM.timer_100ms = 30; }
            break;
                        
        case TM_STATE_COUNTDOWN:
            if (key_id == K_ID_ONOFF) {
                g_TM.state = TM_STATE_IDLE;
                Treadmill_Reset_Data();
            }
            break;
                        
        case TM_STATE_RUNNING: 
            if (key_id == K_ID_ONOFF) { 
                g_TM.state = TM_STATE_STOPPING;
                g_TM.timer_100ms = TM_STOP_TIME_COUNT;
                s_prog_gear_boost_armed = false;
                s_cmd_retry_cnt  = 0;
                prv_smooth_snap_to_fbk();
                AeroLink_Send(FC_CONTROL, 0x01, (void*)0);
            }
            else if (key_id == K_ID_UP || key_id == K_ID_DN) {  
                g_TM.disp_index = DISP_SPEED; g_TM.cycle_timer = 0;
                TM_Adjust_Process(key_id == K_ID_UP ? 1 : -1);
            }
            else if (key_id == K_ID_3 || key_id == K_ID_6 || key_id == K_ID_9 || key_id == K_ID_12) {
                uint16_t preset;
                if (key_id == K_ID_3)       preset = 30;
                else if (key_id == K_ID_6)  preset = 60;
                else if (key_id == K_ID_9)  preset = 90;
                else                        preset = 120;
                g_TM.disp_index = DISP_SPEED;
                g_TM.cycle_timer = 0;
                g_TM.steady_timer = 20;
                g_TM.speed = preset;
                s_prog_gear_boost_armed = false;
                Send_Speed_To_Lower();
            }
            break; /* 运行中 P/M 在 plat_keyfun 已过滤 */
            
        default: break;
    }
    UI_MarkDirty(); 
}

/** 运行轮显：默认速度页；仅 Keylink 心率 >0（握把有有效信号）时插入心率页 */
static TM_DispIndex_t prv_next_run_disp_index(TM_DispIndex_t cur)
{
    if (g_TM.hr_bpm == 0u) {
        switch (cur) {
        case DISP_SPEED: return DISP_TIME;
        case DISP_TIME:  return DISP_DIST;
        case DISP_DIST:  return DISP_CAL;
        case DISP_CAL:   return DISP_SPEED;
        case DISP_HR:
        default:         return DISP_SPEED;
        }
    }
    switch (cur) {
    case DISP_SPEED: return DISP_HR;
    case DISP_HR:    return DISP_TIME;
    case DISP_TIME:  return DISP_DIST;
    case DISP_DIST:  return DISP_CAL;
    case DISP_CAL:   return DISP_SPEED;
    default:         return DISP_SPEED;
    }
}

/* 按状态刷新数码管与指示灯 */
static void TM_UI_Dispatcher(void) {
    if (g_TM.state == TM_STATE_BOOT) {
        if (g_TM.timer_100ms == 0) Beep_Play(BEEP_CLICK); 
        if (++g_TM.timer_100ms < 10) { for(uint8_t i=0; i<6; i++) UI_SetRaw(i, 0xFF); }
#if BOOT_SHOW_VERSION
        else if (g_TM.timer_100ms < 25) {
            if (g_TM.timer_100ms == 10) UI_ClearAll();
            UI_SetRaw(0, SEG_RAW(SEG_U)); UI_SetNumber(1, 10, 2, 2, ZERO_SHOW);
            UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_ON);
        }
        else if (g_TM.timer_100ms < 40) {
            if (g_TM.timer_100ms == 25) UI_ClearAll();
            UI_SetRaw(0, SEG_RAW(SEG_C)); UI_SetNumber(1, 1, 2, 2, ZERO_SHOW);
            UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_ON);
        }
#endif
        else { g_TM.state = TM_STATE_IDLE; g_TM.timer_100ms = 0; }
        return;
    }
    
    static TM_State_t s_last_state = TM_STATE_BOOT;
    static TM_DispIndex_t s_last_disp = DISP_SPEED;

    if (g_TM.state != s_last_state || g_TM.disp_index != s_last_disp) {
        UI_ClearAll(); s_last_state = g_TM.state; s_last_disp = g_TM.disp_index;
    }

    for(uint8_t i=0; i<6; i++) UI_SetBitMode(GRID_IND, i, DISP_OFF); 
    
    bool is_blink = (g_TM.state == TM_STATE_SETTING && g_TM.steady_timer == 0);
    UI_SetBlinkTime(500); /* 冒号：0.5s 亮 + 0.5s 灭 */
    for(uint8_t i=0; i<4; i++) UI_SetGridBlinkMask(i, is_blink);
    
    switch (g_TM.state) {
        case TM_STATE_IDLE:
            if (g_TM.target_mode == TARGET_PROGRAM) {
                UI_SetRaw(1, SEG_RAW(SEG_P));
                UI_SetNumber(2, (uint16_t)(g_TM.prog_id + 1u), 2, 0, ZERO_SHOW);
                UI_SetBitMode(GRID_IND, LED_TIME, DISP_ON);
                UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_OFF);
                UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_OFF);
            } else {
								UI_SetNumber(1, 0, 3, 0, ZERO_SHOW); 
								UI_SetBitMode(GRID_IND, LED_TIME, DISP_ON);
								UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_BLINK);
								UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_BLINK);
            }
            break;

        case TM_STATE_COUNTDOWN: {
            uint8_t cd = (g_TM.timer_100ms + 9) / 10;
            if (cd < 1) cd = 1;
            UI_SetNumber(2, cd, 1, 0, ZERO_HIDE); 
            break;
        }

        case TM_STATE_STOPPING:
				{
					uint16_t speed_val = TM_SpeedForUiDigits();
					if (speed_val < 10) {
						UI_SetNumber(1, speed_val, 2, 0, ZERO_SHOW); 
					} else {
						UI_SetNumber(0, speed_val, 3, 0, ZERO_HIDE);
					}
					UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
					UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_ON);
				}
            break;

        case TM_STATE_SETTING:
        case TM_STATE_RUNNING:
            if ((uint8_t)g_TM.disp_index >= TM_RUN_DISP_CYCLE_LEN) {
                g_TM.disp_index = DISP_SPEED;
            }
            if (g_TM.disp_index == DISP_HR) {
                UI_SetNumber(1, (uint32_t)g_TM.hr_bpm, 3, 0, ZERO_SHOW);
                UI_SetBitMode(GRID_IND, LED_HR, DISP_ON);
                UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_OFF);
                UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_OFF);
            }
            else if (g_TM.disp_index == DISP_SPEED) {
								uint16_t speed_val = TM_SpeedForUiDigits();
								if (speed_val < 10) {
									UI_SetNumber(1, speed_val, 2, 0, ZERO_SHOW); 
								} else {
									UI_SetNumber(0, speed_val, 3, 0, ZERO_HIDE);
								}
								UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
								UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_ON);
            } 
            else if (g_TM.disp_index == DISP_TIME) {
                uint8_t min = g_TM.time / 60; 
								uint8_t sec = g_TM.time % 60;
                uint16_t disp = min * 100 + sec;
                if (min >= 10) UI_SetNumber(0, disp, 4, 2, ZERO_SHOW);
                else UI_SetNumber(1, disp, 3, 2, ZERO_SHOW);
                UI_SetBitMode(GRID_IND, LED_TIME, DISP_ON);
                UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_BLINK);
                UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_BLINK);
            } 
            else if (g_TM.disp_index == DISP_DIST) {
                UI_SetNumber(0, (uint32_t)(g_TM.dist / 10.0f), 4, 3, ZERO_HIDE);
                UI_SetBitMode(GRID_IND, LED_DIS, DISP_ON);
                UI_SetBitMode(GRID_COLON, SEG_COLON_L, is_blink ? DISP_BLINK : DISP_ON);
            } 
            else if (g_TM.disp_index == DISP_CAL) {
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

/* 进入错误态并告警（同码重复不刷屏） */
static void TM_Trigger_Error(uint16_t err_code) {
    if (g_TM.state == TM_STATE_ERROR && g_TM.error_code == err_code)
        return;

    if (g_TM.state != TM_STATE_ERROR || g_TM.error_code != err_code) {
        g_TM.state = TM_STATE_ERROR;
        g_TM.error_code = err_code;
        Beep_Play(BEEP_ALARM);
        UI_MarkDirty();
    }
}

/* Aerolink 一帧回调：踏步数据 + 错误 + 运行状态 */
void AeroLink_OnFrameReceived(AeroFrame_t *f) {
    /* E17：由下位协议与界面全部由安全锁接管，忽略下位帧直至锁恢复 */
    if (g_TM.state == TM_STATE_ERROR && g_TM.error_code == TM_ERR_SAFETY_LOCK)
        return;

    s_watchdog_timer = 0;

    /* FC=0x01 SFC=0x01：下位扩展/错误；心率由按键板 Keylink 0x20 提供 */
    if (f->fc == 0x01 && f->sfc == 0x01) {
        uint16_t err_lv1 = (f->data[0] << 8) | f->data[1];
        uint8_t  err_lv2 = f->data[3];

        if (err_lv1 != 0 || err_lv2 != 0) {
            uint16_t code = 0;
            for (uint8_t i = 0; i < 13; i++) {
                if (err_lv1 & (1 << i)) { code = i + 1; break; }
            }
            if (!code) {
                if (err_lv2 & 0x01) code = 14;
                else if (err_lv2 & 0x02) code = 15;
            }
            if (code) TM_Trigger_Error(code);
            return;
        }
        return;
    }

    /* FC=0x01 SFC=0x00：速度与状态字节 */
    if (g_TM.state == TM_STATE_BOOT || g_TM.state == TM_STATE_ERROR) return;

    if (f->fc == 0x01 && f->sfc == 0x00) {
        s_fbk_speed = (f->data[0] << 8) | f->data[1];
        uint8_t lower_status = f->data[2];
        s_lower_status = lower_status;

        /* 反馈转速 0 且下控等待启动：RUNNING/STOPPING 立即回待机（同上控待机界面），避免 STOPPING 段长时间停在「渐隐到 0」 */
        if (lower_status == LOWER_STATUS_WAIT_START && s_fbk_speed == 0) {
            if (g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_STOPPING) {
                s_disp_speed_smooth = 0;
                g_TM.state = TM_STATE_IDLE;
                Treadmill_Reset_Data();
                UI_MarkDirty();
                return;
            }
        }

        switch (lower_status) {
            case LOWER_STATUS_WAIT_START:
                /* 速度与状态组合见上；此处不按旧逻辑仅凭状态 7 回待机（避免未到零速误判） */
                break;
            case 9:
                /* 运转中：允许从 IDLE/COUNTDOWN 切入 RUNNING */
                if (g_TM.state == TM_STATE_COUNTDOWN || g_TM.state == TM_STATE_IDLE) {
                    /* 下位若在 3-2-1 期间已就绪，会先到 status=9；若不做与倒计时归零相同的程式初始化，
                     * 则不会先发 1.0 再延时发首段速，易出现「显示首段速、电机仍在低速」。 */
                    if (g_TM.state == TM_STATE_COUNTDOWN && g_TM.target_mode == TARGET_PROGRAM) {
                        prv_program_arm_start_gear_ramp();
                        s_cmd_retry_cnt = 0;
                        AeroLink_Send(FC_CONTROL, 0x00, (void*)0);
                        Send_Speed_To_Lower();
                        g_TM.disp_index = DISP_SPEED;
                    }
                    g_TM.state = TM_STATE_RUNNING;
                    g_TM.cycle_timer = 0;
                    g_TM.disp_index = DISP_SPEED;
                }
                break;
            case 10:
                /* 请求减速停止：仅在 RUNNING 时切 STOPPING */
                if (g_TM.state == TM_STATE_RUNNING) {
                    g_TM.state = TM_STATE_STOPPING;
                    g_TM.timer_100ms = TM_STOP_TIME_COUNT;
                    s_prog_gear_boost_armed = false;
                    s_cmd_retry_cnt  = 0;
                    prv_smooth_snap_to_fbk();
                }
                break;
        }
        UI_MarkDirty();
    }
}

/* 主逻辑节拍 100ms */
void Treadmill_Manager_100ms(void)
{
    static float s_accum_dist = 0.0f;

    /* PB00 安全锁：断开（无磁铁，上拉为高）→ E17 + 急停 5 字节；吸合恢复 → 待机清零 */
    {
        uint8_t open = BSP_SafetyLock_Closed() ? 0u : 1u;
        s_safety_hist = (uint8_t)((s_safety_hist << 1) | open) & SAFETY_LOCK_DEBOUNCE_HIST_MASK;
        if (s_safety_hist == SAFETY_LOCK_DEBOUNCE_HIST_MASK)
            s_safety_fault_latched = 1u;
        else if (s_safety_hist == 0u)
            s_safety_fault_latched = 0u;

        if (s_safety_fault_latched) {
            if (!s_safety_trigger_armed) {
                s_safety_trigger_armed = 1u;
                TM_Trigger_Error(TM_ERR_SAFETY_LOCK);
                prv_safety_send_es_burst(SAFETY_ES_BURST_ON_ENTER);
                s_safety_es_repeat = 0u;
            } else if (++s_safety_es_repeat >= SAFETY_ES_REPEAT_TICKS) {
                s_safety_es_repeat = 0u;
                prv_safety_send_es_burst(SAFETY_ES_BURST_EACH_PERIOD);
            }
        } else {
            s_safety_trigger_armed = 0u;
            s_safety_es_repeat = 0u;
            if (g_TM.state == TM_STATE_ERROR && g_TM.error_code == TM_ERR_SAFETY_LOCK) {
                g_TM.state = TM_STATE_IDLE;
                Treadmill_Reset_Data();
                UI_MarkDirty();
            }
        }
    }

    /* 下位 AeroLink 通信看门狗 E14：RUNNING/STOPPING 约 5s 无有效帧则报错（OnFrameReceived 首行喂狗） */
    if (g_TM.state != TM_STATE_ERROR) {
        if (g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_STOPPING) {
            if (s_watchdog_timer < 0xFFFFu)
                s_watchdog_timer++;
            if (s_watchdog_timer >= COMM_WATCHDOG_MAX)
                TM_Trigger_Error(14);
        } else {
            s_watchdog_timer = 0;
        }
    }

    /* 按键板 Keylink 看门狗 E16：RUNNING/STOPPING/COUNTDOWN 约 3s 未收到有效帧 */
    if (g_TM.state != TM_STATE_ERROR) {
        uint8_t mon_kl = (g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_STOPPING ||
                          g_TM.state == TM_STATE_COUNTDOWN) ? 1u : 0u;
        if (KeylinkRx_WatchdogExpired100ms(mon_kl))
            TM_Trigger_Error(16);
    }

    /* 倒计时阶段不发心跳干扰；其它状态约每 500ms 发心跳（E17 时停心跳，避免与急停串扰） */
    if (g_TM.state != TM_STATE_COUNTDOWN &&
        !(g_TM.state == TM_STATE_ERROR && g_TM.error_code == TM_ERR_SAFETY_LOCK)) {
        if (++s_heartbeat_tick >= 5) {
            s_heartbeat_tick = 0;
            AeroLink_Send(FC_HEARTBEAT, 0x00, (void*)0);
        }
    }

    /* RUNNING 且下控 status=待机同步异常：约 500ms 重发启动+目标速。
     * STOPPING：约 500ms 重发停止帧，直至收到待机确认。 */
    if (g_TM.state == TM_STATE_RUNNING && s_lower_status == LOWER_STATUS_WAIT_START) {
        if (++s_cmd_retry_cnt >= 5) {
            s_cmd_retry_cnt = 0;
            AeroLink_Send(FC_CONTROL, 0x00, (void*)0);
            Send_Speed_To_Lower();
        }
    } else if (g_TM.state == TM_STATE_STOPPING) {
        if (++s_cmd_retry_cnt >= 5) {
            s_cmd_retry_cnt = 0;
            AeroLink_Send(FC_CONTROL, 0x01, (void*)0);
        }
    } else {
        s_cmd_retry_cnt = 0;
    }

    if (g_TM.steady_timer > 0 && --g_TM.steady_timer == 0)
        UI_MarkDirty();

    /* 倒计时 3–2–1：timer 自减，关键点蜂鸣 */
    if (g_TM.state == TM_STATE_COUNTDOWN) {
        if (g_TM.timer_100ms > 0) {
            g_TM.timer_100ms--;
            if (g_TM.timer_100ms == 20 || g_TM.timer_100ms == 10)
                Beep_Play(BEEP_CLICK);
        } else {
            /* 归零：按模式加载初值并切 RUNNING，清重发计数等 */
            s_accum_dist = 0.0f;
            if (g_TM.target_mode == TARGET_PROGRAM) {
                prv_program_arm_start_gear_ramp();
            } else {
                if (g_TM.target_mode != TARGET_TIME) g_TM.time = 0;
                if (g_TM.target_mode != TARGET_DIST) g_TM.dist = 0.0f;
                if (g_TM.target_mode != TARGET_CAL)  g_TM.cal  = 0.0f;
                g_TM.speed = TM_SPEED_MIN;
            }
            g_TM.state       = TM_STATE_RUNNING;
            g_TM.cycle_timer = 0;
            g_TM.disp_index  = DISP_SPEED;
            s_cmd_retry_cnt  = 0;
            AeroLink_Send(FC_CONTROL, 0x00, (void*)0);
            Send_Speed_To_Lower();
        }
    }

    /* 新切入 RUNNING：转速顺滑缓冲与反馈对齐（YS617）；非程式起步窗防 Keylink 抬高 */
    if (g_TM.state == TM_STATE_RUNNING && s_mgr_prev_exit_stm != TM_STATE_RUNNING) {
        s_disp_speed_smooth = s_fbk_speed;
        if (g_TM.target_mode != TARGET_PROGRAM)
            s_keylink_ignore_high_speed_100ms = 30u; /* 约 3s 内拒 Keylink 抬高到 >1.0 */
    }

    if (s_keylink_ignore_high_speed_100ms > 0u)
        s_keylink_ignore_high_speed_100ms--;

    /* 程式起动：延后约 500ms 发送到首段目标速（不落在一进入 RUNNING 的那一帧递减） */
    if (g_TM.state == TM_STATE_RUNNING && g_TM.target_mode == TARGET_PROGRAM && s_prog_gear_boost_armed &&
        s_prog_gear_boost_rem > 0 && s_mgr_prev_exit_stm == TM_STATE_RUNNING) {
        s_prog_gear_boost_rem--;
        if (s_prog_gear_boost_rem == 0) {
            g_TM.speed = s_prog_speeds[g_TM.prog_id][g_TM.prog_segment];
            Send_Speed_To_Lower();
            s_prog_gear_boost_armed = false;
            UI_MarkDirty();
        }
    }

    /* 运行中：秒节拍、程式段切换；约 5s 轮显：速度为首；无心率时无「心率页」，握把有效后插入心率 */
    if (g_TM.state == TM_STATE_RUNNING) {
        static uint8_t sec_cnt = 0;

        if (g_TM.hr_bpm == 0u && g_TM.disp_index == DISP_HR) {
            g_TM.disp_index = DISP_SPEED;
            UI_MarkDirty();
        }

        if (g_TM.target_mode == TARGET_PROGRAM) {
            if (++g_TM.prog_seg_timer >= TM_PROG_100MS_PER_SEG) {
                g_TM.prog_seg_timer = 0;
                if ((uint8_t)(g_TM.prog_segment + 1u) < TM_PROG_SEGMENTS) {
                    g_TM.prog_segment++;
                    g_TM.speed = s_prog_speeds[g_TM.prog_id][g_TM.prog_segment];
                    g_TM.steady_timer = 20;
                    Send_Speed_To_Lower();
                    Beep_Play(BEEP_PROG_SHIFT);
                    UI_MarkDirty();
                }
            }
        }

        if (++sec_cnt >= 10) {
            sec_cnt = 0;
            bool trigger_stop = false;
            float dist_inc = (float)g_TM.speed / 36.0f;
            s_accum_dist += dist_inc;
            if (g_TM.target_mode == TARGET_TIME || g_TM.target_mode == TARGET_PROGRAM) {
                if (g_TM.time > 0) g_TM.time--; if (g_TM.time == 0) trigger_stop = true;
            } else { g_TM.time++; if (g_TM.time >= TM_TIME_MAX) trigger_stop = true; }
            if (g_TM.target_mode == TARGET_DIST) {
                g_TM.dist -= dist_inc; if (g_TM.dist <= 0.0f) trigger_stop = true;
            } else { g_TM.dist = s_accum_dist; }
            if (g_TM.target_mode == TARGET_CAL) {
                float cal_dec = dist_inc * CAL_PER;
                g_TM.cal -= cal_dec; if (g_TM.cal <= 0.0f) trigger_stop = true;
            } else { g_TM.cal = s_accum_dist * CAL_PER; }

            if (trigger_stop) {
                g_TM.state = TM_STATE_STOPPING;
                g_TM.timer_100ms = TM_STOP_TIME_COUNT;
                s_prog_gear_boost_armed = false;
                s_cmd_retry_cnt  = 0;
                prv_smooth_snap_to_fbk();
                Beep_Play(BEEP_CLICK);
                AeroLink_Send(FC_CONTROL, 0x01, (void*)0);
            }
        }
        if (g_TM.steady_timer == 0 && ++g_TM.cycle_timer >= 50) {
            g_TM.cycle_timer = 0;
            g_TM.disp_index  = prv_next_run_disp_index(g_TM.disp_index);
        }
    }

    /* 停机：先判零速/超时回待机，再做顺滑；避免下位已 0 仍多拍减速动画、数码管长时间停在 0x.x 再切待机 */
    if (g_TM.state == TM_STATE_STOPPING) {
        if (s_fbk_speed == 0u) {
            g_TM.speed = 0;
            g_TM.state = TM_STATE_IDLE;
            Treadmill_Reset_Data();
        } else if (g_TM.timer_100ms > 0) {
            g_TM.timer_100ms--;
            TM_DispSpeedSmooth_Step();
        } else {
            g_TM.speed = 0;
            g_TM.state = TM_STATE_IDLE;
            Treadmill_Reset_Data();
        }
    }

    if ((g_TM.state == TM_STATE_IDLE && g_TM.target_mode == TARGET_PROGRAM) ||
        g_TM.state == TM_STATE_SETTING) {
        if (++g_TM.ui_idle_return_100ms >= TM_UI_IDLE_RETURN_TICKS) {
            g_TM.ui_idle_return_100ms = 0;
            g_TM.state = TM_STATE_IDLE;
            Treadmill_Reset_Data();
            UI_MarkDirty();
        }
    } else {
        g_TM.ui_idle_return_100ms = 0;
    }

    s_mgr_prev_exit_stm = g_TM.state;

    TM_UI_Dispatcher();
}

void Treadmill_Keylink_SetSpeed0p1(uint16_t v)
{
    if (g_TM.state == TM_STATE_ERROR && g_TM.error_code == TM_ERR_SAFETY_LOCK)
        return;
    if (v < TM_SPEED_MIN)
        v = TM_SPEED_MIN;
    if (v > TM_SPEED_MAX)
        v = TM_SPEED_MAX;
    /* 程式首档爬坡窗内：忽略按键板上发的「高于首档」目标速，避免显示先跳到首段表速而下位仍只为 1.0 */
    if (g_TM.state == TM_STATE_RUNNING && g_TM.target_mode == TARGET_PROGRAM && s_prog_gear_boost_armed &&
        v > TM_SPEED_MIN)
        return;
    /* 非程式起步窗：丢弃 Keylink 把显示「抬高到高于当前目标」的帧（多按键板残留 8.0 在 1.0 之后到达） */
    if (g_TM.state == TM_STATE_RUNNING && g_TM.target_mode != TARGET_PROGRAM &&
        s_keylink_ignore_high_speed_100ms > 0u && v > TM_SPEED_MIN && v > g_TM.speed)
        return;
    g_TM.speed = v;
    if (g_TM.state == TM_STATE_RUNNING) {
        uint8_t data[8] = {0};
        data[0] = (uint8_t)(g_TM.speed >> 8);
        data[1] = (uint8_t)(g_TM.speed & 0xFF);
        AeroLink_Send(FC_GEAR, 0x00, data);
    }
    KeylinkRx_SendTargetSpeed0p1((uint8_t)g_TM.speed);
    UI_MarkDirty();
}

void Treadmill_Keylink_ApplyHotkey(uint16_t speed_0p1)
{
    if (g_TM.state == TM_STATE_ERROR && g_TM.error_code == TM_ERR_SAFETY_LOCK)
        return;
    if (g_TM.state != TM_STATE_RUNNING)
        return;
    if (speed_0p1 < TM_SPEED_MIN)
        speed_0p1 = TM_SPEED_MIN;
    if (speed_0p1 > TM_SPEED_MAX)
        speed_0p1 = TM_SPEED_MAX;
    g_TM.disp_index = DISP_SPEED;
    g_TM.cycle_timer = 0;
    g_TM.steady_timer = 20;
    g_TM.speed = speed_0p1;
    s_prog_gear_boost_armed = false;
    {
        uint8_t data[8] = {0};
        data[0] = (uint8_t)(g_TM.speed >> 8);
        data[1] = (uint8_t)(g_TM.speed & 0xFF);
        AeroLink_Send(FC_GEAR, 0x00, data);
    }
    KeylinkRx_SendTargetSpeed0p1((uint8_t)g_TM.speed);
    UI_MarkDirty();
}

void Treadmill_Keylink_SetHeartRate(uint8_t bpm)
{
    if (g_TM.state == TM_STATE_ERROR && g_TM.error_code == TM_ERR_SAFETY_LOCK)
        return;
    if (bpm > 250u)
        return;
    uint16_t prev = g_TM.hr_bpm;
    if (g_TM.hr_bpm != (uint16_t)bpm) {
        g_TM.hr_bpm = bpm;
        /* 运行中一旦从无心率变为有效：立即优先显示心率页，完整 5s（50×100ms）后再按序轮显下一项 */
        if (g_TM.state == TM_STATE_RUNNING && bpm > 0u && prev == 0u) {
            g_TM.disp_index   = DISP_HR;
            g_TM.cycle_timer  = 0;
            g_TM.steady_timer = 0;
        }
        if (g_TM.state == TM_STATE_RUNNING)
            UI_MarkDirty();
    }
}
