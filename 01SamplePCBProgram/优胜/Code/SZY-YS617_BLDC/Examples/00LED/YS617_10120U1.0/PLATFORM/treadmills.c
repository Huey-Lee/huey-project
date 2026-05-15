#include "treadmills.h"
#include "ui_display.h"
#include "plat_segment.h"
#include "plat_beep.h"
#include "plat_keyfun.h"
#include "plat_aerolink.h"


#define TM_STOP_TIME_COUNT    100    /* STOPPING：timer_100ms 超时兜底，约 100×100ms≈10s；仍未收到 fbk==0 则强制待机（显示已改为跟顺滑反馈速，不用此常量做插值） */
#define COMM_WATCHDOG_MAX     30     /* 通信看门狗约 3s（若启用） */
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
static uint16_t s_disp_speed_smooth = 0; /* RUNNING：转速显示的顺滑跟随值 → s_fbk_speed */
static uint8_t  s_prog_gear_boost_rem = 0; /* 程式起步待发高档：剩余节拍（仅在上一周期已为 RUNNING 时递减） */
static _Bool     s_prog_gear_boost_armed = false;
static TM_State_t s_mgr_prev_exit_stm = TM_STATE_BOOT; /* 上一 Manager 周期结束时的 TM 状态 */

/* 指示灯所在逻辑格 GRID_IND=4 */
#define GRID_IND       4   

#define LED_STEP       1   // SEG2
#define LED_SPEED      3   // SEG2
#define LED_TIME       2   // SEG1
#define LED_DIS        4   // SEG3
#define LED_CAL        5   // SEG4
#define LED_BLUE       6   // SEG2

#define GRID_COLON     4   
#define SEG_COLON_H    7   // SEG6
#define SEG_COLON_L    0   // SEG5



/* 下发目标速度到下位（0.1 km/h） */
static void Send_Speed_To_Lower(void) {
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(g_TM.speed >> 8);
    data[1] = (uint8_t)(g_TM.speed & 0xFF);
    AeroLink_Send(FC_GEAR, 0x00, data);
	
}

/* 转速数码顺滑（运行速度页或停机减速）：跟踪 s_fbk_speed，弱化通信瞬时跳变（如 3→1） */
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

/* 速度数码：运行速度页、停机减速均跟顺滑反馈速；仅运行中且不处于停机时保底 1.0km/h 起显 */
static uint16_t TM_SpeedForUiDigits(void)
{
    if (g_TM.state == TM_STATE_STOPPING) {
        return s_disp_speed_smooth;
    }
    if (g_TM.state == TM_STATE_RUNNING && g_TM.disp_index == DISP_SPEED) {
        uint16_t v = s_disp_speed_smooth;
        if (v < TM_SPEED_MIN)
            v = TM_SPEED_MIN;
        return v;
    }
    return g_TM.speed;
}

/* 待机/复位：清零运行相关数据并重绘 */
static void Treadmill_Reset_Data(void) {
    g_TM.time	  = 0;
    g_TM.dist     = 0.0f;
    g_TM.cal      = 0.0f;
    g_TM.speed    = 0;
    g_TM.step_rate = 0;
    
    g_TM.target_mode  = TARGET_NONE;
    g_TM.disp_index   = DISP_TIME;
    g_TM.steady_timer = 0;
    g_TM.cycle_timer  = 0;
    s_fbk_speed       = 0;
    s_lower_status    = 0;
    s_cmd_retry_cnt   = 0;
    s_disp_speed_smooth   = 0;
    s_prog_gear_boost_rem = 0;
    s_prog_gear_boost_armed = false;
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
        case DISP_STEP:
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
            if (g_TM.disp_index == DISP_STEP) {
                /* 四位右对齐消隐：1~9→xxxN，10~99→xxNN，100~999→xNNN，≥1000→NNNN */
                UI_SetNumber(0, g_TM.step_rate, 4, 0, ZERO_HIDE);
                UI_SetBitMode(GRID_IND, LED_STEP, DISP_ON);
                UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_OFF);
                UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_OFF);
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
    s_watchdog_timer = 0;

    /* FC=0x01 SFC=0x01：Data4/5 为步频等 */
    if (f->fc == 0x01 && f->sfc == 0x01) {
        g_TM.step_rate = (uint16_t)((f->data[4] << 8) | f->data[5]);

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
        if (g_TM.state == TM_STATE_RUNNING) UI_MarkDirty();
        return;
    }

    /* FC=0x01 SFC=0x00：速度与状态字节 */
    if (g_TM.state == TM_STATE_BOOT || g_TM.state == TM_STATE_ERROR) return;

    if (f->fc == 0x01 && f->sfc == 0x00) {
        s_fbk_speed = (f->data[0] << 8) | f->data[1];
        uint8_t lower_status = f->data[2];
        s_lower_status = lower_status;

        /* 反馈转速 0 且下控待机(等待启动)：立即切待机界面，不靠其它状态迂回 */
        if (lower_status == LOWER_STATUS_WAIT_START && s_fbk_speed == 0) {
            if (g_TM.state == TM_STATE_RUNNING || g_TM.state == TM_STATE_STOPPING ||
                g_TM.state == TM_STATE_COUNTDOWN) {
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
                    g_TM.state = TM_STATE_RUNNING;
                    g_TM.cycle_timer = 0;
                    g_TM.disp_index  = DISP_SPEED;
                }
                break;
            case 10:
                /* 请求减速停止：仅在 RUNNING 时切 STOPPING */
                if (g_TM.state == TM_STATE_RUNNING) {
                    g_TM.state = TM_STATE_STOPPING;
                    g_TM.timer_100ms = TM_STOP_TIME_COUNT;
                    s_prog_gear_boost_armed = false;
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

/* 可选：安全开关 E01（当前注释掉）
//      if (!g_TM.safety_key && g_TM.state != TM_STATE_BOOT)
//          TM_Trigger_Error(1);
//
//      通信超时 E14（当前注释掉）
*/

    /* 倒计时阶段不发心跳干扰；其它状态约每 500ms 发心跳 */
    if (g_TM.state != TM_STATE_COUNTDOWN) {
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
                g_TM.time = TM_PROG_TOTAL_SEC;
                g_TM.prog_segment   = 0;
                g_TM.prog_seg_timer = 0;
                g_TM.speed = TM_SPEED_MIN;
                s_prog_gear_boost_armed = true;
                s_prog_gear_boost_rem = TM_PROG_GEAR_DELAY_TICKS;
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

    /* 新切入 RUNNING：转速显示从下位反馈拉齐一格 */
    if (g_TM.state == TM_STATE_RUNNING && s_mgr_prev_exit_stm != TM_STATE_RUNNING)
        s_disp_speed_smooth = s_fbk_speed;

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

    /* 运行中：秒节拍、程式段切换；约 5s 轮显：速度-时间-路程-卡路里-步频 */
    if (g_TM.state == TM_STATE_RUNNING) {
        static uint8_t sec_cnt = 0;

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
                Beep_Play(BEEP_CLICK);
                AeroLink_Send(FC_CONTROL, 0x01, (void*)0);
            }
        }
        if (g_TM.steady_timer == 0 && ++g_TM.cycle_timer >= 50) {
            g_TM.cycle_timer = 0;
            g_TM.disp_index = (TM_DispIndex_t)((g_TM.disp_index + 1u) % TM_RUN_DISP_CYCLE_LEN);
        }
    }

    /* 运行速度页 / 停机：数码顺滑跟踪下位反馈，避免 0.x 级瞬时跳变 */
    if ((g_TM.state == TM_STATE_RUNNING && g_TM.disp_index == DISP_SPEED) ||
        g_TM.state == TM_STATE_STOPPING) {
        TM_DispSpeedSmooth_Step();
    }

    /* 停机：反馈零速或超时回待机；数码用 TM_SpeedForUiDigits（顺滑跟 s_fbk_speed） */
    if (g_TM.state == TM_STATE_STOPPING) {
        if (s_fbk_speed == 0) {
            g_TM.speed = 0;
            g_TM.state = TM_STATE_IDLE;
            Treadmill_Reset_Data();
        } else if (g_TM.timer_100ms > 0) {
            g_TM.timer_100ms--;
        } else {
            /* 超时仍未为零速则强制待机 */
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



