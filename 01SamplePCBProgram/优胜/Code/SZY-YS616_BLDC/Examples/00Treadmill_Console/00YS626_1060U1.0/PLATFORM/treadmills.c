#include "treadmills.h"
#include "ui_display.h"
#include "plat_segment.h"
#include "plat_beep.h"
#include "plat_keyfun.h"
#include "plat_aerolink.h"


/* ???????? */
#define TM_STOP_TIME_COUNT    100    // ?????????50 * 100ms = 5 ?? (?????????)
#define COMM_WATCHDOG_MAX     30	  // 3?????????
#define SPEED_SYNC_TIMEOUT    20    // 2???????????? (20 * 100ms)


TM_Control_t g_TM;

// ???????????
static uint16_t s_watchdog_timer = 0; // ???????(ms)
static uint16_t s_heartbeat_tick = 0; // ??????????
static uint16_t s_fbk_speed = 0;     // ?????????????
static uint8_t  s_lower_status = 0;  // ??????????????????????
static uint8_t  s_cmd_retry_cnt = 0; // ????/????????????????100ms??????

/* ?????????? (GRID4) */
#define GRID_IND       4   
#define LED_SPEED      1   // SEG2
#define LED_TIME       0   // SEG1
#define LED_DIS        2   // SEG3
#define LED_CAL        3   // SEG4

#define GRID_COLON     4   
#define SEG_COLON_H    5   // SEG6
#define SEG_COLON_L    4   // SEG5



/**
 * @brief ????????????????????????
 */
static void Send_Speed_To_Lower(void) {
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(g_TM.speed >> 8);   // Data1 ????
    data[1] = (uint8_t)(g_TM.speed & 0xFF); // Data1 ????
    AeroLink_Send(FC_GEAR, 0x00, data);
	
}

/**
 * @brief ???????????
 */
static void Treadmill_Reset_Data(void) {
    g_TM.time	  = 0;
    g_TM.dist     = 0.0f;
    g_TM.cal      = 0.0f;
    g_TM.speed    = 0;
    
    g_TM.target_mode  = TARGET_NONE;
    g_TM.disp_index   = DISP_TIME;
    g_TM.steady_timer = 0;
    g_TM.cycle_timer  = 0;
    s_fbk_speed       = 0;
    s_lower_status    = 0;
    s_cmd_retry_cnt   = 0;
    UI_ClearAll(); 
}

/**
 * @brief ???????
 */
void Treadmill_Init(void) {
    g_TM.state      = TM_STATE_BOOT; 
    g_TM.boot_phase = BOOT_FULL;
    g_TM.safety_key = true; 
    Treadmill_Reset_Data(); 
}

/**
 * @brief ??????????? (?????)
 */
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

/**
 * @brief ??????????
 */
void Treadmill_On_Event(uint8_t key_id, uint8_t evt) {
    if (evt == K_EVT_RELEASE) return;
	
		// ?????????????????????
    if (g_TM.state == TM_STATE_ERROR) {
        if (key_id == K_ID_ONOFF && evt == K_EVT_PRESS) {
            // ?????????????????????????????????????
            if (g_TM.safety_key && s_watchdog_timer < COMM_WATCHDOG_MAX) {
                g_TM.state = TM_STATE_IDLE;
                Treadmill_Reset_Data();
            }
        }
        return;
    }
		
    if ((key_id == K_ID_MODE || key_id == K_ID_ONOFF) && evt == K_EVT_HOLD) return;
    
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
            if (key_id == K_ID_UP || key_id == K_ID_DN) return; 
            if (key_id == K_ID_ONOFF) {
                Treadmill_Reset_Data();
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
            if (key_id == K_ID_ONOFF) { g_TM.state = TM_STATE_IDLE; Treadmill_Reset_Data(); }
            break;
                        
        case TM_STATE_RUNNING: 
            if (key_id == K_ID_ONOFF) { 
                g_TM.state = TM_STATE_STOPPING;
                g_TM.timer_100ms = TM_STOP_TIME_COUNT;
                s_cmd_retry_cnt  = 0;
                AeroLink_Send(FC_CONTROL, 0x01, (void*)0); // s_cmd_retry_cnt ??????????
            }
            else if (key_id == K_ID_UP || key_id == K_ID_DN) {  
                g_TM.disp_index = DISP_SPEED; g_TM.cycle_timer = 0;
                TM_Adjust_Process(key_id == K_ID_UP ? 1 : -1);
            }                           
            break;
            
        default: break;
    }
    UI_MarkDirty(); 
}

/**
 * @brief UI ???????????
 */
static void TM_UI_Dispatcher(void) {
    if (g_TM.state == TM_STATE_BOOT) {
        if (g_TM.timer_100ms == 0) Beep_Play(BEEP_CLICK); 
        if (++g_TM.timer_100ms < 10) { for(uint8_t i=0; i<6; i++) UI_SetRaw(i, 0xFF); } 
        else if (g_TM.timer_100ms < 25) { 
            if (g_TM.timer_100ms == 10) UI_ClearAll();
            UI_SetRaw(0, SEG_RAW(SEG_U)); UI_SetNumber(1, 10, 2, 2, ZERO_SHOW); // ???????
            UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_ON);
        } 
        else if (g_TM.timer_100ms < 40) { 
            if (g_TM.timer_100ms == 25) UI_ClearAll();
            UI_SetRaw(0, SEG_RAW(SEG_C)); UI_SetNumber(1, 1, 2, 2, ZERO_SHOW); // ???????
            UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_ON);
        } 
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
    UI_SetBlinkTime(1000);
    for(uint8_t i=0; i<4; i++) UI_SetGridBlinkMask(i, is_blink);
    
    switch (g_TM.state) {
        case TM_STATE_IDLE:
            UI_SetNumber(1, 0, 3, 0, ZERO_SHOW); 
            UI_SetBitMode(GRID_IND, LED_TIME, DISP_ON);
            UI_SetBitMode(GRID_COLON, SEG_COLON_H, DISP_BLINK);
            UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_BLINK);
            break;

        case TM_STATE_COUNTDOWN: {
            uint8_t cd = (g_TM.timer_100ms + 9) / 10;
            if (cd < 1) cd = 1;
            UI_SetNumber(2, cd, 1, 0, ZERO_HIDE); 
            break;
        }

        case TM_STATE_STOPPING: 
            // ??????????????????????
			{
				uint16_t speed_val = g_TM.speed;
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
            if (g_TM.disp_index == DISP_SPEED) {
				uint16_t speed_val = g_TM.speed;
				if (speed_val < 10) {
					UI_SetNumber(1, speed_val, 2, 0, ZERO_SHOW); 
				} else {
					UI_SetNumber(0, speed_val, 3, 0, ZERO_HIDE);
				}
				UI_SetBitMode(GRID_IND, LED_SPEED, DISP_ON);
				UI_SetBitMode(GRID_COLON, SEG_COLON_L, DISP_ON); // ??????
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

/**
 * @brief ??????????????
 * @param err_code ??????
 * ?????????????????????????????????????????
 */
static void TM_Trigger_Error(uint16_t err_code) {
    // ??????????????????????????????????????
    if (g_TM.state == TM_STATE_ERROR && g_TM.error_code == err_code) return;

    // ???????????????????????????????????????????????????
    if (g_TM.state != TM_STATE_ERROR || g_TM.error_code != err_code) {
        g_TM.state = TM_STATE_ERROR;
        g_TM.error_code = err_code;
        Beep_Play(BEEP_ALARM); // ??????????????????
        UI_MarkDirty();
    }
}

/**
 * @brief ???????????????????????????????
 */
void AeroLink_OnFrameReceived(AeroFrame_t *f) {
    s_watchdog_timer = 0; // ?????????

    /* 1. ??????????? (????????) */
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
            if (code) TM_Trigger_Error(code); // ????????????
            return; 
        }
    }

    /* 2. ??????????? */
    if (g_TM.state == TM_STATE_BOOT || g_TM.state == TM_STATE_ERROR) return;

    if (f->fc == 0x01 && f->sfc == 0x00) {
        s_fbk_speed = (f->data[0] << 8) | f->data[1]; 
        uint8_t lower_status = f->data[2];
        s_lower_status = lower_status; // ??????????????????????????

        switch (lower_status) {
            case 7:  // ????/?????? ?? ???????????????
                // RUNNING / STOPPING / COUNTDOWN ??? status==7 ?????? IDLE
                // COUNTDOWN ???????????????/??????????????????
                if (g_TM.state == TM_STATE_RUNNING  ||
                    g_TM.state == TM_STATE_STOPPING  ||
                    g_TM.state == TM_STATE_COUNTDOWN) {
                    g_TM.state = TM_STATE_IDLE;
                    Treadmill_Reset_Data();
                }
                break;
            case 9:  // ?????????? ?? ???????????????????
                if (g_TM.state == TM_STATE_COUNTDOWN || g_TM.state == TM_STATE_IDLE) {
                    g_TM.state = TM_STATE_RUNNING;
                    g_TM.cycle_timer = 0;
                    g_TM.disp_index  = DISP_SPEED;
                }
                break;
            case 10: // ???????? ?? ????????RUNNING??????????STOPPING????
                if (g_TM.state == TM_STATE_RUNNING) {
                    g_TM.state = TM_STATE_STOPPING;
                    g_TM.timer_100ms = TM_STOP_TIME_COUNT;
                }
                break;
        }
        UI_MarkDirty();
    }
}

/**
 * @brief ???? 100ms ???????
 */
void Treadmill_Manager_100ms(void)
{
    static float    s_accum_dist = 0.0f;
    static uint16_t s_stop_start_speed = 0; // ???????????????????????????????

		 /* ??????????? */
		
//		// ?????????? (E01)
//		if (!g_TM.safety_key && g_TM.state != TM_STATE_BOOT) {
//			 TM_Trigger_Error(1); // E-01
//		}
		
//    // ??????? (E-14) - ???????????????
//    if (g_TM.state != TM_STATE_BOOT) {
//        if (++s_watchdog_timer >= COMM_WATCHDOG_MAX) {
//            TM_Trigger_Error(14); // E-14
//        }
//    }	
		
    // ???????????????????????????????????????? status==7 ?????????
    // ??????? 3 ????????????????????????
    if (g_TM.state != TM_STATE_COUNTDOWN) {
        if (++s_heartbeat_tick >= 5) {
            s_heartbeat_tick = 0;
            AeroLink_Send(FC_HEARTBEAT, 0x00, (void*)0);
        }
    }

//    // ??????????????????????? UI ???????????????
//    if (g_TM.state == TM_STATE_ERROR) {
//        TM_UI_Dispatcher();
//        return;
//    }
//		
    /* ================================================================
     * ?????????????????????? s_lower_status ??????
     * RUNNING  + status!=9  ?? ?500ms???????+?????????????????
     * STOPPING             ?? ?500ms?????????????????? status==7
     * ================================================================ */
    if (g_TM.state == TM_STATE_RUNNING && s_lower_status == 7) {
        // ??????????????????????????????500ms???????+???
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

    // ?????????????????????????????????????????????????????????
		
		/* ?????????? */
		if (g_TM.steady_timer > 0 && --g_TM.steady_timer == 0) UI_MarkDirty();

    // ???????? 
    if (g_TM.state == TM_STATE_COUNTDOWN) {
        if (g_TM.timer_100ms > 0) {
			g_TM.timer_100ms--;
            // ????????? 30????? 20 ?? 10 ?????????
            if (g_TM.timer_100ms == 20 || g_TM.timer_100ms == 10) {
                Beep_Play(BEEP_CLICK);
            }
	        } else {
            // ?????????????????????????????????
            // ?????????????????status==9????s_cmd_retry_cnt ???????????
            s_accum_dist = 0.0f;
            if (g_TM.target_mode != TARGET_TIME) g_TM.time = 0;
            if (g_TM.target_mode != TARGET_DIST) g_TM.dist = 0.0f;
            if (g_TM.target_mode != TARGET_CAL)  g_TM.cal  = 0.0f;
            g_TM.speed = TM_SPEED_MIN;
            // ???? RUNNING???? s_cmd_retry_cnt ????????????? status==9 ???
            g_TM.state       = TM_STATE_RUNNING;
            g_TM.cycle_timer = 0;
            g_TM.disp_index  = DISP_SPEED;
            s_cmd_retry_cnt  = 0;
            AeroLink_Send(FC_CONTROL, 0x00, (void*)0);
            Send_Speed_To_Lower();
        }
    }

    // ???????? 
    if (g_TM.state == TM_STATE_RUNNING) {
        static uint8_t sec_cnt = 0;
        if (++sec_cnt >= 10) {
            sec_cnt = 0;
            bool trigger_stop = false;
            float dist_inc = (float)g_TM.speed / 36.0f; 
            s_accum_dist += dist_inc;
            if (g_TM.target_mode == TARGET_TIME) {
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
                s_cmd_retry_cnt  = 0;
                Beep_Play(BEEP_CLICK);
                AeroLink_Send(FC_CONTROL, 0x01, (void*)0); // s_cmd_retry_cnt ??????????
            }
        }
        if (g_TM.steady_timer == 0 && ++g_TM.cycle_timer >= 50) {
            g_TM.cycle_timer = 0;
            g_TM.disp_index = (TM_DispIndex_t)((g_TM.disp_index + 1) % 4);
        }
    }

    // ͣ���߼�����ʾƽ�����Լ��ٶ��� + �� s_fbk_speed==0 ��׼�ж�ֹͣʱ��
    if (g_TM.state == TM_STATE_STOPPING) {
        // ��¼��ʼ�ٶȣ����ڽ��� STOPPING �ĵ�һִ֡�У�
        if (g_TM.timer_100ms == TM_STOP_TIME_COUNT) {
            s_stop_start_speed = g_TM.speed;
        }

        // �����жϣ��¿ط����ٶ��ѹ��� �� ���������λ����׼ͬ����
        if (s_fbk_speed == 0) {
            g_TM.speed = 0;
            g_TM.state = TM_STATE_IDLE;
            Treadmill_Reset_Data();
        } else if (g_TM.timer_100ms > 0) {
            g_TM.timer_100ms--;
            // ��ʾ�����Լ��ٶ�����ÿ 100ms �̶���������֤ƽ���ݼ�������
            g_TM.speed = (uint32_t)s_stop_start_speed * g_TM.timer_100ms / TM_STOP_TIME_COUNT;
            UI_MarkDirty();
        } else {
            // ��ʱ���ף��¿س��� 10s ��δ���㣬ǿ�ƹ�λ
            g_TM.speed = 0;
            g_TM.state = TM_STATE_IDLE;
            Treadmill_Reset_Data();
        }
    }
				
    TM_UI_Dispatcher();
}



