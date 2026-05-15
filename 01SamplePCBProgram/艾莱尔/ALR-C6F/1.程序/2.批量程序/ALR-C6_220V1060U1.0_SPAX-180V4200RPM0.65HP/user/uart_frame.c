/*
 * uart_frame.c
 *
 *  Created on: 2019��12��11��
 *      Author: Administrator
 */
#include "common.h"
#include "uart_frame.h"
#include "queue.h"
#include "treadmills.h"
#include "param.h"
#include "led_disp.h"
#include "indecate.h"
#include "beep.h"
#include "uart.h"
#include "cfg.h"
#include "keypad.h"
#include "keypad_fun.h"


void UART0_cmd_proc(void);  //����ģ�����ָ���
void UART0_SetCmd_proc(void);
void UART0_DataCmd_proc(void);

T_QUEUE UART1_rx_queue;
T_QUEUE UART0_rx_queue;

u8 UART1_rx_buf[UART1_RX_BUFF_SIZE];
u8 UART0_rx_buf[UART0_RX_BUFF_SIZE];


ProtocolFrame_t frame;

/* ================================================================
 * AeroLink Э��ʵ��
 * ֡��ʽ��AA BB CID FC SFC DATA[8] CS_H CS_L EE FF (��17�ֽ�)
 * У�飺CS = CID + FC + SFC + DATA[0..7]
 * ================================================================ */

/* �¿ط���ȫ�ֱ��� */
u16 g_fbk_speed    = 0;
u8  g_lower_status = 0;

/* �ڲ����Ʊ��� */
static u8  s_hb_tick      = 0;  /* ������ʱ��100ms ��λ�� */
static u8  s_cmd_retry    = 0;  /* ����/ֹͣ�����ط���ʱ */

/* �ײ��ֽڷ��ͣ��������� UART1 Ӳ���� */
static void aerolink_send_raw(u8 fc, u8 sfc, u16 speed_val)
{
    u8  buf[17];
    u16 cs;
    u8  i;

    buf[0]  = AERO_HEAD1;
    buf[1]  = AERO_HEAD2;
    buf[2]  = AERO_CID_SEND;
    buf[3]  = fc;
    buf[4]  = sfc;
    buf[5]  = (u8)(speed_val >> 8);   /* Data1 ��λ���ٶȸ��ֽڣ� */
    buf[6]  = (u8)(speed_val & 0xFF); /* Data1 ��λ���ٶȵ��ֽڣ� */
    buf[7]  = 0; buf[8]  = 0;
    buf[9]  = 0; buf[10] = 0;
    buf[11] = 0; buf[12] = 0;

    cs = (u16)AERO_CID_SEND + fc + sfc;
    for (i = 5; i <= 12; i++) cs += buf[i];

    buf[13] = (u8)(cs >> 8);
    buf[14] = (u8)(cs & 0xFF);
    buf[15] = AERO_TAIL1;
    buf[16] = AERO_TAIL2;

    UART_Send_Buf(UART1, buf, 17);
}

void aerolink_send_heartbeat(void)
{
    aerolink_send_raw(AERO_FC_HEARTBEAT, AERO_SFC_HB, 0);
}

/* start: FC=0x02 SFC=0x00 x2 then speed (follow ref program) */
void aerolink_send_start(u16 speed)
{
    aerolink_send_raw(AERO_FC_STARTSTOP, AERO_SFC_START, 0);
    aerolink_send_raw(AERO_FC_STARTSTOP, AERO_SFC_START, 0);
    aerolink_send_raw(AERO_FC_GEAR, 0x00, speed / 10);
}

/* ֹͣ: FC=0x01 SFC=0x02 */
void aerolink_send_stop(void)
{
    aerolink_send_raw(AERO_FC_STARTSTOP, AERO_SFC_STOP, 0);
}

/* �ٶȵ�λ: FC=0x03 SFC=0x00, data1=�ٶ�ֵ/10 */
void aerolink_send_speed(u16 speed)
{
    aerolink_send_raw(AERO_FC_GEAR, 0x00, speed / 10);
}
static u8  s_rx_fc    = 0;
static u8  s_rx_sfunc = 0;
static u8  s_rx_buf[8];

/* ����֡�������޲�����ʹ�þ�̬���壩 */
static void aerolink_on_frame(void)
{
    treadmills.ack = 1;
    if (s_rx_fc == 0x01 && s_rx_sfunc == 0x00) {
        g_fbk_speed    = ((u16)s_rx_buf[0] << 8) | s_rx_buf[1];
        g_lower_status = s_rx_buf[3];   /* d2_l */
    }
    else if (s_rx_fc == 0x01 && s_rx_sfunc == 0x01) {
        u16 err_lv1;
        u8  err_lv2;
        u8  ei;
        err_lv1 = ((u16)s_rx_buf[0] << 8) | s_rx_buf[1];
        err_lv2 = s_rx_buf[3];
        if ((err_lv1 || err_lv2) && treadmills.error_code == 0) {
            for (ei = 0; ei < 13; ei++) {
                if (err_lv1 & ((u16)1 << ei)) { treadmills.error_code = ei + 1; break; }
            }
            if (!treadmills.error_code) {
                if (err_lv2 & 0x01) treadmills.error_code = 14;
                else if (err_lv2 & 0x02) treadmills.error_code = 15;
            }
            if (treadmills.error_code) treadmills.status = STATUS_ERROR;
        }
    }
}

/* ÿ 100ms ���ã����� + ��ͣ����ɿ��ط�
 * ������STATUS_RUNNING ���¿�δȷ������ �� ÿ500ms�ط�
 * ֹͣ��STATUS_STOP_WAIT ���ٶ�δ����    �� ÿ500ms�ط� */
void aerolink_ctrl_loop(void)
{
    /* ����������ʱ�ڼ���ͣ�������¿ػ� status==7 ���ŵ���ʱ�� */
    if (treadmills.status != STATUS_RUN) {
        if (++s_hb_tick >= 5) {
            s_hb_tick = 0;
            aerolink_send_heartbeat();
        }
    }

    /* ��������ɿ��ط� */
    if (treadmills.status == STATUS_RUNNING && g_lower_status != 9) {
        if (++s_cmd_retry >= 5) {
            s_cmd_retry = 0;
            aerolink_send_start(treadmills.param.speed);
        }
    }
    /* ֹͣ����ɿ��ط� */
    else if (treadmills.status == STATUS_STOP_WAIT) {
        if (++s_cmd_retry >= 5) {
            s_cmd_retry = 0;
            aerolink_send_stop();
        }
        /* �¿��ٶ��ѹ��� �� ������λ */
        if (g_lower_status != 9) {
            treadmills.status = STATUS_STOP_OVER;
            s_cmd_retry = 0;
        }
    }
    else {
        s_cmd_retry = 0;
    }
    /* speed sync: running and lower running -> send if speed mismatch */
    if (treadmills.status == STATUS_RUNNING && g_lower_status == 9) {
        if ((treadmills.param.speed / 10) != g_fbk_speed) {
            aerolink_send_speed(treadmills.param.speed);
        }
    }
}

/* ================================================================ */

void uart_frame_init(void)
{
	Create_Queue(&UART1_rx_queue,&UART1_rx_buf[0],UART1_RX_BUFF_SIZE);
	Create_Queue(&UART0_rx_queue,&UART0_rx_buf[0],UART0_RX_BUFF_SIZE);
}



/* ---------------------------------------------------------------
 * AeroLink ����״̬�� (�滻�� uart1_frame_loop)
 * ֡��ʽ: AA BB CID FC SFC DATA[8] CS_H CS_L EE FF (17�ֽ�)
 * --------------------------------------------------------------- */
void uart1_frame_loop(void)
{
    static u8  sm = 0, d_idx = 0;
    /* rx buffers now use file-scope statics s_rx_fc/s_rx_sfunc/s_rx_buf */
    /* (s_rx_buf is declared above) */
    static u16 cs_calc, cs_match;
    u8 dat;

    while (Denter_queue(&UART1_rx_queue, &dat)) {
        switch (sm) {
        case 0: if (dat == AERO_HEAD1) sm = 1; break;
        case 1: sm = (dat == AERO_HEAD2) ? 2 : 0; break;
        case 2:
            if (dat == AERO_CID_RECV) { cs_calc = dat; sm = 3; }
            else sm = 0;
            break;
        case 3: s_rx_fc    = dat; cs_calc += dat; sm = 4; break;
        case 4: s_rx_sfunc = dat; cs_calc += dat; d_idx = 0; sm = 5; break;
        case 5:
            s_rx_buf[d_idx++] = dat; cs_calc += dat;
            if (d_idx >= 8) sm = 6;
            break;
        case 6: cs_match = (u16)dat << 8; sm = 7; break;
        case 7:
            cs_match |= dat;
            sm = (cs_match == cs_calc) ? 8 : 0;
            break;
        case 8: sm = (dat == AERO_TAIL1) ? 9 : 0; break;
        case 9:
            if (dat == AERO_TAIL2) aerolink_on_frame();
            sm = 0;
            break;
        default: sm = 0; break;
        }
    }
}


/* ���У�麯�� */
unsigned char crc_cheack(unsigned char *pkt, unsigned char len)  
{  
    unsigned char i = 0, calculatedCRC = pkt[0];  
    for(i = 1; i < len; i++)   
    {  
        calculatedCRC ^= pkt[i];  
    }  
    return calculatedCRC;  
}


void uart0_send_frame(u8 sof, u8 cmd, u8 subcmd, u8 len, u8 *buf, u8 end)
{
    u8 tx_buf[6 + MAX_DATA_LEN]; // max buffer
    u8 temp[3 + MAX_DATA_LEN];   // for CRC
    u8 i;

    tx_buf[0] = sof;
    tx_buf[1] = cmd;
    tx_buf[2] = subcmd;
    tx_buf[3] = len;

    temp[0] = cmd;
    temp[1] = subcmd;
    temp[2] = len;

    // ���������
    if (len > 0 && buf != NULL)
    {
        for (i = 0; i < len; i++)
        {
            tx_buf[4 + i] = buf[i];
            temp[3 + i] = buf[i];
        }
    }

    tx_buf[4 + len] = crc_cheack(temp, 3 + len); // fcs
    tx_buf[5 + len] = end;                       // end

    UART_Send_Buf(UART0, tx_buf, 6 + len);       // total length
}


// === ȫ�ֱ����붨�� ===
volatile u16 bt_init_tick = 0;
volatile u8 bt_param_send_step = 0;
bit bt_param_send_flag = 0;
bit app_control_permission = 0;

/* �������մ��� */
void uart0_frame_loop(void)
{
    static u8 sm = 0, len = 0;
    static u8 *ptr;
    u8 datt;

    while (Denter_queue(&UART0_rx_queue, &datt)) {
        switch (sm)
        {
            case 0:
                if (datt == 0x53 || datt == 0x57) {
                    frame.sof = datt;
                    sm = 1;
                }
                break;
            case 1:
                frame.cmd = datt;
                sm = 2;
                break;
            case 2:
                frame.subcmd = datt;
                sm = 3;
                break;
            case 3:
                frame.len = datt;
                if (frame.len > MAX_DATA_LEN) {
                    sm = 0;
                    break;
                }
                len = 0;
                ptr = frame.buf;
                sm = 4;
                break;
            case 4:
                ptr[len++] = datt;
                if (len >= frame.len + 1)
                    sm = 5;
                break;
            case 5:
                if ((datt == 0x54 && frame.sof == 0x53) || (datt == 0x58 && frame.sof == 0x57)) {
                    frame.end = datt;
                    if (frame.sof == 0x53 && frame.cmd == 0x03 && frame.subcmd == 0x00 && frame.len == 0) 
										{
                        uart0_send_frame(0x53, 0x03, 0x00, 0, NULL, 0x54);
                        bt_init_tick = 0;
                        bt_param_send_flag = 1;
                        bt_param_send_step = 0;
                    }
                    else if(frame.sof == 0x57) 
										{
                      UART0_cmd_proc();
                    }
                }
                sm = 0;
                break;
            default:
                sm = 0;
                break;
        }
    }
}

void UART0_cmd_proc(void)
{
    u8 result = 0x01; // Ĭ�ϲ����ɹ�
  
    // ========== APP������ָ�� ==========
    if (frame.cmd == 0x03)
    {
        switch (frame.subcmd)
        {
            case 0x00: // APP�������
                if(frame.len == 0x00)
                {
                  uart0_send_frame(0x57, 0x03, 0x00, 0x01, &result, 0x58);
                }
            break;

            case 0x01: // ��λ
                if(frame.len == 0x00)
                {
                    treadmills.status = STATUS_STOP;
                    treadmills.param.distance = 0;
                    treadmills.param.calorie = 0;
                    treadmills.param.second = 0;                   
                    uart0_send_frame(0x57, 0x03, 0x01, 0x01, &result, 0x58);                                     
                }
                break;

            case 0x02: // ����Ŀ���ٶ�
              if(frame.len == 0x02)
              {
                if (treadmills.status == STATUS_RUNNING)
                {
										treadmills.param.speed = frame.buf[0] | (frame.buf[1] << 8);
                    aerolink_send_speed(treadmills.param.speed); /* BT APP�����ٶȡ�AeroLink */
                    treadmills.display.index = 0;
                    treadmills.display.mode = DISP_MODE_SINGLE;
                    beep_set(1, DEFAULT_ON_TICKS, DEFAULT_OFF_TICKS);
                }
								else
								{
									result = 0x05; // ����������
								}
								uart0_send_frame(0x57, 0x03, 0x02, 0x01, &result, 0x58);
              }
              break;

            case 0x03: // ����Ŀ���¶ȣ����ޣ�
                result = 0x02; // ��֧��
                uart0_send_frame(0x57, 0x03, 0x03, 0x01, &result, 0x58);
                break;

            case 0x07: // ����/�ָ�						
                if (treadmills.status == STATUS_SLEEP_MODE)
                {
                    treadmills.status = STATUS_POWERON;
                }
								else
								{										
										if ((treadmills.status == STATUS_WAIT ||treadmills.status == STATUS_STOP_OVER ||treadmills.status == STATUS_PAUSE||
												 treadmills.status == STATUS_STOP) && (treadmills.error_code == 0))
										{
												treadmills.status = STATUS_RUN;
												key_mask = 0xFF;
												beep_set(1, DEFAULT_ON_TICKS, DEFAULT_OFF_TICKS);
										}
										else
										{
												result = 0x05; // ����������
										}
										uart0_send_frame(0x57, 0x03, 0x07, 0x01, &result, 0x58);
								}
								break;

            case 0x08: // ֹͣ0x01/��ͣ0x02						
              if(frame.len == 0x01 && frame.buf[0] == 0x01)
              {
										PAUSE_FLAG = 0;
                    treadmills.status = STATUS_STOP;//_OVER;
                    beep_set(1, DEFAULT_ON_TICKS, DEFAULT_OFF_TICKS);
              }
              else if (frame.len == 0x01 && frame.buf[0] == 0x02)
              {
									 PAUSE_FLAG = 1;
                   treadmills.status = STATUS_STOP;
                   beep_set(1, DEFAULT_ON_TICKS, DEFAULT_OFF_TICKS);
              }
              else
              {
                 result = 0x05; // ����������
              }
              
              uart0_send_frame(0x57, 0x03, 0x08, 0x01, &result, 0x58);
              break;

            default:
                break;
        }
    }
		
		else if(frame.cmd == 0x04)
		{
			switch (frame.subcmd)
        {
            case 0x01: // APP����						
							break;
						case 0x02:	 // �Ͽ��豸
							treadmills.status = STATUS_STOP;//_OVER;
              beep_set(1, DEFAULT_ON_TICKS, DEFAULT_OFF_TICKS);
							break;
						default:
							break;							
				}
			
		}			
}   


u8 name[] = {0x53, 0x50, 0x41, 0x58}; // SPAX
u8 feature_setting_data[8] = {0x04, 0x12, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
u8 treadmill_status[] = {0x01};
u8 speed_range[6] = {0x64, 0x00, 0x58, 0x02, 0x0A, 0x00};		// С�˱��� 1.0 6.0 0.1
u8 device_type[4] = {0x01, 0x00, 0x02, 0x01};

/* �����ӳٲ��������߼� */
void bt_power_send(void)
{
    static u8 delay_cnt = 0;

    if (bt_param_send_flag)
    {
        delay_cnt++;
        if (delay_cnt < 1) return; // ��֤100ms���Ľ���
        delay_cnt = 0;

        switch (bt_param_send_step++)
        {
            case 0:
                uart0_send_frame(0x53, 0x03, 0x01, sizeof(name), name, 0x54);
                break;
            case 1:
                uart0_send_frame(0x53, 0x04, 0x01, sizeof(feature_setting_data), feature_setting_data, 0x54);
                break;
            case 2:
                uart0_send_frame(0x53, 0x04, 0x02, sizeof(treadmill_status), treadmill_status, 0x54);
                break;
            case 3:
                uart0_send_frame(0x53, 0x04, 0x03, sizeof(speed_range), speed_range, 0x54);
                break;
            case 4:
                uart0_send_frame(0x53, 0x04, 0x0A, sizeof(device_type), device_type, 0x54);
                bt_param_send_flag = 0;
                bt_param_send_step = 0;
                break;
            default:
                break;
        }
    }
}


// �������ϱ������� 0x02 ��֡��
void bt_panel_send_cmd(u8 subcmd, u8 len, u8* buf)
{
    uart0_send_frame(0x57, 0x02, subcmd, len, buf, 0x58);
}

void bt_panel_send_speed(void)
{
    u8 speed_buf[2];

    speed_buf[0] = treadmills.param.speed & 0xFF;         // ���ֽ�
    speed_buf[1] = (treadmills.param.speed >> 8) & 0xFF;  // ���ֽ�

    bt_panel_send_cmd(0x05, 0x02, speed_buf); // 0x05: �����ٶȣ�����Ϊ2
}


// ========== �ϱ��˶�����Э��֡ ==========
void send_motion_data(u8 status_code)
{
    u8 buf[18];
    u16 speed     = treadmills.param.speed;
    u32 distance  = treadmills.param.distance;
    u16 calorie   = treadmills.param.calorie / 1000;
    u16 hour_cal  = 0;
    u8  min_cal   = 0;
    u16 seconds   = treadmills.param.second;
    u16 steps     = 0;
    u16 flags     = 0x0484;//��steps

    buf[0]  = status_code;
    buf[1]  = flags & 0xFF;
    buf[2]  = (flags >> 8) & 0xFF;
    buf[3]  = speed & 0xFF;
    buf[4]  = (speed >> 8) & 0xFF;
    buf[5]  = distance & 0xFF;
    buf[6]  = (distance >> 8) & 0xFF;
    buf[7]  = (distance >> 16) & 0xFF;
    buf[8]  = calorie & 0xFF;
    buf[9]  = (calorie >> 8) & 0xFF;
    buf[10] = hour_cal & 0xFF;
    buf[11] = (hour_cal >> 8) & 0xFF;
    buf[12] = min_cal;
    buf[13] = seconds & 0xFF;
    buf[14] = (seconds >> 8) & 0xFF;
    buf[15] = steps & 0xFF;
    buf[16] = (steps >> 8) & 0xFF;
    buf[17] = 0x00;

    uart0_send_frame(0x57, 0x01, 0x00, sizeof(buf), buf, 0x58);
}

void send_motion_data_by_status(void)
{
    u8 status_code;

    switch (treadmills.status)
    {
      
        case STATUS_POWERON:
        case STATUS_WAIT:
        case STATUS_STOP_OVER:
				case STATUS_PAUSE:
            status_code = 0x01;
            break;
        case STATUS_RUN:
            status_code = 0x0E;//0x0E
            break;
        case STATUS_RUNNING:
            status_code = 0x0D;
            break;
        case STATUS_STOP:
            status_code = 0x0F;
            break;
        case STATUS_ERROR:
//            status_code = 0x10;
            break;
        case STATUS_REED_ERROR:
//            status_code = 0x11;
            break;
        default:
            return; // ���ϱ�
    }

    send_motion_data(status_code);
}



