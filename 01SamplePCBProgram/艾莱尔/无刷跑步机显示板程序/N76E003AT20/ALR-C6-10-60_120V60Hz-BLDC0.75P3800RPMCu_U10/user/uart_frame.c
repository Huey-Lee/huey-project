/*
 * uart_frame.c
 *
 *  Created on: 2019年12月11日
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

void UART1_cmd_proc(void);  //下控通讯接收指令处理
void UART0_cmd_proc(void);  //蓝牙模组接收指令处理
void UART0_SetCmd_proc(void);
void UART0_DataCmd_proc(void);

T_QUEUE UART1_rx_queue;
T_QUEUE UART0_rx_queue;

u8 UART1_rx_buf[UART1_RX_BUFF_SIZE];
u8 UART0_rx_buf[UART0_RX_BUFF_SIZE];

uart_frame_t uart_frame;
uart_frame_t uart_tx_frame;

ProtocolFrame_t frame;

/* ================================================================
 * AeroLink 协议实现
 * 帧格式：AA BB CID FC SFC DATA[8] CS_H CS_L EE FF (共17字节)
 * 校验：CS = CID + FC + SFC + DATA[0..7]
 * ================================================================ */

/* 下控反馈全局变量 */
u16 g_fbk_speed    = 0;
u8  g_lower_status = 0;

/* 内部控制变量 */
static u8  s_hb_tick      = 0;  /* 心跳计时（100ms 单位） */
static u8  s_cmd_retry    = 0;  /* 启动/停止命令重发计时 */

/* 底层字节发送（复用已有 UART1 硬件） */
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
    buf[5]  = (u8)(speed_val >> 8);   /* Data1 高位（速度高字节） */
    buf[6]  = (u8)(speed_val & 0xFF); /* Data1 低位（速度低字节） */
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
    aerolink_send_raw(AERO_FC_HEARTBEAT, 0x00, 0);
}

/* speed = treadmills.param.speed（单位0.1km/h×10，如100=1.0km/h）
 * 下控档位 = speed / 10（10~60对应1.0~6.0km/h） */
void aerolink_send_start(u16 speed)
{
    aerolink_send_raw(AERO_FC_CONTROL, 0x00, speed / 10);
}

void aerolink_send_stop(void)
{
    aerolink_send_raw(AERO_FC_CONTROL, 0x01, 0);
}

void aerolink_send_speed(u16 speed)
{
    aerolink_send_raw(AERO_FC_GEAR, 0x00, speed / 10);
}

/* 接收帧处理回调 */
static void aerolink_on_frame(u8 fc, u8 sfc, u8 *data)
{
    treadmills.ack = 1; /* 收到任意帧即确认通讯正常 */

    if (fc == 0x01 && sfc == 0x00) {
        /* 运行信息帧：data[0:1]=速度, data[2]=状态 */
        g_fbk_speed    = ((u16)data[0] << 8) | data[1];
        g_lower_status = data[2];
    }
    else if (fc == 0x01 && sfc == 0x01) {
        /* 故障信息帧：data[0:1]=一级故障, data[3]=二级故障 */
        u16 err_lv1 = ((u16)data[0] << 8) | data[1];
        u8  err_lv2 = data[3];
        if ((err_lv1 || err_lv2) && treadmills.error_code == 0) {
            u8 i;
            for (i = 0; i < 13; i++) {
                if (err_lv1 & ((u16)1 << i)) { treadmills.error_code = i + 1; break; }
            }
            if (!treadmills.error_code) {
                if (err_lv2 & 0x01) treadmills.error_code = 14;
                else if (err_lv2 & 0x02) treadmills.error_code = 15;
            }
            if (treadmills.error_code) treadmills.status = STATUS_ERROR;
        }
    }
}

/* 每 100ms 调用：心跳 + 启停命令可靠重发
 * 启动：STATUS_RUNNING 且下控未确认运行 → 每500ms重发
 * 停止：STATUS_STOP_WAIT 且速度未归零    → 每500ms重发 */
void aerolink_ctrl_loop(void)
{
    /* 心跳（倒计时期间暂停，避免下控回 status==7 干扰倒计时） */
    if (treadmills.status != STATUS_RUN) {
        if (++s_hb_tick >= 5) {
            s_hb_tick = 0;
            aerolink_send_heartbeat();
        }
    }

    /* 启动命令可靠重发 */
    if (treadmills.status == STATUS_RUNNING && g_lower_status == LOWER_STATUS_IDLE) {
        if (++s_cmd_retry >= 5) {
            s_cmd_retry = 0;
            aerolink_send_start(treadmills.param.speed);
        }
    }
    /* 停止命令可靠重发 */
    else if (treadmills.status == STATUS_STOP_WAIT) {
        if (++s_cmd_retry >= 5) {
            s_cmd_retry = 0;
            aerolink_send_stop();
        }
        /* 下控速度已归零 → 立即归位 */
        if (g_fbk_speed == 0) {
            treadmills.status = STATUS_STOP_OVER;
            s_cmd_retry = 0;
        }
    }
    else {
        s_cmd_retry = 0;
    }
}

/* ================================================================ */

void uart_frame_init(void)
{
	Create_Queue(&UART1_rx_queue,&UART1_rx_buf[0],UART1_RX_BUFF_SIZE);
	Create_Queue(&UART0_rx_queue,&UART0_rx_buf[0],UART0_RX_BUFF_SIZE);
}

//下控通讯发送
/* 旧协议发送函数：已由 AeroLink 替代，保留为空桩 */
void uart1_frame_tx(u8 cmd, u8 dat)             { cmd; dat; }
void uart1_frame_tx_2(u8 cmd, u8 dat0, u8 dat1) { cmd; dat0; dat1; }


/* ---------------------------------------------------------------
 * AeroLink 接收状态机 (替换旧 uart1_frame_loop)
 * 帧格式: AA BB CID FC SFC DATA[8] CS_H CS_L EE FF (17字节)
 * --------------------------------------------------------------- */
void uart1_frame_loop(void)
{
    static u8  sm = 0, d_idx = 0;
    static u8  rx_fc, rx_sfc;
    static u8  rx_data[8];
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
        case 3: rx_fc  = dat; cs_calc += dat; sm = 4; break;
        case 4: rx_sfc = dat; cs_calc += dat; d_idx = 0; sm = 5; break;
        case 5:
            rx_data[d_idx++] = dat; cs_calc += dat;
            if (d_idx >= 8) sm = 6;
            break;
        case 6: cs_match = (u16)dat << 8; sm = 7; break;
        case 7:
            cs_match |= dat;
            sm = (cs_match == cs_calc) ? 8 : 0;
            break;
        case 8: sm = (dat == AERO_TAIL1) ? 9 : 0; break;
        case 9:
            if (dat == AERO_TAIL2) aerolink_on_frame(rx_fc, rx_sfc, rx_data);
            sm = 0;
            break;
        default: sm = 0; break;
        }
    }
}



//---------------------------------------------------------------
extern u8 error_times;
extern u8 down_control;
//下控通讯接收指令处理
void UART1_cmd_proc(void)
{
	switch(uart_frame.cmd)
	{
	case CMD_STATUS:
		break;
	case CMD_SPEED:
		break;
//  case CMD_STATUS_HEART:
//    treadmills.param.runstep++;
//    break;
	case CMD_ACK:
    LED1=!LED1;//指示灯闪烁
    treadmills.ack=1;
    down_control = uart_frame.buf[0];
    debug_printf("\r\n --->rply ack");
    //LED1=1;//熄灭
    break;
	case CMD_RESET:
	#if(0)
    debug_printf("\r\n software reset");
    AUXR1 &=(0X7F);
    EA=0;
    TA=0XAA;
    TA=0X55;
    CHPCON |=(1<<7);
    //EA=1;
	#endif
	  break;
	case CMD_STOP_OVER:
		if(error_times==0)treadmills.status=STATUS_STOP_OVER;
		break;
	case CMD_ERROR:
		treadmills.ack=1;
		if(treadmills.error_code ==0)
		{
			treadmills.error_code=uart_frame.buf[0];
      error_times=0;
		}
		treadmills.status=STATUS_ERROR;
		break;
	case CMD_WRITE_PARAM:
		uart_tx_frame.cmd=CMD_WRITE_PARAM;
		uart_tx_frame.len=2;
		uart_tx_frame.buf[0]=(u8)param.speed_adc;
		uart_tx_frame.buf[1]=(~(u8)param.speed_adc);
		UART_Send_Buf(UART1,(UINT8*)&uart_tx_frame,sizeof(uart_tx_frame));
		break;
	case CMD_READ_PARAM:
		//验证校验
		if((uart_frame.buf[0] & uart_frame.buf[1])==0)
		{
			param.speed_adc=(s8)uart_frame.buf[0];
		}
		else
		{
			param.speed_adc=0;
		}
		break;
	case CMD_TEST:
		beep_set(1,DEFAULT_ON_TICKS,DEFAULT_OFF_TICKS);
		if(uart_frame.buf[0] ==0)
		{
			indecate.led_step=LED_ON;
			indecate.led_speed=LED_ON;
			indecate.led_updp=LED_ON;
			indecate.led_downdp=LED_ON;

			indecate.led_time=LED_ON;
			indecate.led_distance=LED_ON;
			indecate.led_calorie=LED_ON;
			set_seg_val(1234);
			set_seg_mode(SEG_MODE_NORMAL);
		}
		else if(uart_frame.buf[0] ==1)
		{
			indecate.led_step=LED_ON;
			indecate.led_speed=LED_ON;
			indecate.led_updp=LED_ON;
			indecate.led_downdp=LED_ON;

			indecate.led_time=LED_ON;
			indecate.led_distance=LED_ON;
			indecate.led_calorie=LED_ON;
			set_seg_val(5678);
			set_seg_mode(SEG_MODE_NORMAL);
		}
		else if(uart_frame.buf[0] ==2)
		{
			indecate.led_step=LED_ON;
			indecate.led_speed=LED_ON;
			indecate.led_updp=LED_ON;
			indecate.led_downdp=LED_ON;

			indecate.led_time=LED_ON;
			indecate.led_distance=LED_ON;
			indecate.led_calorie=LED_ON;
			set_seg_val(8888);
			set_seg_mode(SEG_MODE_NORMAL);
		}
		break;
	default:
		break;
	}
}

/* 异或校验函数 */
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

    // 如果有数据
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


// === 全局变量与定义 ===
volatile u16 bt_init_tick = 0;
volatile u8 bt_param_send_step = 0;
bit bt_param_send_flag = 0;
bit app_control_permission = 0;

/* 蓝牙接收处理 */
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
    u8 result = 0x01; // 默认操作成功
  
    // ========== APP控制类指令 ==========
    if (frame.cmd == 0x03)
    {
        switch (frame.subcmd)
        {
            case 0x00: // APP请求控制
                if(frame.len == 0x00)
                {
                  uart0_send_frame(0x57, 0x03, 0x00, 0x01, &result, 0x58);
                }
            break;

            case 0x01: // 复位
                if(frame.len == 0x00)
                {
                    treadmills.status = STATUS_STOP;
                    treadmills.param.distance = 0;
                    treadmills.param.calorie = 0;
                    treadmills.param.second = 0;                   
                    uart0_send_frame(0x57, 0x03, 0x01, 0x01, &result, 0x58);                                     
                }
                break;

            case 0x02: // 设置目标速度
              if(frame.len == 0x02)
              {
                if (treadmills.status == STATUS_RUNNING)
                {
										treadmills.param.speed = frame.buf[0] | (frame.buf[1] << 8);
                    aerolink_send_speed(treadmills.param.speed); /* BT APP设置速度→AeroLink */
                    treadmills.display.index = 0;
                    treadmills.display.mode = DISP_MODE_SINGLE;
                    beep_set(1, DEFAULT_ON_TICKS, DEFAULT_OFF_TICKS);
                }
								else
								{
									result = 0x05; // 不允许操作
								}
								uart0_send_frame(0x57, 0x03, 0x02, 0x01, &result, 0x58);
              }
              break;

            case 0x03: // 设置目标坡度（暂无）
                result = 0x02; // 不支持
                uart0_send_frame(0x57, 0x03, 0x03, 0x01, &result, 0x58);
                break;

            case 0x07: // 启动/恢复						
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
												result = 0x05; // 不允许操作
										}
										uart0_send_frame(0x57, 0x03, 0x07, 0x01, &result, 0x58);
								}
								break;

            case 0x08: // 停止0x01/暂停0x02						
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
                 result = 0x05; // 不允许操作
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
            case 0x01: // APP连接						
							break;
						case 0x02:	 // 断开设备
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
u8 speed_range[6] = {0x64, 0x00, 0x58, 0x02, 0x0A, 0x00};		// 小端编码 1.0 6.0 0.1
u8 device_type[4] = {0x01, 0x00, 0x02, 0x01};

/* 蓝牙延迟参数设置逻辑 */
void bt_power_send(void)
{
    static u8 delay_cnt = 0;

    if (bt_param_send_flag)
    {
        delay_cnt++;
        if (delay_cnt < 1) return; // 保证100ms节拍进入
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


// 面板操作上报（发送 0x02 类帧）
void bt_panel_send_cmd(u8 subcmd, u8 len, u8* buf)
{
    uart0_send_frame(0x57, 0x02, subcmd, len, buf, 0x58);
}

void bt_panel_send_speed(void)
{
    u8 speed_buf[2];

    speed_buf[0] = treadmills.param.speed & 0xFF;         // 低字节
    speed_buf[1] = (treadmills.param.speed >> 8) & 0xFF;  // 高字节

    bt_panel_send_cmd(0x05, 0x02, speed_buf); // 0x05: 设置速度，长度为2
}


// ========== 上报运动数据协议帧 ==========
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
    u16 flags     = 0x0484;//无steps

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
            return; // 不上报
    }

    send_motion_data(status_code);
}



