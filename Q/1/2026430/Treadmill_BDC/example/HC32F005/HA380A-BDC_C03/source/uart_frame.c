/* uart_frame.c - 7E/7F frame pack/parse, cmds, display link, params. */
#include "uart_frame.h"
#include "queue.h"
#include "motor.h"
#include "uart.h"
#include "user_usart.h"	
#include "user_timer.h"
#include "pid.h"
extern volatile u16 over_current;
extern volatile u16 over_voltage_adc;
extern u16 over_current_low_spd;
ee_param_t  rx_setparam;


T_QUEUE rx_queue;
u8 rx_buf[RX_BUFF_SIZE];  //135
uart_frame_t uart_frame;
uart_frame_t uart_tx_frame;
extern u8 relay_wait_en;

void uart_frame_init(void)
{
	Create_Queue(&rx_queue,&rx_buf[0],RX_BUFF_SIZE);

	uart_frame.sof=START_OF_FRAME;    /* 0x7F */
	uart_frame.eof=END_OF_FRAME;      /* 0x7E */

	uart_tx_frame.sof=START_OF_FRAME;
	uart_tx_frame.eof=END_OF_FRAME;
}
void UART_Send_Buf(uint8_t UARTPort, uint8_t *ptr,uint8_t len)
{
	for(uint8_t l=0;l<len;l++)
	{
		Uart_SendDataPoll(M0P_UART1,ptr[l]);
	}
}
void uart_frame_tx(u8 cmd,u8 dat)
{
	u8 xor=0;     /* XOR of cmd,len,payload */
	uart_tx_frame.cmd=cmd;
	xor^=cmd;
	uart_tx_frame.len=1;
	xor^=1;
	uart_tx_frame.buf[0]=dat;
	xor^=dat;
	uart_tx_frame.buf[1]=xor;
	uart_tx_frame.crch=0;
	uart_tx_frame.crcl=END_OF_FRAME;
	UART_Send_Buf(UART0,(u8*)&uart_tx_frame,uart_tx_frame.len+6);
}

void uart_frame_tx_2(u8 cmd,u8 dat0,u8 dat1)
{
	u8 xor=0;
	uart_tx_frame.cmd=cmd;
	xor^=cmd;
	uart_tx_frame.len=2;
	xor^=2;
	uart_tx_frame.buf[0]=dat0;
	xor^=dat0;
	uart_tx_frame.buf[1]=dat1;
	xor^=dat1;
	uart_tx_frame.crch=xor;
	uart_tx_frame.crcl=0;
	uart_tx_frame.eof=END_OF_FRAME;
	UART_Send_Buf(UART0,(u8*)&uart_tx_frame,uart_tx_frame.len+6);
}

/* display poll ~500ms */
void communication_checkout(void)
{
	if(ctr.ack == 1)
	{
		if(ctr.error_code)
		{
			uart_frame_tx(CMD_ERROR,ctr.error_code);
		}
		else
		{
			uart_frame_tx(CMD_ACK,2);
		}
		ctr.ack=0;
	}
}
extern u8 uart_time_error;
void uart_frame_loop(void)  /* RX state machine, bytes from Denter_queue */
{
   static u8  sm=0,len=0;
   static u8  *ptr;
   u8 dat;
   u8 ret;

   ret=Denter_queue(&rx_queue,&dat);
   if(ret)    /* one byte dequeued */
   {
	   if(dat==uart_frame.sof){sm=1;uart_time_error=0;return;}
	   switch(sm)
	   {
	   case 1:
		   uart_frame.cmd=dat;
		   sm=2;
		   break;
	   case 2:
		   uart_frame.len=dat;
		   if(uart_frame.len>2){sm=0;return;}
		   len=0;
		   ptr=&uart_frame.buf[0];
		   sm=3;
		   break;
	   case 3:
		   ptr[len]=dat; /* payload + 2B checksum in buf */
		   len++;
		   if(len>=(uart_frame.len+2)){sm=4;}
		   break;
	   case 4:
		   if(dat==uart_frame.eof)												/* EOF */
		   {
			   uint8_t i;
			   uint8_t xsum, m7, expect, s2;
			   ptr=&uart_frame.cmd;
			   dat=0;
			   for(len=0;len<(uart_frame.len+2);len++)
			   {
				  dat^=ptr[len];
			   }
			   xsum=dat; /* == ptr[0]^ptr[1]^...^data */
			   s2=ptr[0];
			   s2=(uint8_t)(s2+ptr[1]);
			   for (i=0; i<uart_frame.len; i++) {
					s2=(uint8_t)(s2+ptr[2+i]);
			   }
			   m7=(uint8_t)(s2%7u);
			   expect=ptr[uart_frame.len+2u];
			   /* accept XOR (legacy) or (CMD+LEN+DATA)%7 (8+1+0)%7=0x02 */
			   if (xsum==expect || m7==expect) { cmd_proc(); }
		   }
			 sm=0;
		   break;
	   default:
		   break;
	   }
   }
}

extern u8 speed_step;
extern pid_t mt_pid;

static u8 ClampKiv(u8 raw)
{
    return (raw > 10u) ? 10u : raw;
}

void cmd_proc(void)
{
	u8 scale=0;
    u8 kiv_val = 0;
	switch(uart_frame.cmd)
	{
	case CMD_STATUS:
		
		if(motor.en !=uart_frame.buf[0])
		{

			if(uart_frame.buf[0]==0)
			{
				ctr.ack = 1;
				motor.en = 0;
				ctr.error_code = 0u;  /* 上控明确停机，清除故障码，允许下次重启 */
				motor.set_speed_scale = MT_STOP_SPEED;/* min stop speed */
//				motor.kiv=speed_param[motor.index].kiv;
				ctr.status = STATUS_STOP;							/* stop request */
			}
			else if(uart_frame.buf[0]==1)
			{
				/* 故障未清除时拒绝运行指令 */
				if (ctr.error_code != 0u) { break; }
				/* 受控减停进行中（E06/E01 soft-stop），禁止重启：
				 * motor_controlled_stop_set 不修改 motor.en，上控先发
				 * CMD_STATUS=0（清错）再发 CMD_STATUS=1（重启）会在减
				 * 停尚未完成时令电机以最低速重启，用户看到"6V不停"。
				 * motor.status==NULL（完全停止）后才允许重启。         */
				if (motor.status == STATUS_MT_STOP) { break; }
				 ctr.ack = 1;
			   motor.en = 1;
			   ctr.status = STATUS_RUN;

			   if(uart_frame.buf[1] > motor.adjust_speed_max ){uart_frame.buf[1] = motor.adjust_speed_max;}
				 motor.adjust_sta_voltage = motor.start_valtage;
			   motor.set_speed_scale=uart_frame.buf[1]*10;
			   motor.cur_speed_scale=motor.sta_speed_scale;
			}
		}
		break;
	case CMD_PID_P:
		mt_pid.kp = (float)uart_frame.buf[0] / 100.0f;
		uart_frame_tx(CMD_PID_P, (u8)(mt_pid.kp * 100.0f));
	  break;
	case CMD_PID_I:
		mt_pid.ki = (float)uart_frame.buf[0] / 100.0f;
		uart_frame_tx(CMD_PID_I, (u8)(mt_pid.ki * 100.0f));
	  break;
	case CMD_PID_D:
		mt_pid.kd = (float)uart_frame.buf[0] / 100.0f;
		uart_frame_tx(CMD_PID_D, (u8)(mt_pid.kd * 100.0f));
	  break;
	case CMD_STA_LEVEL:			/* 1~15，0为兜底 */
	if(uart_frame.buf[0] > 15)
		uart_frame.buf[0] = 15;
	/* 起始电压5档（40/60/80/100/120 ADC ≈ 5/7.6/10/12.6/15.1V），
	 * 每档内斜率3级（2/4/6），共15档均匀覆盖。
	 *
	 * start_valtage：初始建压值（<前馈钳位≈135ADC，均有效）
	 * start_add_val：每91ms爬升步长，小=柔和慢，大=有力快
	 *
	 * 1档最柔（轻载低速），15档最有力（重载高速） */
	switch(uart_frame.buf[0])
	{
		case(0):   /* 兜底：与1档相同 */
		case(1):  motor.start_valtage =  40; motor.start_add_val =  2; break; /* ≈5.0V  慢 */
		case(2):  motor.start_valtage =  40; motor.start_add_val =  4; break; /* ≈5.0V  中 */
		case(3):  motor.start_valtage =  40; motor.start_add_val =  6; break; /* ≈5.0V  快 */
		case(4):  motor.start_valtage =  60; motor.start_add_val =  2; break; /* ≈7.6V  慢 */
		case(5):  motor.start_valtage =  60; motor.start_add_val =  4; break; /* ≈7.6V  中 */
		case(6):  motor.start_valtage =  60; motor.start_add_val =  6; break; /* ≈7.6V  快 */
		case(7):  motor.start_valtage =  80; motor.start_add_val =  2; break; /* ≈10.1V 慢 */
		case(8):  motor.start_valtage =  80; motor.start_add_val =  4; break; /* ≈10.1V 中 */
		case(9):  motor.start_valtage =  80; motor.start_add_val =  6; break; /* ≈10.1V 快 */
		case(10): motor.start_valtage = 100; motor.start_add_val =  2; break; /* ≈12.6V 慢 */
		case(11): motor.start_valtage = 100; motor.start_add_val =  4; break; /* ≈12.6V 中 */
		case(12): motor.start_valtage = 100; motor.start_add_val =  6; break; /* ≈12.6V 快 */
		case(13): motor.start_valtage = 120; motor.start_add_val =  2; break; /* ≈15.1V 慢 */
		case(14): motor.start_valtage = 120; motor.start_add_val =  4; break; /* ≈15.1V 中 */
		case(15): motor.start_valtage = 120; motor.start_add_val =  6; break; /* ≈15.1V 快 */
		default:  motor.start_valtage =  80; motor.start_add_val =  4; break; /* 安全回退 */
	}
		break;
	case CMD_STATUS_INQUIRE:
		break;
	case CMD_END_LEVEL:			/* 2~20 */
			if(uart_frame.buf[0] > 20)
				uart_frame.buf[0] = 20;
			if(uart_frame.buf[0] < 2)
				uart_frame.buf[0] = 2;
			motor.END_speed_scale = uart_frame.buf[0];
		break;

	case CMD_Accelerated_LEVEL:	/* 1~15 */
			if(uart_frame.buf[0] > 15)
				uart_frame.buf[0] = 15;
			if(uart_frame.buf[0] < 1)
				uart_frame.buf[0] = 1;
			motor.accelerated = uart_frame.buf[0];
		break;

	case CMD_deceleration_LEVEL: /* 1~15 */
			if(uart_frame.buf[0] > 15)
				uart_frame.buf[0] = 15;
			if(uart_frame.buf[0] < 1)
				uart_frame.buf[0] = 1;
			motor.deceleration = uart_frame.buf[0];
		break;
	case CMD_SPEED:

		ctr.ack=1;

		if(uart_frame.buf[0]>rx_setparam.treadmills_speed_max){uart_frame.buf[0]= rx_setparam.treadmills_speed_max ;}
		scale=uart_frame.buf[0];
		motor.set_speed_scale=scale*10;
		break;
	case CMD_ACK:
		ctr.ack=1;
		break;
	case CMD_RESET:
		break;
	case CMD_WRITE_PARAM:
		break;
	case CMD_READ_PARAM:
		uart_frame_tx(CMD_READ_PARAM, 0x5Au);  /* 新固件应答标识，上控验证用 */
		break;
	case CMD_TEST:
//		over_current=OVER_CURRENT_MAX_TEST;
		motor.accelerated = 80;
		motor.deceleration = 80;
		break;
	case CMD_STATUS_URGENT_STOP:
		/* E07: emergency key/off-trigger -> fast braking profile. */
		motor_emergency_brake_set(ERROR_07);
		break;
  case CMD_TREADMILLS_SPEED_MAX:
    rx_setparam.treadmills_speed_max = uart_frame.buf[0];
	  motor.adjust_speed_max = rx_setparam.treadmills_speed_max*10;
    motor.voltage_param =( ((rx_setparam.voltage_max - rx_setparam.voltage_min )*1.0) /((rx_setparam.treadmills_speed_max*10) - 100) );/* dV/d(speed) */
    break;
  case CMD_VOLTAGE_MAX:
    rx_setparam.voltage_max = uart_frame.buf[0];
	  motor.adjust_max_voltage = rx_setparam.voltage_max;
    motor.voltage_param = ((rx_setparam.voltage_max - rx_setparam.voltage_min) * 1.0f) /
                          ((rx_setparam.treadmills_speed_max * 10) - 100);
    /* 过压阈值 = 最大电压×130%，直接换算至ADC量纲（1V≈7.937ADC，×1032/100≈×1.3×7.937） */
    over_voltage_adc = (u16)((u32)rx_setparam.voltage_max * 1032u / 100u);
    break;
  case CMD_VOLTAGE_MIN:
    rx_setparam.voltage_min = uart_frame.buf[0];
		motor.adjust_min_voltage = rx_setparam.voltage_min;
    motor.voltage_param =( ((rx_setparam.voltage_max - rx_setparam.voltage_min )*1.0) /((rx_setparam.treadmills_speed_max*10) - 100) );
    break;
  case CMD_OVER_CURRENT_MAX:
    motor.adjust_ove_curren = uart_frame.buf[0]*10;
    over_current = motor.adjust_ove_curren;
    break;
  case CMD_OVER_CURRENT_LOW_SPD:
    /* 上控下发×10单位值（如23），下控×10得ADC门限（23×10=230≈8.4A）
     * 不下发或发0时保持默认（OVER_CURRENT_MAX_LOW_SPEED=170≈6A） */
    if (uart_frame.buf[0] > 0u)
        over_current_low_spd = (u16)uart_frame.buf[0] * 10u;
    break;
  case CMD_OVER_VALTAGE_MAX:
    /* 过压阈值现由CMD_VOLTAGE_MAX×130%自动计算，此命令忽略 */
    break;
  case CMD_KIV_1KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_1KM_INDEX] = kiv_val;
    speed_param[0].kiv = (rx_setparam.kiv[KIV_1KM_INDEX] );
    speed_param[1].kiv = (rx_setparam.kiv[KIV_1KM_INDEX] );
    break;
  case CMD_KIV_2KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_2KM_INDEX] = kiv_val;
    speed_param[2].kiv = (rx_setparam.kiv[KIV_2KM_INDEX] );
    break;
  case CMD_KIV_3KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_3KM_INDEX] = kiv_val;
    speed_param[3].kiv = (rx_setparam.kiv[KIV_3KM_INDEX] );
    break;
  case CMD_KIV_4KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_4KM_INDEX] = kiv_val;
    speed_param[4].kiv = (rx_setparam.kiv[KIV_4KM_INDEX] );
    break;
  case CMD_KIV_5KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_5KM_INDEX] = kiv_val;
    speed_param[5].kiv = (rx_setparam.kiv[KIV_5KM_INDEX] );
    break;
  case CMD_KIV_6KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_6KM_INDEX] = kiv_val;
    speed_param[6].kiv = (rx_setparam.kiv[KIV_6KM_INDEX] );
    break;
  case CMD_KIV_7KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_7KM_INDEX] = kiv_val;
    speed_param[7].kiv = (rx_setparam.kiv[KIV_7KM_INDEX] );
    break;
  case CMD_KIV_8KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_8KM_INDEX] = kiv_val;
    speed_param[8].kiv = (rx_setparam.kiv[KIV_8KM_INDEX] );
    break;
  case CMD_KIV_9KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_9KM_INDEX] = kiv_val;
    speed_param[9].kiv = (rx_setparam.kiv[KIV_9KM_INDEX] );
    break;
  case CMD_KIV_10KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_10KM_INDEX] = kiv_val;
    speed_param[10].kiv = (rx_setparam.kiv[KIV_10KM_INDEX] );
    break;
  case CMD_KIV_11KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_11KM_INDEX] = kiv_val;
    speed_param[11].kiv = (rx_setparam.kiv[KIV_11KM_INDEX] );
    break;
  case CMD_KIV_12KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_12KM_INDEX] = kiv_val;
    speed_param[12].kiv = (rx_setparam.kiv[KIV_12KM_INDEX] );
    break;
  case CMD_KIV_13KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_13KM_INDEX] = kiv_val;
    speed_param[13].kiv = (rx_setparam.kiv[KIV_13KM_INDEX] );
    break;
  case CMD_KIV_14KM:
    kiv_val = ClampKiv(uart_frame.buf[0]);
    rx_setparam.kiv[KIV_14KM_INDEX] = kiv_val;
    speed_param[14].kiv = (rx_setparam.kiv[KIV_14KM_INDEX] );
    break;
  default:
    break;
	}
	/* 新固件：对所有配置命令回显（cmd+buf[0]），上控逐行ACK验证用
	 * CMD_PID_P/I/D 已有独立回发，此处排除避免重复；CMD_READ_PARAM 有特殊应答 */
	if (uart_frame.cmd >= (u8)CMD_TREADMILLS_SPEED_MAX &&
	    uart_frame.cmd != (u8)CMD_PID_P &&
	    uart_frame.cmd != (u8)CMD_PID_I &&
	    uart_frame.cmd != (u8)CMD_PID_D &&
	    uart_frame.cmd != (u8)CMD_READ_PARAM) {
		uart_frame_tx(uart_frame.cmd, uart_frame.buf[0]);
	}
}

void Uart1_IRQHandler(void)
{
		uint8_t reg;
    if(TRUE == Uart_GetStatus(M0P_UART1, UartRC))
    {
        Uart_ClrStatus(M0P_UART1, UartRC);

        reg = Uart_ReceiveData(M0P_UART1);

				Enter_queue(&rx_queue,reg);
    }
}

