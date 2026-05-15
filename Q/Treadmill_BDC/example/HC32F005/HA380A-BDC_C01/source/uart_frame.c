/* uart_frame.c - 7E/7F frame pack/parse, cmds, display link, params. */
#include "uart_frame.h"
#include "queue.h"
#include "motor.h"
#include "uart.h"
#include "user_usart.h"	
#include "user_timer.h"
#include "pid.h"
extern volatile u16 over_current;
extern volatile u16 over_voltage;
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
				ctr.ack=1;
				motor.en=0;
				motor.set_speed_scale=MT_STOP_SPEED;/* min stop speed */
//				motor.kiv=speed_param[motor.index].kiv;
				ctr.status=STATUS_STOP;							/* stop request */
			}
			else if(uart_frame.buf[0]==1)
			{
				 ctr.ack=1;
			   motor.en=1;
			   ctr.status=STATUS_RUN;

			   if(uart_frame.buf[1] > motor.adjust_speed_max ){uart_frame.buf[1] = motor.adjust_speed_max;}
				 motor.adjust_sta_voltage = motor.start_valtage;
			   motor.set_speed_scale=uart_frame.buf[1]*10;
			   motor.cur_speed_scale=motor.sta_speed_scale;
			}
		}
		break;
	case CMD_PID_P:
		mt_pid.kp = (uart_frame.buf[0])/100;
		uart_frame_tx(CMD_PID_P,(u8)(mt_pid.kp*100));
	  break;
	case CMD_PID_I:
		mt_pid.ki = (uart_frame.buf[0])/100;
		uart_frame_tx(CMD_PID_I,(u8)(mt_pid.ki*100));
	  break;
	case CMD_PID_D:
		mt_pid.kd = (uart_frame.buf[0])/100;
		uart_frame_tx(CMD_PID_D,(u8)(mt_pid.kd*100));
	  break;
	case CMD_STA_LEVEL:			//between 1~10 (0 as fallback)
//		 if(uart_frame.buf[0] > 80){uart_frame.buf[0] = 80;}
//		 motor.sta_speed_scale=uart_frame.buf[0];
	if(uart_frame.buf[0] > 10)
	{
		uart_frame.buf[0] = 10;
	}
	switch(uart_frame.buf[0])
	{ // start_valtage，U= start÷8.13
		case(0): // 兜底：按最柔和启动
		case(1): motor.start_valtage = 40;	motor.start_add_val = 3; break; /* ~4.9V */
		case(2): motor.start_valtage = 80;	motor.start_add_val = 5; break; /* 保持原2档固定不变 */
		case(3): motor.start_valtage = 100;	motor.start_add_val = 5; break; /* ~12.3V */
		case(4): motor.start_valtage = 120;	motor.start_add_val = 5; break; /* ~14.8V */
		case(5): motor.start_valtage = 140;	motor.start_add_val = 6; break; /* ~17.2V */
		case(6): motor.start_valtage = 160;	motor.start_add_val = 6; break; /* ~19.7V */
		case(7): motor.start_valtage = 180;	motor.start_add_val = 7; break; /* ~22.1V */
		case(8): motor.start_valtage = 200;	motor.start_add_val = 8; break; /* ~24.6V */
		case(9): motor.start_valtage = 220;	motor.start_add_val = 9; break; /* ~27.1V */
		case(10):motor.start_valtage = 240;	motor.start_add_val = 10;break; /* ~29.5V */
		default: motor.start_valtage = 80;	motor.start_add_val = 5; break; /* 安全回退 */
	}
		break;
	case CMD_STATUS_INQUIRE:
		break;
	case CMD_END_LEVEL:			//between 13~25
//			if(uart_frame.buf[0] > 25 || uart_frame.buf[0] < 10){uart_frame.buf[0] = 15;}
			if(uart_frame.buf[0] > 25)
			{
			uart_frame.buf[0]=25;
			}
			if(uart_frame.buf[0] < 2)
			{
			uart_frame.buf[0]=2;
			}
			 motor.END_speed_scale=uart_frame.buf[0];
		break;

	case CMD_Accelerated_LEVEL:	//between 1~10
//			if(uart_frame.buf[0] > 7 || uart_frame.buf[0] < 1){uart_frame.buf[0] = 3;}
			 if(uart_frame.buf[0]>10)
				 uart_frame.buf[0]=10;
			 if(uart_frame.buf[0]<1)
				 uart_frame.buf[0]=1;
			 motor.accelerated=uart_frame.buf[0];
		break;

	case CMD_deceleration_LEVEL: //between 1~10
//			if(uart_frame.buf[0] > 7 || uart_frame.buf[0] < 1){uart_frame.buf[0] = 3;}
			 if(uart_frame.buf[0]>10)
				 uart_frame.buf[0]=10;
			 if(uart_frame.buf[0]<1)
				 uart_frame.buf[0]=1;
			 motor.deceleration=uart_frame.buf[0];
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
    motor.voltage_param =( ((rx_setparam.voltage_max - rx_setparam.voltage_min )*1.0) /((rx_setparam.treadmills_speed_max*10) - 100) );
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
  case CMD_OVER_VALTAGE_MAX:
    over_voltage = uart_frame.buf[0];

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

