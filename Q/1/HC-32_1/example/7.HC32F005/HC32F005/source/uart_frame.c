/*
 * uart_frame.c
 *
 *  Created on: 2019年12月11日
 *      Author: Administrator
 */
#include "uart_frame.h"
#include "queue.h"
#include "motor.h"
#include "uart.h"
#include "my_usart.h"	
#include "MY_TIM.h"
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
	Create_Queue(&rx_queue,&rx_buf[0],RX_BUFF_SIZE);    //调用Create_Queue函数，创建一个队列 rx_queue，并将 rx_buf 数组作为队列的缓冲区，RX_BUFF_SIZE 作为队列的最大容量

	uart_frame.sof=START_OF_FRAME;    //(0X7F)
	uart_frame.eof=END_OF_FRAME;      //(0X7E)

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
	u8 xor=0;     //用于计算校验和
	uart_tx_frame.cmd=cmd;
	xor^=cmd;
	uart_tx_frame.len=1;
	xor^=1;
	uart_tx_frame.buf[0]=dat;
	xor^=dat;
	uart_tx_frame.buf[1]=xor;    //用于存储校验和
	uart_tx_frame.crch=0;
	uart_tx_frame.crcl=END_OF_FRAME;
	UART_Send_Buf(UART0,(u8*)&uart_tx_frame,uart_tx_frame.len+6);  //调用 UART_Send_Buf 函数，将 uart_tx_frame 结构体的地址作为参数，以及 uart_tx_frame 的长度加上 6（包括起始帧、命令、长度、数据、校验和和结束帧）作为参数，发送 UART 数据
}

void uart_frame_tx_2(u8 cmd,u8 dat0,u8 dat1)
{
	u8 xor=0;      //用于计算校验和
	uart_tx_frame.cmd=cmd;
	xor^=cmd;
	uart_tx_frame.len=2;
	xor^=2;
	uart_tx_frame.buf[0]=dat0;
	xor^=dat0;
	uart_tx_frame.buf[1]=dat1;
	xor^=dat1;
	uart_tx_frame.crch=xor;    //用于存储校验和
	uart_tx_frame.crcl=0;
	uart_tx_frame.eof=END_OF_FRAME;
	UART_Send_Buf(UART0,(u8*)&uart_tx_frame,uart_tx_frame.len+6);
}

//base time:500ms
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
			uart_frame_tx(CMD_ACK,1);
		}
		ctr.ack=0;
	}
}
extern u8 uart_time_error;
void uart_frame_loop(void)  //从接收队列中获取 UART 数据
{
   static u8  sm=0,len=0;
   static u8  *ptr;
   u8 dat;
   u8 ret;

   ret=Denter_queue(&rx_queue,&dat);
   if(ret)    //ret=1 队列中有数据，数据出队列，即发送数据
   {
	   if(dat==uart_frame.sof){sm=1;uart_time_error=0;return;}	//首字节
	   switch(sm)
	   {
	   case 1:
		   uart_frame.cmd=dat;														//命令
		   sm=2;
		   break;
	   case 2:
		   uart_frame.len=dat;														//数据长度
		   if(uart_frame.len>2){sm=0;return;}
		   len=0;
		   ptr=&uart_frame.buf[0];
		   sm=3;
		   break;
	   case 3:
		   ptr[len]=dat;																	//数据内容  +   两个校验位
		   len++;
		   if(len>=(uart_frame.len+2)){sm=4;}
		   break;
	   case 4:
		   if(dat==uart_frame.eof)												//结束标志
		   {
			   ptr=&uart_frame.cmd;
			   dat=0;
			   for(len=0;len<(uart_frame.len+2);len++)
			   {
				  dat^=ptr[len];
			   }
			   if(dat==ptr[len]){cmd_proc();}
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
void cmd_proc(void)
{
	u8 scale=0;
	switch(uart_frame.cmd)
	{
	case CMD_STATUS:
		
		if(motor.en !=uart_frame.buf[0])
		{

			if(uart_frame.buf[0]==0)
			{
				ctr.ack=1;
				motor.en=0;
				motor.set_speed_scale=MT_STOP_SPEED;//设置停止速度
				motor.kiv=speed_param[motor.index].kiv;
				ctr.status=STATUS_STOP;							//进入停止模式
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
	case CMD_STA_LEVEL:			//between 1~5
//		 if(uart_frame.buf[0] > 80){uart_frame.buf[0] = 80;}
//		 motor.sta_speed_scale=uart_frame.buf[0];
	switch(uart_frame.buf[0])
	{
		case(1):motor.start_valtage = 40;motor.start_add_val = 3;break;//5V			//110V大功率
		case(2):motor.start_valtage = 80;motor.start_add_val = 5;break;//10V		//220V大功率
		case(3):motor.start_valtage = 120;motor.start_add_val = 5;break;//10V
		case(4):motor.start_valtage = 160;motor.start_add_val = 5;break;//10V
		case(5):motor.start_valtage = 200;motor.start_add_val = 8;break;//10V
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

	case CMD_Accelerated_LEVEL:	//between 1~6
//			if(uart_frame.buf[0] > 7 || uart_frame.buf[0] < 1){uart_frame.buf[0] = 3;}
			 if(uart_frame.buf[0]>6)
				 uart_frame.buf[0]=6;
			 if(uart_frame.buf[0]<1)
				 uart_frame.buf[0]=1;
			 motor.accelerated=uart_frame.buf[0];
		break;

	case CMD_deceleration_LEVEL: //between 1~6
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
		motor.accelerated=50;
		motor.deceleration=50;
		break;
	case CMD_STATUS_URGENT_STOP:
		ctr.status = NULL;
		motor.status = NULL;
		tim0stop();
		tim6stop();

//		uart_frame_tx(CMD_STOP_OVER,1);
		break;
  case CMD_TREADMILLS_SPEED_MAX:
    rx_setparam.treadmills_speed_max = uart_frame.buf[0];
	  motor.adjust_speed_max = rx_setparam.treadmills_speed_max*10;
    motor.voltage_param =( ((rx_setparam.voltage_max - rx_setparam.voltage_min )*1.0) /((rx_setparam.treadmills_speed_max*10) - 100) );//获得单位速度下的电压
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
    rx_setparam.kiv[KIV_1KM_INDEX] = uart_frame.buf[0];
    speed_param[0].kiv = (rx_setparam.kiv[KIV_1KM_INDEX] ); //kiv 值2023.2.17
    speed_param[1].kiv = (rx_setparam.kiv[KIV_1KM_INDEX] ); //kiv 值2023.2.17
    break;
  case CMD_KIV_2KM:
    rx_setparam.kiv[KIV_2KM_INDEX] = uart_frame.buf[0];
    speed_param[2].kiv = (rx_setparam.kiv[KIV_2KM_INDEX] ); //kiv 值2023.2.17		
    break;
  case CMD_KIV_3KM:
    rx_setparam.kiv[KIV_3KM_INDEX] = uart_frame.buf[0];
    speed_param[3].kiv = (rx_setparam.kiv[KIV_3KM_INDEX] ); //kiv 值2023.2.17	
    break;
  case CMD_KIV_4KM:
    rx_setparam.kiv[KIV_4KM_INDEX] = uart_frame.buf[0];
    speed_param[4].kiv = (rx_setparam.kiv[KIV_4KM_INDEX] ); //kiv 值2023.2.17
    break;
  case CMD_KIV_5KM:
    rx_setparam.kiv[KIV_5KM_INDEX] = uart_frame.buf[0];
    speed_param[5].kiv = (rx_setparam.kiv[KIV_5KM_INDEX] ); //kiv 值2023.2.17	
    break;
  case CMD_KIV_6KM:
    rx_setparam.kiv[KIV_6KM_INDEX] = uart_frame.buf[0];
    speed_param[6].kiv = (rx_setparam.kiv[KIV_6KM_INDEX] ); //kiv 值2023.2.17			
    break;
  case CMD_KIV_7KM:
    rx_setparam.kiv[KIV_7KM_INDEX] = uart_frame.buf[0];
    speed_param[7].kiv = (rx_setparam.kiv[KIV_7KM_INDEX] ); //kiv 值2023.2.17		
    break;
  case CMD_KIV_8KM:
    rx_setparam.kiv[KIV_8KM_INDEX] = uart_frame.buf[0];
    speed_param[8].kiv = (rx_setparam.kiv[KIV_8KM_INDEX] ); //kiv 值2023.2.17	
    break;
  case CMD_KIV_9KM:
    rx_setparam.kiv[KIV_9KM_INDEX] = uart_frame.buf[0];
    speed_param[9].kiv = (rx_setparam.kiv[KIV_9KM_INDEX] ); //kiv 值2023.2.17		
    break;
  case CMD_KIV_10KM:
    rx_setparam.kiv[KIV_10KM_INDEX] = uart_frame.buf[0];
    speed_param[10].kiv = (rx_setparam.kiv[KIV_10KM_INDEX] ); //kiv 值2023.2.17
    break;
  case CMD_KIV_11KM:
    rx_setparam.kiv[KIV_11KM_INDEX] = uart_frame.buf[0];
    speed_param[11].kiv = (rx_setparam.kiv[KIV_11KM_INDEX] ); //kiv 值2023.2.17		
    break;
  case CMD_KIV_12KM:
    rx_setparam.kiv[KIV_12KM_INDEX] = uart_frame.buf[0];
    speed_param[12].kiv = (rx_setparam.kiv[KIV_12KM_INDEX] ); //kiv 值2023.2.17
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

