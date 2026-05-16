/*
 * uart_frame.c
 *
 *  Created on: 2019年12月11日
 *      Author: Administrator
 */

#include "common.h"
#include "uart_frame.h"
#include "queue.h"
#include "uart.h"
#include "treadmills.h"
#include "param.h"
#include "led_disp.h"
#include "indecate.h"
#include "beep.h"
void cmd_proc(void);




T_QUEUE rx_queue;
u8 rx_buf[RX_BUFF_SIZE];
uart_frame_t uart_frame;

uart_frame_t uart_tx_frame;



#include "uart.h"

void uart0_init(void)
{
	InitialUART0_Timer3(2400);
	set_ES; /* Enable UART0 interrupt */
}


void uart_frame_init(void)
{
	Create_Queue(&rx_queue,&rx_buf[0],RX_BUFF_SIZE);

	uart_frame.sof=START_OF_FRAME;
	uart_frame.eof=END_OF_FRAME;

	uart_tx_frame.sof=START_OF_FRAME;
	uart_tx_frame.eof=END_OF_FRAME;
}

void uart_frame_tx(u8 cmd,u8 dat)
{
	uart_tx_frame.cmd=cmd;
	uart_tx_frame.len=1;
	uart_tx_frame.buf[0]=dat;
	UART_Send_Buf(UART0,(UINT8*)&uart_tx_frame,sizeof(uart_tx_frame));
}



void uart_frame_loop(void)
{
   static u8 sm=0,len=0;
   u8 dat;
   u8 ret;
   ret=Denter_queue(&rx_queue,&dat);
   if(ret)
   {
	   //UART_Send_Data(UART0,dat);
	   switch(sm)
	   {
	   case 1:
		   uart_frame.cmd=dat;
		   sm=2;
		   break;
	   case 2:
		   uart_frame.len=dat;
		   sm=3;
		   break;
	   case 3:
		   uart_frame.buf[0]=dat;
		   len=0;
		   sm=4;
		   break;
	   case 4:
		   if(dat==uart_frame.eof)
		   {
			   cmd_proc();
			   sm=0;
		   }
		   else
		   {
			   len++;
			   //if(len>2){sm=0;}
		   }
		   break;
	   default:
		   if(dat==uart_frame.sof){sm=1;}
		   break;
	   }
   }

}



void cmd_proc(void)
{

	switch(uart_frame.cmd)
	{
	case CMD_STATUS:
		break;
	case CMD_SPEED:
		break;
	case CMD_ACK:
			treadmills.ack=1;
			debug_printf("\r\n --->rply ack");
			LED1=1;//熄灭
			break;
	case CMD_RESET:

	//		ANL AUXR1,#01111111b ;software reset flag clear
	//		CLR EA
	//		MOV TA,#0Aah
	//		MOV TA,#55h
	//		ORL CHPCON,#10000000b ;software reset

			debug_printf("\r\n software reset");
			AUXR1 &=(0X7F);
			EA=0;
			TA=0XAA;
			TA=0X55;
			CHPCON |=(1<<7);
			//EA=1;
			break;
	case CMD_STOP_OVER:
		treadmills.status=STATUS_STOP_OVER;
		break;
	case CMD_ERROR:
		treadmills.error_code=uart_frame.buf[0];
		treadmills.status=STATUS_ERROR;
		break;
	case CMD_WRITE_PARAM:
//		uart_tx_frame.cmd=CMD_WRITE_PARAM;
//		uart_tx_frame.len=2;
//		uart_tx_frame.buf[0]=(u8)param.speed_adc;
//		uart_tx_frame.buf[1]=(~(u8)param.speed_adc);
//		UART_Send_Buf(UART0,(UINT8*)&uart_tx_frame,sizeof(uart_tx_frame));
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
			indecate.led_auto=LED_ON;
			indecate.led_speed=LED_ON;
			indecate.led_updp=LED_ON;
			indecate.led_downdp=LED_ON;

			indecate.led_time=LED_OFF;
			indecate.led_distance=LED_OFF;
			indecate.led_calorie=LED_OFF;
			set_seg_val(1234);
			set_seg_mode(SEG_MODE_NORMAL);
		}
		else if(uart_frame.buf[0] ==1)
		{
			indecate.led_auto=LED_OFF;
			indecate.led_speed=LED_OFF;
			indecate.led_updp=LED_OFF;
			indecate.led_downdp=LED_OFF;

			indecate.led_time=LED_ON;
			indecate.led_distance=LED_ON;
			indecate.led_calorie=LED_ON;
			set_seg_val(5678);
			set_seg_mode(SEG_MODE_FLASH_SELF);
		}
		else if(uart_frame.buf[0] ==2)
		{
			indecate.led_auto=LED_ON;
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
