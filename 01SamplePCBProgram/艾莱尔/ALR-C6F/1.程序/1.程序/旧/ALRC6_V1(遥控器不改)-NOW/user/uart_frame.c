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

void UART1_cmd_proc(void);  //下控通讯接收指令处理

T_QUEUE UART1_rx_queue;

u8 UART1_rx_buf[UART1_RX_BUFF_SIZE];

uart_frame_t uart_frame;
uart_frame_t uart_tx_frame;


void uart_frame_init(void)
{
	Create_Queue(&UART1_rx_queue,&UART1_rx_buf[0],UART1_RX_BUFF_SIZE);    //调用Create_Queue函数，创建一个队列 rx_queue，并将 rx_buf 数组作为队列的缓冲区，RX_BUFF_SIZE 作为队列的最大容量
  
  //下控通讯
	uart_frame.sof=START_OF_FRAME;    //(0X7F)
	uart_frame.eof=END_OF_FRAME;      //(0X7E)
	uart_tx_frame.sof=START_OF_FRAME;
	uart_tx_frame.eof=END_OF_FRAME;
}

//下控通讯发送
void uart1_frame_tx(u8 cmd,u8 dat)
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
	UART_Send_Buf(UART1,(u8*)&uart_tx_frame,uart_tx_frame.len+6);  //调用UART_Send_Buf函数，将uart_tx_frame结构体的地址作为参数，以及uart_tx_frame的长度加上 6（包括起始帧、命令、长度、数据、校验和和结束帧）作为参数，发送 UART 数据
}

void uart1_frame_tx_2(u8 cmd,u8 dat0,u8 dat1)
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
	UART_Send_Buf(UART1,(u8*)&uart_tx_frame,uart_tx_frame.len+6);
}


//---------------------------------------------------------------
//下控通讯接收指令处理
void uart1_frame_loop(void)  //从接收队列中获取 UART 数据
{
   static u8 sm=0,len=0;
   static u8 *ptr;
   u8 dat;
   u8 ret;
   //ET0 = 0;	//允许中断
   ret=Denter_queue(&UART1_rx_queue,&dat);
   //ET0 = 1;	//允许中断
   if(ret)    //ret=1 队列中有数据，数据出队列，即发送数据
   {
	   if(dat==uart_frame.sof){sm=1;treadmills.ack=1;return;}
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
		   ptr[len]=dat;
		   len++;
		   if(len>=(uart_frame.len+2)){sm=4;}
		   break;
	   case 4:
		   if(dat==uart_frame.eof)
		   {
			   ptr=&uart_frame.cmd;
			   dat=0;
			   for(len=0;len<(uart_frame.len+2);len++)
			   {
				  dat^=ptr[len];
			   }
			   if(dat==ptr[len]){UART1_cmd_proc();}
		   }
			sm=0;
		   break;
	   default:
		   break;
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
