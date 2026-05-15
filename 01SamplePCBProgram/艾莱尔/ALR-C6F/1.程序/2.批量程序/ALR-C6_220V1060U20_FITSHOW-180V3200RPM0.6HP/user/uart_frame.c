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

uart0_frame_t uart0_frame;
uart0_frame_t uart0_tx_frame;

void uart_frame_init(void)
{
	Create_Queue(&UART1_rx_queue,&UART1_rx_buf[0],UART1_RX_BUFF_SIZE);    //调用Create_Queue函数，创建一个队列 rx_queue，并将 rx_buf 数组作为队列的缓冲区，RX_BUFF_SIZE 作为队列的最大容量
	Create_Queue(&UART0_rx_queue,&UART0_rx_buf[0],UART0_RX_BUFF_SIZE);
  
  //下控通讯
	uart_frame.sof=START_OF_FRAME;    //(0X7F)
	uart_frame.eof=END_OF_FRAME;      //(0X7E)
	uart_tx_frame.sof=START_OF_FRAME;
	uart_tx_frame.eof=END_OF_FRAME;
  
  //蓝牙模块通讯
  uart0_frame.sof=0X02;             
	uart0_tx_frame.sof=0X02; 
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

//蓝牙模块发送
void uart0_frame_tx_0(u8 cmd,u8 subcmd)
{
	u8 xor=0;     //用于计算校验和  异或
	uart0_tx_frame.cmd=cmd;
	uart0_tx_frame.subcmd=subcmd;
	xor=cmd^subcmd;
	uart0_tx_frame.buf[0]=xor;    //用于存储校验和
	uart0_tx_frame.buf[1]=0X03;  
	UART_Send_Buf(UART0,(u8*)&uart0_tx_frame,5);  //调用UART_Send_Buf函数，将uart_tx_frame结构体的地址作为参数，以及 uart_tx_frame 的长度加上 4（包括起始帧、命令、长度、数据、校验和）作为参数，发送 UART 数据
}


void uart0_frame_tx_1(u8 cmd,u8 subcmd,u8 dat)
{
	u8 xor=0;     //用于计算校验和  异或
	uart0_tx_frame.cmd=cmd;
	uart0_tx_frame.subcmd=subcmd;
	xor=cmd^subcmd;
	uart0_tx_frame.buf[0]=dat;
	xor^=dat;
	uart0_tx_frame.buf[1]=xor;    //用于存储校验和
	uart0_tx_frame.buf[2]=0X03;  
	UART_Send_Buf(UART0,(u8*)&uart0_tx_frame,6);  //调用UART_Send_Buf函数，将uart_tx_frame结构体的地址作为参数，以及 uart_tx_frame 的长度加上 4（包括起始帧、命令、长度、数据、校验和）作为参数，发送 UART 数据
}


void uart0_frame_tx_2(u8 cmd,u8 subcmd,u8 dat0,u8 dat1)
{
	u8 xor=0;     //用于计算校验和  异或
	uart0_tx_frame.cmd=cmd;
	uart0_tx_frame.subcmd=subcmd;
	xor=cmd^subcmd;
	uart0_tx_frame.buf[0]=dat0;
	xor^=dat0;
	uart0_tx_frame.buf[1]=dat1;
	xor^=dat1;
  uart0_tx_frame.buf[2]=xor;    //用于存储校验和
	uart0_tx_frame.buf[3]=0X03;   
	UART_Send_Buf(UART0,(u8*)&uart0_tx_frame,7);
}


void uart0_frame_tx_3(u8 cmd,u8 subcmd,u8 dat0,u8 dat1,u8 dat2)
{
	u8 xor=0;     //用于计算校验和  异或
	uart0_tx_frame.cmd=cmd;
	uart0_tx_frame.subcmd=subcmd;
	xor=cmd^subcmd;
	uart0_tx_frame.buf[0]=dat0;
	xor^=dat0;
	uart0_tx_frame.buf[1]=dat1;
	xor^=dat1;
  uart0_tx_frame.buf[2]=dat2;
	xor^=dat2;
  uart0_tx_frame.buf[3]=xor;    //用于存储校验和
	uart0_tx_frame.buf[4]=0X03;     
	UART_Send_Buf(UART0,(u8*)&uart0_tx_frame,8);
}


void uart0_frame_tx_4(u8 cmd,u8 subcmd,u8 dat0,u8 dat1,u8 dat2,u8 dat3)
{
	u8 xor=0;     //用于计算校验和  异或
	uart0_tx_frame.cmd=cmd;
	uart0_tx_frame.subcmd=subcmd;
	xor=cmd^subcmd;
	uart0_tx_frame.buf[0]=dat0;
	xor^=dat0;
	uart0_tx_frame.buf[1]=dat1;
	xor^=dat1;
  uart0_tx_frame.buf[2]=dat2;
	xor^=dat2;
  uart0_tx_frame.buf[3]=dat3;
	xor^=dat3;
  uart0_tx_frame.buf[4]=xor;    //用于存储校验和
	uart0_tx_frame.buf[5]=0X03;    
	UART_Send_Buf(UART0,(u8*)&uart0_tx_frame,9);
}


void uart0_frame_tx_12(u8 cmd,u8 subcmd,u8 dat0,u8 dat1,u8 dat2,u8 dat3,u8 dat4,u8 dat5,u8 dat6,u8 dat7,u8 dat8,u8 dat9,u8 dat10,u8 dat11)
{
	u8 xor=0;     //用于计算校验和  异或
	uart0_tx_frame.cmd=cmd;
	uart0_tx_frame.subcmd=subcmd;
	xor=cmd^subcmd;
	uart0_tx_frame.buf[0]=dat0;
	xor^=dat0;
	uart0_tx_frame.buf[1]=dat1;
	xor^=dat1;
  uart0_tx_frame.buf[2]=dat2;
	xor^=dat2;
  uart0_tx_frame.buf[3]=dat3;
	xor^=dat3;
  uart0_tx_frame.buf[4]=dat4;
	xor^=dat4;
  uart0_tx_frame.buf[5]=dat5;
	xor^=dat5;  
	uart0_tx_frame.buf[6]=dat6;
	xor^=dat6;
  uart0_tx_frame.buf[7]=dat7;
	xor^=dat7;
  uart0_tx_frame.buf[8]=dat8;
	xor^=dat8;
  uart0_tx_frame.buf[9]=dat9;
	xor^=dat9;
	uart0_tx_frame.buf[10]=dat10;
	xor^=dat10;
	uart0_tx_frame.buf[11]=dat11;
	xor^=dat11;  
  uart0_tx_frame.buf[12]=xor;    //用于存储校验和
	uart0_tx_frame.buf[13]=0X03;   
	UART_Send_Buf(UART0,(u8*)&uart0_tx_frame,17);
}

//蓝牙模组接收指令处理
void uart0_frame_loop(void)  //从接收队列中获取 UART 数据
{
   static u8 sm=0,len=0;    // 静态变量用于保存状态和长度信息
   static u8 *ptr;          // 指向数据缓冲区的指针
   u8 datt;                 // 用于保存从队列中读取的数据
   u8 rett;                 // 用于保存队列读取操作的返回值

   rett=Denter_queue(&UART0_rx_queue,&datt);   // 从UART0_rx_queue队列中读取数据
   if(rett)    // 如果成功读取到数据
   {
	   switch(sm)
	   {
	   case 1:   // 状态1，接收命令字节
		   uart0_frame.cmd=datt;   // 保存命令字节    
       sm=2;
		   break;
	   case 2:   // 状态2，接收子命令字节
		   uart0_frame.subcmd=datt;   // 保存子命令字节
		   len=0;    // 清零数据长度计数器
		   ptr=&uart0_frame.buf[0];    // 指向数据缓冲区的起始位置
		   sm=3;
		   break;
	   case 3:   // 状态3，接收数据字节
		   if(datt==0x03)
		   {
         UART0_cmd_proc();
         sm=0;
		   } 
       ptr[len]=datt;       
		   len++;
		   break;
	   default:
       if(datt==0X02){sm=1;}   // 如果读取到的数据是起始字节,切换到状态1
		   break;
	   }
   }
}


// 蓝牙模组接收指令处理
void UART0_cmd_proc(void)
{   
    uint8_t speed = treadmills.param.speed / 10;// 提取常用变量
    uint16_t seconds = treadmills.param.second;
    uint32_t distance = (uint32_t)treadmills.param.distance;
    uint16_t calorie = treadmills.param.calorie / 100;
    switch (uart0_frame.cmd)
    {
    case 0x50: // 获取设备信息
        switch (uart0_frame.subcmd)
        {
        case 0x00:
            uart0_frame_tx_4(0x50, 0x00, 0xE1, 0x00, 0x03, 0x00);//0xE1, 0x01
            break;
        case 0x02:
            uart0_frame_tx_2(0x50, 0x02, 0x64, 0x0A);//10.0km,1.0
            break;
        case 0x03:       
            uart0_frame_tx_3(0x50, 0x03, 0x00, 0x00, 0x02);    
            break;
        default:
            break;
        }
        break;

    case 0x51: // 获取设备状态
        switch (treadmills.status)
        {
        case STATUS_WAIT:
            uart0_frame_tx_0(0x51, 0x00);
            break;
        case STATUS_RUN:
            uart0_frame_tx_1(0x51, 0x02, 0x03);
            break;
        case STATUS_STOP_OVER:
        case STATUS_RUNNING:
        case STATUS_STOP_WAIT:
        case STATUS_PAUSE:
            uart0_frame_tx_12(0x51,
                              treadmills.status == STATUS_STOP_OVER ? 0x01 :
                              treadmills.status == STATUS_RUNNING   ? 0x03 :
                              treadmills.status == STATUS_STOP_WAIT ? 0x04 : 0x0A,
                              speed, 0x00, seconds & 0xFF, (seconds >> 8) & 0xFF,
                              distance & 0xFF, (distance >> 8) & 0xFF,
                              calorie & 0xFF, (calorie >> 8) & 0xFF,
                              0x00, 0x00, 0x00, 0x00);
            break;
        case STATUS_ERROR:
            uart0_frame_tx_1(0x51, 0x05, 0x00);
            break;
        default:
            break;
        }
        break;

    case 0x53: // 设备控制
        switch (uart0_frame.subcmd)
        {
        case 0x01: // 启动/恢复
            if ((treadmills.status == STATUS_WAIT || treadmills.status == STATUS_PAUSE) && !treadmills.error_code)
            {
                uart0_frame_tx_1(0x53, 0x01, 0x03);
                treadmills.status = STATUS_RUN;
                key_mask = 0xFF;
                beep_set(1, DEFAULT_ON_TICKS, DEFAULT_OFF_TICKS);
            }
            else
            {
                beep_set(2, DEFAULT_ON_TICKS, DEFAULT_OFF_TICKS);
            }
            break;
        case 0x02: // 设置速度和坡度
            if (treadmills.status == STATUS_RUNNING)
            {
                treadmills.param.speed = uart0_frame.buf[0] * 10;
                uart1_frame_tx(CMD_SPEED, treadmills.param.speed / 10);
                treadmills.display.index = 0;
                treadmills.display.mode = DISP_MODE_SINGLE;
                uart0_frame_tx_2(0x53, 0x02, treadmills.param.speed / 10, 0x00);
                beep_set(1, DEFAULT_ON_TICKS, DEFAULT_OFF_TICKS);
            }
            break;
        case 0x03: // 停止
            if (treadmills.status == STATUS_RUNNING || treadmills.status == STATUS_PAUSE)
            {
                PAUSE_FLAG = 0;
                uart0_frame_tx_0(0x53, 0x03);
                treadmills.status = STATUS_STOP;
                key_mask = 0x00;
                beep_set(1, DEFAULT_ON_TICKS, DEFAULT_OFF_TICKS);
            }
            break;
        case 0x0A: // 暂停
            if (treadmills.status == STATUS_RUNNING)
            {
                PAUSE_FLAG = 1;
                uart0_frame_tx_0(0x53, 0x0A);
                treadmills.status = STATUS_STOP;
                key_mask = 0x00;
                beep_set(1, DEFAULT_ON_TICKS, DEFAULT_OFF_TICKS);
            }
            break;
        default:
            break;
        }
        break;

    default:
        break;
    }
}

