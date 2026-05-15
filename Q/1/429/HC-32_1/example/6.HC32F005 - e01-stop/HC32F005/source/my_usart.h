#ifndef __MY_USART_H
#define __MY_USART_H
#include "stdio.h"	
#include "hc32f005.h"
#include "ringfifo.h"
#include "uart.h"
#include "bt.h"



//////////////////////////////////////////////////////////////////////////////////
//typedef enum _cmd_e
//{
//	CMD_START=1,
//	CMD_STOP,
//	CMD_SPEED,
//	CMD_ACK,
//	CMD_ERROR,
//	CMD_RESET,
//	CMD_STOP_OVER,
//	CMD_WRITE_PARAM,
//	CMD_READ_PARAM,
//	CMD_TEST,
//	CMD_STATUS_INQUIRE,      //INQUIRE 询问
//	CMD_STATUS_URGENT_STOP,  //URGENT_STOP 紧急停止
//  
//  CMD_TREADMILLS_SPEED_MAX,   // 12 跑步机最大速度 0x0C
//  CMD_VOLTAGE_MAX,            //13 最大电压  0x0D
//  CMD_VOLTAGE_MIN,            //14 最小电压   0x0E
//  CMD_OVER_CURRENT_MAX,       //15 最大电流值  0x0F
//  CMD_KIV_1KM,                    //16  KIV参数   0x10 
//  CMD_KIV_2KM,                    //17  KIV参数   0x11 
//  CMD_KIV_3KM,                    //18  KIV参数   0x12 
//  CMD_KIV_4KM,                    //19  KIV参数   0x13 
//  CMD_KIV_5KM,                    //20  KIV参数   0x14 
//  CMD_KIV_6KM,                    //21  KIV参数   0x15 
//  CMD_KIV_7KM,                    //22  KIV参数   0x16 
//  CMD_KIV_8KM,                    //23  KIV参数   0x17 
//  CMD_KIV_9KM,                    //24  KIV参数   0x18 
//  CMD_KIV_10KM,                   //25  KIV参数   0x19 
//  CMD_KIV_11KM,                   //26  KIV参数   0x1A 
//  CMD_KIV_12KM,                   //27  KIV参数   0x1B 
//	
//	CMD_STATUS_HEART,
//}cmd_e;








//如果想串口中断接收，请不要注释以下宏定义
void uart_init(uint32_t bound);
uint8_t uart_send(uint8_t *data, uint8_t len);
int Int_To_String(signed long Int_Num, unsigned char String[]);
int Float_To_String(float fNum, unsigned char str[]);
void App_UartInit(void);
void App_PortInit(void);
void uart1_ringfifo_init(void);
unsigned int uart1_rx_ringfifo_get(   uint8_t* buffer, unsigned int len);
unsigned char * str_hex(unsigned char *str);
void MY_UART_INIT(void);
//void uart_frame_loop(void);
#endif


