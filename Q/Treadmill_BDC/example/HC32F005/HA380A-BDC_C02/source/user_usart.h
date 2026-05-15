/* user_usart.h - UART1 API, ring buffer, init. */
#ifndef USER_USART_H
#define USER_USART_H
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
//	CMD_STATUS_INQUIRE,      //INQUIRE ѯ��
//	CMD_STATUS_URGENT_STOP,  //URGENT_STOP ���ֹͣ
//  
//  CMD_TREADMILLS_SPEED_MAX,   // 12 �ܲ�������ٶ� 0x0C
//  CMD_VOLTAGE_MAX,            //13 ����ѹ  0x0D
//  CMD_VOLTAGE_MIN,            //14 ��С��ѹ   0x0E
//  CMD_OVER_CURRENT_MAX,       //15 ������ֵ  0x0F
//  CMD_KIV_1KM,                    //16  KIV����   0x10 
//  CMD_KIV_2KM,                    //17  KIV����   0x11 
//  CMD_KIV_3KM,                    //18  KIV����   0x12 
//  CMD_KIV_4KM,                    //19  KIV����   0x13 
//  CMD_KIV_5KM,                    //20  KIV����   0x14 
//  CMD_KIV_6KM,                    //21  KIV����   0x15 
//  CMD_KIV_7KM,                    //22  KIV����   0x16 
//  CMD_KIV_8KM,                    //23  KIV����   0x17 
//  CMD_KIV_9KM,                    //24  KIV����   0x18 
//  CMD_KIV_10KM,                   //25  KIV����   0x19 
//  CMD_KIV_11KM,                   //26  KIV����   0x1A 
//  CMD_KIV_12KM,                   //27  KIV����   0x1B 
//	
//	CMD_STATUS_HEART,
//}cmd_e;








//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(uint32_t bound);
uint8_t uart_send(uint8_t *data, uint8_t len);
int Int_To_String(signed long Int_Num, unsigned char String[]);
int Float_To_String(float fNum, unsigned char str[]);
void App_UartInit(void);
void App_PortInit(void);
void uart1_ringfifo_init(void);
unsigned int uart1_rx_ringfifo_get(   uint8_t* buffer, unsigned int len);
unsigned char * str_hex(unsigned char *str);
void user_usart_init(void);
//void uart_frame_loop(void);
#endif


