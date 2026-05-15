/*
 * uart.h
 *
 *  Created on: 2020Äê9ÔÂ28ÈÕ
 *      Author: Administrator
 */

#ifndef BSP_UART_H_
#define BSP_UART_H_
#include "common.h"


#define UART0 0
#define UART1 1

extern void uart_init(void);
//unsigned char Receive_Data(unsigned char UARTPort);
void UART_Send_Buf(UINT8 UARTPort, UINT8 *ptr,UINT8 len);
//void UART_Send_Value(UINT8 UARTPort, UINT16 val);


#endif /* BSP_UART_H_ */
