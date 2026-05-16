/* user_usart.h - UART1 API, ring buffer, init. */
#ifndef USER_USART_H
#define USER_USART_H
#include "stdio.h"
#include "hc32f005.h"
#include "ringfifo.h"
#include "uart.h"
#include "bt.h"

void uart_init(uint32_t bound);
uint8_t uart_send(uint8_t *data, uint8_t len);
int Int_To_String(signed long Int_Num, unsigned char String[]);
int Float_To_String(float fNum, unsigned char str[]);
void App_UartInit(void);
void App_PortInit(void);
void uart1_ringfifo_init(void);
unsigned int uart1_rx_ringfifo_get(uint8_t *buffer, unsigned int len);
unsigned char *str_hex(unsigned char *str);
void user_usart_init(void);

#endif
