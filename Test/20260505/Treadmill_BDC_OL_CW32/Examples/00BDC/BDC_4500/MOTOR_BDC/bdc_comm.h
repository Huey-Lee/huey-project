#ifndef BDC_COMM_H
#define BDC_COMM_H

#include <stdint.h>

void user_usart_init(void);
void BDC_Uart_SendBytes(uint8_t *p, uint8_t len);
void bdc_uart_rx_irq_handler(void);

#endif
