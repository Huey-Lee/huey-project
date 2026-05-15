#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include "sys_aerobuf.h"

#define BSP_UART_UCLK_FREQ        48000000U
/** UART1：显示板/下位链路 */
#define UART_LINK_BAUD            4800U
/** UART2：蓝牙模组 PB02=TX PB03=RX，8N1 */
#define UART_BLE_BAUD             9600U

extern AeroBuf_t uart1_rx_buf;
extern AeroBuf_t uart2_rx_buf;

void BSP_Uart_Init(void);
void BSP_Uart2_Init(void);
void BSP_Uart_WriteByte(UART_TypeDef *UARTx, uint8_t byte);

#endif
