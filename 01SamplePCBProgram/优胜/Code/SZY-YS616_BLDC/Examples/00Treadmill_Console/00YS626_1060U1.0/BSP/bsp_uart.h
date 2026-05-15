#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include "sys_aerobuf.h"

// 频率与波特率配置
#define BSP_UART_UCLK_FREQ        48000000   // 核心主频 48MHz
#define UART_LOWER_BAUD           4800       // 下控波特率
#define UART_BLE_BAUD             115200     // 蓝牙波特率

// 外部缓冲区引用声明
extern AeroBuf_t uart1_rx_buf; 
extern AeroBuf_t uart2_rx_buf;

// API接口 
void BSP_Uart_Init(void);
void BSP_Uart_WriteByte(UART_TypeDef *UARTx, uint8_t byte);

#endif
