#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include "sys_aerobuf.h"

// 频率与波特率配置
#define BSP_UART_UCLK_FREQ        48000000   // 核心主频 48MHz
#define UART_LOWER_BAUD           4800       /* 下控 MCU UART1 + 按键板 UART1：与按键板 plat_keylink 一致 */
/** UART2 PB03：接按键板 TX 收 Keylink；须与按键板串口波特率相同（本条工程与 UART_LOWER_BAUD 一致） */
#define UART_KEYLINK_BAUD         UART_LOWER_BAUD

// 外部缓冲区引用声明
extern AeroBuf_t uart1_rx_buf; 
extern AeroBuf_t uart2_rx_buf;

// API接口 
void BSP_Uart_Init(void);
void BSP_Uart_WriteByte(UART_TypeDef *UARTx, uint8_t byte);

#endif
