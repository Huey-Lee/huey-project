#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include "sys_aerobuf.h"
#include "led_driver.h"

#define BSP_UART_UCLK_FREQ        48000000   /* PCLK 48MHz          */
#define UART_LOWER_BAUD           4800       /* 下控机通信波特率    */
#define UART_BLE_BAUD             115200     /* 蓝牙模块波特率      */

extern AeroBuf_t uart1_rx_buf;
/* uart2_rx_buf 仅在非 TM1652 驱动时存在（TM1652 占用 UART2）   */
#if (LED_DRIVER_TYPE != LED_DRIVER_TM1652)
extern AeroBuf_t uart2_rx_buf;
#endif

// API接口 
void BSP_Uart_Init(void);
void BSP_Uart_WriteByte(UART_TypeDef *UARTx, uint8_t byte);

#endif
