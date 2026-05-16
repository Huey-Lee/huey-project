/*
 * bsp_uart.h  -  UART hardware abstraction for CW32L010 display board
 *
 * UART1: motor controller link  PA01(TX) / PA00(RX)
 * UART2: BLE module             PB02(TX) / PB03(RX)  (not used when TM1652 drives LED)
 *
 * Baud rate for UART1 is taken from TM_UART_BAUD in treadmills.h.
 * To change the baud rate, edit TM_UART_BAUD there and also update
 * CFG_UART_BAUDRATE in the lower board's config.h, then reflash both boards.
 */

#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include "sys_aerobuf.h"
#include "led_driver.h"
#include "treadmills.h"     /* TM_UART_BAUD */

#define BSP_UART_UCLK_FREQ      48000000u          /* PCLK 48 MHz */
#define UART_LOWER_BAUD         TM_UART_BAUD        /* matches lower board CFG_UART_BAUDRATE */
#define UART_BLE_BAUD           115200u             /* BLE module */

extern AeroBuf_t uart1_rx_buf;
#if (LED_DRIVER_TYPE != LED_DRIVER_TM1652)
extern AeroBuf_t uart2_rx_buf;
#endif

void BSP_Uart_Init(void);
void BSP_Uart_WriteByte(UART_TypeDef *UARTx, uint8_t byte);

#endif /* __BSP_UART_H */
