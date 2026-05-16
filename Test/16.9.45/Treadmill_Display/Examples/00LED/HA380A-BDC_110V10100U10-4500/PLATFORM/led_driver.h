/**
 * @file led_driver.h
 * @brief TM1620 / TM1640 / TM1652 LED HAL: set LED_DRIVER_TYPE, pins, use LED_* API.
 */

#ifndef __LED_DRIVER_H
#define __LED_DRIVER_H

#include "main.h"

#define LED_DRIVER_TM1620   1
#define LED_DRIVER_TM1640   2
#define LED_DRIVER_TM1652   3

#define LED_DRIVER_TYPE     LED_DRIVER_TM1640


/* TM1620: STB PB01, CLK PA03, DIN PA04 */
#define LED_TM1620_PORT_A       CW_GPIOA
#define LED_TM1620_PORT_B       CW_GPIOB
#define LED_TM1620_PIN_CLK      GPIO_PIN_3
#define LED_TM1620_PIN_DIN      GPIO_PIN_4
#define LED_TM1620_PIN_STB      GPIO_PIN_1

#define LED_TM1620_STB_LOW()    PB01_SETLOW()
#define LED_TM1620_STB_HIGH()   PB01_SETHIGH()
#define LED_TM1620_CLK_LOW()    PA03_SETLOW()
#define LED_TM1620_CLK_HIGH()   PA03_SETHIGH()
#define LED_TM1620_DIN_LOW()    PA04_SETLOW()
#define LED_TM1620_DIN_HIGH()   PA04_SETHIGH()

/* TM1640: DIN PA03, CLK PA04 */
#define LED_TM1640_PORT_A       CW_GPIOA
#define LED_TM1640_PIN_DIN      GPIO_PIN_3
#define LED_TM1640_PIN_CLK      GPIO_PIN_4

#define LED_TM1640_CLK_LOW()    PA04_SETLOW()
#define LED_TM1640_CLK_HIGH()   PA04_SETHIGH()
#define LED_TM1640_DIN_LOW()    PA03_SETLOW()
#define LED_TM1640_DIN_HIGH()   PA03_SETHIGH()

/* TM1652: PB04 UART2 TX, 19200 8O1 */
#define LED_TM1652_PORT_B       CW_GPIOB
#define LED_TM1652_PIN_DAT      GPIO_PIN_4
#define LED_TM1652_UARTx        CW_UART2
#define TM1652_PCLK_FREQ        48000000UL


#define TM1620_BUF_SIZE     12
#define TM1640_BUF_SIZE     16
#define TM1640_GRID_NUM     16  /* 16 TX bytes for GR16 */
#define TM1652_GRID_MAX      5
#define TM1652_CMD_DATA     0x08
#define TM1652_CMD_CTRL     0x18
#define TM1652_CTRL_OFF     0x00


void LED_Init(void);
void LED_Refresh(void);
void LED_SetBrightness(uint8_t level);
void LED_WriteBuf(uint8_t grid, uint8_t seg_data);
void LED_ClearBuf(void);
void LED_FillBuf(uint8_t data);


#endif /* __LED_DRIVER_H */
