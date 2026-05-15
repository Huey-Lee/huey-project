#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
 extern "C" {
#endif


/********************************************************************************************************
 *                                         C语言头文件
 ********************************************************************************************************/
#include  <stdarg.h>
#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
#include  <string.h>
#include  "stdint.h"

/********************************************************************************************************
 *                                         HC32标准库头文件
 ********************************************************************************************************/
#include "hc32f005.h"
#include "ddl.h"
#include "sysctrl.h"
#include "gpio.h"
#include "adc.h"
#include "uart.h"
#include "bt.h"
#include "flash.h"
#include "adt.h"
#include "pca.h"
#include "lpm.h"

/********************************************************************************************************
 *                                           宏定义
 ********************************************************************************************************/
#define CHK_BATT_EN        1
#define MCU_OUT_38K_PWM_EN 0
#define MCU_CLK_SEL        ClkFreq24Mhz 
#define MCU_UART_BAUD      115200 //4MHZ MAX 9600
#define MCU_AD_REF_V       3.3


#define debug_printf(x)  /*x*/
#define debug_printf_d(v) /*v*/
#define IDEL  (0)

#define LED_1(a) Gpio_WriteOutputIO(GpioPort0, GpioPin3, a)
/********************************************************************************************************
 *                                        类型定义
 ********************************************************************************************************/

/*!< STM32F10x Standard Peripheral Library old types (maintained for legacy purpose) */
typedef int32_t  s32;
typedef int16_t  s16;
typedef int8_t   s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;    /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t  u16;
typedef uint8_t   u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t  uc8;   /*!< Read Only */




extern uint8_t  ms10_flag;
extern uint8_t  ms100_flag;
extern uint8_t  ms250_flag;
extern uint8_t  ms300_flag;
extern uint8_t  ms500_flag;
extern uint8_t  ms1000_flag;

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H__ */
