#ifndef  __MY_GPIO_H__
#define  __MY_GPIO_H__

#include "ddl.h"
#include "gpio.h"
#include "adc.h"
#include "bgr.h"

#define KEY_ADC_THD      100      // ADC容差
#define KEY_COUNT_THD    3       // 连续多少次算有效按键
#define KEY_COUNT_MAX    100     // 防止计数溢出


void GpioInit(void);
void TouchKey_Scan(void);
#endif /* __DDL_DEVICE_H__ */


