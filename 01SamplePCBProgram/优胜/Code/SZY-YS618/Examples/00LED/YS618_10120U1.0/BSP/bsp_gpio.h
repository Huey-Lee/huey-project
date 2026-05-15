#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "main.h"


// TM1620 ???????
#define TM1620_STB_PORT   CW_GPIOB
#define TM1620_STB_PIN    GPIO_PIN_1

#define TM1620_CLK_PORT   CW_GPIOA
#define TM1620_CLK_PIN    GPIO_PIN_3

#define TM1620_DIN_PORT   CW_GPIOA
#define TM1620_DIN_PIN    GPIO_PIN_4


// GPIO ?????
#define STB_HIGH()    GPIO_WritePin(TM1620_STB_PORT, TM1620_STB_PIN, GPIO_Pin_SET)
#define STB_LOW()     GPIO_WritePin(TM1620_STB_PORT, TM1620_STB_PIN, GPIO_Pin_RESET)

#define CLK_HIGH()    GPIO_WritePin(TM1620_CLK_PORT, TM1620_CLK_PIN, GPIO_Pin_SET)
#define CLK_LOW()     GPIO_WritePin(TM1620_CLK_PORT, TM1620_CLK_PIN, GPIO_Pin_RESET)

#define DIN_HIGH()    GPIO_WritePin(TM1620_DIN_PORT, TM1620_DIN_PIN, GPIO_Pin_SET)
#define DIN_LOW()     GPIO_WritePin(TM1620_DIN_PORT, TM1620_DIN_PIN, GPIO_Pin_RESET)

/** PB00����ȫ����һ�� GND��һ�� PB00������=��Ϊ�������Ͽ�=�����߱� E17�� */
#define SAFETY_LOCK_PORT   CW_GPIOB
#define SAFETY_LOCK_PIN    GPIO_PIN_0

void BSP_GPIO_Init(void);
/** @return �� 0������/�д�����0���Ͽ����뱨 E17 */
uint8_t BSP_SafetyLock_Closed(void);

#endif
