#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "main.h"


// TM1620 引脚宏定义
#define TM1620_STB_PORT   CW_GPIOB
#define TM1620_STB_PIN    GPIO_PIN_1

#define TM1620_CLK_PORT   CW_GPIOA
#define TM1620_CLK_PIN    GPIO_PIN_3

#define TM1620_DIN_PORT   CW_GPIOA
#define TM1620_DIN_PIN    GPIO_PIN_4


// 433 接收引脚定义
#define RF_REC_PORT     CW_GPIOB
#define RF_REC_PIN      GPIO_PIN_0


// GPIO 控制宏
#define STB_HIGH()    GPIO_WritePin(TM1620_STB_PORT, TM1620_STB_PIN, GPIO_Pin_SET)
#define STB_LOW()     GPIO_WritePin(TM1620_STB_PORT, TM1620_STB_PIN, GPIO_Pin_RESET)

#define CLK_HIGH()    GPIO_WritePin(TM1620_CLK_PORT, TM1620_CLK_PIN, GPIO_Pin_SET)
#define CLK_LOW()     GPIO_WritePin(TM1620_CLK_PORT, TM1620_CLK_PIN, GPIO_Pin_RESET)

#define DIN_HIGH()    GPIO_WritePin(TM1620_DIN_PORT, TM1620_DIN_PIN, GPIO_Pin_SET)
#define DIN_LOW()     GPIO_WritePin(TM1620_DIN_PORT, TM1620_DIN_PIN, GPIO_Pin_RESET)


// 初始化函数
void BSP_GPIO_Init(void);

#endif
