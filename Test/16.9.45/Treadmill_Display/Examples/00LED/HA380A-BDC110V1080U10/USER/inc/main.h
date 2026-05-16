/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : main.c 对应头文件。
  *                   本文件包含应用层公共定义。
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 CW.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* 防止头文件重复包含 -------------------------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* 头文件包含 ----------------------------------------------------------------*/
#include "base_types.h"
#include "cw32l010.h"
#include "system_cw32l010.h"
#include "interrupts_cw32l010.h"
#include "cw32l010_btim.h"
#include "cw32l010_sysctrl.h"
#include "cw32l010_gpio.h"
#include "cw32l010_uart.h"
#include "cw32l010_gtim.h"	
/* 私有头文件 ----------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

#include <stdbool.h>
/* 导出类型 ----------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */


/* 导出常量 ----------------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */


/* 导出宏 ------------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */


/* 导出函数原型 --------------------------------------------------------------*/
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */


/* 私有宏定义 ----------------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT CW *****END OF FILE****/
