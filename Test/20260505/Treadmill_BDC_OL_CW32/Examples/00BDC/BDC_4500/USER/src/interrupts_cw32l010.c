/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    interrupts_cw32l010.c
  * @brief   中断服务程序。
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

/* 头文件包含 ----------------------------------------------------------------*/
#include "..\inc\main.h"
#include "interrupts_cw32l010.h"

/* 私有头文件 ----------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */


/* 私有类型定义 --------------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */


/* 私有宏定义 ----------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */


/* 私有宏 --------------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */


/* 私有变量 ------------------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */


/* 私有函数声明 --------------------------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */


/* 私有用户代码 --------------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* 外部变量 ------------------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */


/******************************************************************************/
/*           Cortex-M0P 内核异常与不可屏蔽中断处理                            */
/******************************************************************************/
/**
  * @brief 不可屏蔽中断 NMI。
  */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn */

    /* USER CODE END NonMaskableInt_IRQn */
}

/**
  * @brief 硬件错误 HardFault。
  */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn */

    /* USER CODE END HardFault_IRQn */
    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn */

        /* USER CODE END W1_HardFault_IRQn */
    }
}


/**
  * @brief 系统服务调用 SVC（SWI）。
  */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVCall_IRQn */

    /* USER CODE END SVCall_IRQn */
}


/**
  * @brief 可挂起系统调用 PendSV。
  */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn */

    /* USER CODE END PendSV_IRQn */
}


/******************************************************************************/
/* CW030 外设中断处理                                                         */
/* 在此添加所使用外设的中断服务函数。                                         */
/* 外设中断服务函数名称请参考                                                 */
/* 启动文件 startup_ps030.s。                                                 */
/******************************************************************************/

/**
 * @brief WDT 中断服务
 */
void WDT_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief LVD 中断服务
 */
void LVD_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief RTC 中断服务
 */
void RTC_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief FLASHRAM 中断服务
 */
void FLASHRAM_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief SYSCTRL 中断服务
 */
void SYSCTRL_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief GPIOA 中断服务
 */
void GPIOA_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief GPIOB 中断服务
 */
void GPIOB_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief ADC 中断服务
 */
void ADC_IRQHandler(void)
{
    extern void bdc_user_adc_irq_handler(void);
    bdc_user_adc_irq_handler();
}

/**
 * @brief ATIM 中断服务
 */
void ATIM_IRQHandler(void)
{
    extern void bdc_atim_pwm_isr(void);
    bdc_atim_pwm_isr();
}

/**
 * @brief VC1 中断服务
 */
void VC1_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief VC2 中断服务
 */
void VC2_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief GTIM1 中断服务
 */
void GTIM1_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief LPTIM 中断服务
 */
void LPTIM_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

///**
// * @brief BTIM1 中断服务
// */
//void BTIM1_IRQHandler(void)
//{
//    /* USER CODE BEGIN */
//    if (BTIM_GetITStatus(CW_BTIM1, BTIM_IT_UPDATE))
//    {
//        BTIM_ClearITPendingBit(CW_BTIM1, BTIM_IT_UPDATE);
//        PB02_TOG();
//    }

//    /* USER CODE END */
//}

///**
// * @brief BTIM2 中断服务
// */
//void BTIM2_IRQHandler(void)
//{
//    /* USER CODE BEGIN */

//    /* USER CODE END */
//}

/**
 * @brief BTIM3 中断服务
 */
void BTIM3_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief I2C1 中断服务
 */
void I2C1_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief SPI1 中断服务
 */
void SPI1_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

///**
// * @brief UART1 中断服务
// */
//void UART1_IRQHandler(void)
//{
//    /* USER CODE BEGIN */

//    /* USER CODE END */
//}

void UART2_IRQHandler(void)
{
    extern void bdc_uart_rx_irq_handler(void);
    bdc_uart_rx_irq_handler();
}

/**
 * @brief FAULT 中断服务
 */
void CLKFAULT_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}



/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT CW *****END OF FILE****/
