#include "bsp_uart.h"
#include "led_driver.h"

AeroBuf_t uart1_rx_buf;
static uint8_t uart1_pool[128];

/* TM1652 用 UART2 驱屏时，不初始化蓝牙 UART2 */
#if (LED_DRIVER_TYPE != LED_DRIVER_TM1652)
AeroBuf_t uart2_rx_buf;
static uint8_t uart2_pool[256];
#endif

/**
 * UART1：下位机协议 PA01/PA00 4800
 * UART2：蓝牙 PB02/PB03 115200（非 TM1652 时）
 */
void BSP_Uart_Init(void)
{
    UART_InitTypeDef UART_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    AeroBuf_Init(&uart1_rx_buf, uart1_pool, 128);

    SYSCTRL_AHBPeriphClk_Enable(SYSCTRL_AHB_PERIPH_GPIOA, ENABLE);
    SYSCTRL_APBPeriphClk_Enable1(SYSCTRL_APB1_PERIPH_UART1, ENABLE);

    GPIO_InitStructure.Pins = GPIO_PIN_1;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(CW_GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.Pins = GPIO_PIN_0;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_Init(CW_GPIOA, &GPIO_InitStructure);
    PA01_AFx_UART1TXD();
    PA00_AFx_UART1RXD();

    UART_InitStructure.UART_Over     = UART_Over_16;
    UART_InitStructure.UART_Source   = UART_Source_PCLK;
    UART_InitStructure.UART_UclkFreq = BSP_UART_UCLK_FREQ;
    UART_InitStructure.UART_StartBit = UART_StartBit_FE;
    UART_InitStructure.UART_StopBits = UART_StopBits_1;
    UART_InitStructure.UART_Parity   = UART_Parity_No;
    UART_InitStructure.UART_Mode     = UART_Mode_Rx | UART_Mode_Tx;
    UART_InitStructure.UART_BaudRate = UART_LOWER_BAUD;
    UART_Init(CW_UART1, &UART_InitStructure);

    UART_ITConfig(CW_UART1, UART_IT_RC, ENABLE);
    NVIC_SetPriority(UART1_IRQn, 1);
    NVIC_EnableIRQ(UART1_IRQn);

#if (LED_DRIVER_TYPE != LED_DRIVER_TM1652)
    AeroBuf_Init(&uart2_rx_buf, uart2_pool, 256);
    SYSCTRL_AHBPeriphClk_Enable(SYSCTRL_AHB_PERIPH_GPIOB, ENABLE);
    SYSCTRL_APBPeriphClk_Enable1(SYSCTRL_APB1_PERIPH_UART2, ENABLE);

    GPIO_InitStructure.Pins = GPIO_PIN_2;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(CW_GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.Pins = GPIO_PIN_3;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_Init(CW_GPIOB, &GPIO_InitStructure);
    PB02_AFx_UART2TXD();
    PB03_AFx_UART2RXD();

    UART_InitStructure.UART_BaudRate = UART_BLE_BAUD;
    UART_InitStructure.UART_Parity   = UART_Parity_No;
    UART_Init(CW_UART2, &UART_InitStructure);

    UART_ITConfig(CW_UART2, UART_IT_RC, ENABLE);
    NVIC_SetPriority(UART2_IRQn, 0);
    NVIC_EnableIRQ(UART2_IRQn);
#endif
}

void BSP_Uart_WriteByte(UART_TypeDef *UARTx, uint8_t byte)
{
    uint32_t timeout = 0x8000;
    UART_SendData_8bit(UARTx, byte);
    while (UART_GetFlagStatus(UARTx, UART_FLAG_TXE) == RESET) {
        if (--timeout == 0) return;
    }
}

void UART1_IRQHandler(void)
{
    if (UART_GetITStatus(CW_UART1, UART_IT_RC) != RESET) {
        AeroBuf_Push(&uart1_rx_buf, UART_ReceiveData_8bit(CW_UART1));
        UART_ClearITPendingBit(CW_UART1, UART_IT_RC);
    }
}

#if (LED_DRIVER_TYPE != LED_DRIVER_TM1652)
void UART2_IRQHandler(void)
{
    if (UART_GetITStatus(CW_UART2, UART_IT_RC) != RESET) {
        AeroBuf_Push(&uart2_rx_buf, UART_ReceiveData_8bit(CW_UART2));
        UART_ClearITPendingBit(CW_UART2, UART_IT_RC);
    }
}
#endif
