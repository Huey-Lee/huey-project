/*
 * bsp_uart.c  -  UART driver for CW32L010 display board
 *
 * UART1: motor controller  PA01(TX)/PA00(RX)  baud = TM_UART_BAUD
 * UART2: BLE module        PB02(TX)/PB03(RX)  baud = UART_BLE_BAUD
 *        (UART2 not initialized when TM1652 drives LED, as it shares pins)
 */
#include "bsp_uart.h"
#include "led_driver.h"

AeroBuf_t uart1_rx_buf;
static uint8_t uart1_pool[128];

#if (LED_DRIVER_TYPE != LED_DRIVER_TM1652)
AeroBuf_t uart2_rx_buf;
static uint8_t uart2_pool[256];
#endif

void BSP_Uart_Init(void)
{
    UART_InitTypeDef uart_cfg = {0};
    GPIO_InitTypeDef gpio_cfg = {0};

    AeroBuf_Init(&uart1_rx_buf, uart1_pool, sizeof(uart1_pool));

    SYSCTRL_AHBPeriphClk_Enable(SYSCTRL_AHB_PERIPH_GPIOA, ENABLE);
    SYSCTRL_APBPeriphClk_Enable1(SYSCTRL_APB1_PERIPH_UART1, ENABLE);

    gpio_cfg.Pins = GPIO_PIN_1;
    gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(CW_GPIOA, &gpio_cfg);
    gpio_cfg.Pins = GPIO_PIN_0;
    gpio_cfg.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_Init(CW_GPIOA, &gpio_cfg);
    PA01_AFx_UART1TXD();
    PA00_AFx_UART1RXD();

    uart_cfg.UART_Over     = UART_Over_16;
    uart_cfg.UART_Source   = UART_Source_PCLK;
    uart_cfg.UART_UclkFreq = BSP_UART_UCLK_FREQ;
    uart_cfg.UART_StartBit = UART_StartBit_FE;
    uart_cfg.UART_StopBits = UART_StopBits_1;
    uart_cfg.UART_Parity   = UART_Parity_No;
    uart_cfg.UART_Mode     = UART_Mode_Rx | UART_Mode_Tx;
    uart_cfg.UART_BaudRate = UART_LOWER_BAUD;
    UART_Init(CW_UART1, &uart_cfg);

    UART_ITConfig(CW_UART1, UART_IT_RC, ENABLE);
    NVIC_SetPriority(UART1_IRQn, 1);
    NVIC_EnableIRQ(UART1_IRQn);

#if (LED_DRIVER_TYPE != LED_DRIVER_TM1652)
    AeroBuf_Init(&uart2_rx_buf, uart2_pool, sizeof(uart2_pool));
    SYSCTRL_AHBPeriphClk_Enable(SYSCTRL_AHB_PERIPH_GPIOB, ENABLE);
    SYSCTRL_APBPeriphClk_Enable1(SYSCTRL_APB1_PERIPH_UART2, ENABLE);

    gpio_cfg.Pins = GPIO_PIN_2;
    gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(CW_GPIOB, &gpio_cfg);
    gpio_cfg.Pins = GPIO_PIN_3;
    gpio_cfg.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_Init(CW_GPIOB, &gpio_cfg);
    PB02_AFx_UART2TXD();
    PB03_AFx_UART2RXD();

    uart_cfg.UART_BaudRate = UART_BLE_BAUD;
    uart_cfg.UART_Parity   = UART_Parity_No;
    UART_Init(CW_UART2, &uart_cfg);

    UART_ITConfig(CW_UART2, UART_IT_RC, ENABLE);
    NVIC_SetPriority(UART2_IRQn, 0);
    NVIC_EnableIRQ(UART2_IRQn);
#endif
}

/* Blocking byte write; returns on TDR-empty or timeout. */
void BSP_Uart_WriteByte(UART_TypeDef *UARTx, uint8_t byte)
{
    uint32_t timeout = 0x8000u;
    while (UART_GetFlagStatus(UARTx, UART_FLAG_TXE) == RESET) {
        if (--timeout == 0u) {
            return;
        }
    }
    UART_SendData_8bit(UARTx, byte);
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
