#include "bdc_comm.h"
#include "bdc_board.h"
#include "uart_frame.h"
#include "queue.h"

#include "cw32l010_uart.h"
#include "cw32l010_gpio.h"
#include "cw32l010_sysctrl.h"

void user_usart_init(void)
{
    GPIO_InitTypeDef g = {0};
    UART_InitTypeDef u = {0};

    uart_frame_init();

    SYSCTRL_AHBPeriphClk_Enable(SYSCTRL_AHB_PERIPH_GPIOB, ENABLE);
    SYSCTRL_APBPeriphClk_Enable1(SYSCTRL_APB1_PERIPH_UART2, ENABLE);

    g.Pins = BDC_UART_TX_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.IT   = GPIO_IT_NONE;
    GPIO_Init(BDC_UART_TX_PORT, &g);
    g.Pins = BDC_UART_RX_PIN;
    g.Mode = GPIO_MODE_INPUT_PULLUP;
    g.IT   = GPIO_IT_NONE;
    GPIO_Init(BDC_UART_RX_PORT, &g);

    BDC_UART_TX_AF();
    BDC_UART_RX_AF();

    u.UART_Over      = UART_Over_16;
    u.UART_Source    = UART_Source_PCLK;
    u.UART_UclkFreq = 48000000u;
    u.UART_StartBit  = UART_StartBit_FE;
    u.UART_StopBits  = UART_StopBits_1;
    u.UART_Parity    = UART_Parity_No;
    u.UART_Mode      = UART_Mode_Rx | UART_Mode_Tx;
    u.UART_BaudRate  = BDC_UART_BAUD;
    UART_Init(BDC_UART, &u);

    UART_ITConfig(BDC_UART, UART_IT_RC, ENABLE);
    NVIC_SetPriority(BDC_UART_IRQn, 3);
    NVIC_EnableIRQ(BDC_UART_IRQn);
}

void BDC_Uart_SendBytes(uint8_t *p, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        uint32_t t = 0x8000u;
        while (UART_GetFlagStatus(BDC_UART, UART_FLAG_TXE) == RESET && --t) {
        }
        UART_SendData_8bit(BDC_UART, p[i]);
    }
}

void bdc_uart_rx_irq_handler(void)
{
    if (UART_GetITStatus(BDC_UART, UART_IT_RC) != RESET) {
        uint8_t b = UART_ReceiveData_8bit(BDC_UART);
        Enter_queue(&rx_queue, b);
        UART_ClearITPendingBit(BDC_UART, UART_IT_RC);
    }
}
