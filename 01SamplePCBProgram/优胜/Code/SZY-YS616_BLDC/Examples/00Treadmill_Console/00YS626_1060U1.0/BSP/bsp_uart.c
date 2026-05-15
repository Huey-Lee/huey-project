#include "bsp_uart.h"


// UART1
AeroBuf_t uart1_rx_buf;
static uint8_t uart1_pool[128];

// UART2
AeroBuf_t uart2_rx_buf;
static uint8_t uart2_pool[256];

/**
 * @brief 串口硬件初始化
 */
void BSP_Uart_Init(void)
{
    UART_InitTypeDef UART_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};
	AeroBuf_Init(&uart1_rx_buf, uart1_pool, 128);
    AeroBuf_Init(&uart2_rx_buf, uart2_pool, 256);
	
    /* 时钟使能 */
    SYSCTRL_AHBPeriphClk_Enable(SYSCTRL_AHB_PERIPH_GPIOA | SYSCTRL_AHB_PERIPH_GPIOB, ENABLE);
    SYSCTRL_APBPeriphClk_Enable1(SYSCTRL_APB1_PERIPH_UART1 | SYSCTRL_APB1_PERIPH_UART2, ENABLE);

    /* GPIO 引脚配置 */
    // UART1 (下控): PA01(TX), PA00(RX)
    GPIO_InitStructure.Pins = GPIO_PIN_1;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(CW_GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.Pins = GPIO_PIN_0;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_Init(CW_GPIOA, &GPIO_InitStructure);
    PA01_AFx_UART1TXD(); 
		PA00_AFx_UART1RXD(); // 复用映射

    // UART2 (蓝牙): PB02(TX), PB03(RX)
    GPIO_InitStructure.Pins = GPIO_PIN_2;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(CW_GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.Pins = GPIO_PIN_3;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_Init(CW_GPIOB, &GPIO_InitStructure);
    PB02_AFx_UART2TXD(); 
	PB03_AFx_UART2RXD(); // 复用映射

    /* UART 模块参数配置 */
    UART_InitStructure.UART_Over = UART_Over_16;
    UART_InitStructure.UART_Source = UART_Source_PCLK;
    UART_InitStructure.UART_UclkFreq = BSP_UART_UCLK_FREQ;
    UART_InitStructure.UART_StartBit = UART_StartBit_FE;
    UART_InitStructure.UART_StopBits = UART_StopBits_1;
    UART_InitStructure.UART_Parity = UART_Parity_No;
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;

    // 初始化 UART1 (4800)
    UART_InitStructure.UART_BaudRate = UART_LOWER_BAUD;
    UART_Init(CW_UART1, &UART_InitStructure);

    // 初始化 UART2 (115200)
    UART_InitStructure.UART_BaudRate = UART_BLE_BAUD;
    UART_Init(CW_UART2, &UART_InitStructure);

    /* 中断与 NVIC 配置 */
    
    // 使能接收中断
    UART_ITConfig(CW_UART1, UART_IT_RC, ENABLE);
    UART_ITConfig(CW_UART2, UART_IT_RC, ENABLE);

    // 设置 NVIC 优先级 (蓝牙波特率高，设为最高级 0，下控设为 1)
    NVIC_SetPriority(UART2_IRQn, 0); 
    NVIC_SetPriority(UART1_IRQn, 1); 
    
    NVIC_EnableIRQ(UART1_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);
}

/**
 * @brief 统一发送字节函数 (带安全超时保护)
 */
void BSP_Uart_WriteByte(UART_TypeDef *UARTx, uint8_t byte)
{
    uint32_t timeout = 0x8000; // 约 3-5ms 的安全窗口 (基于48MHz)

    UART_SendData_8bit(UARTx, byte);

    // 等待发送缓冲区空
    while (UART_GetFlagStatus(UARTx, UART_FLAG_TXE) == RESET)
    {
        if (--timeout == 0) 
        {
            // 可以在此处加入简单的错误计数或硬件复位尝试
            return; 
        }
    }
}

/**
 * @brief UART1 中断服务 (下控)
 */
void UART1_IRQHandler(void)
{
    if(UART_GetITStatus(CW_UART1, UART_IT_RC) != RESET)
    {
			 uint8_t res = UART_ReceiveData_8bit(CW_UART1);
   
//        BSP_Uart_WriteByte(CW_UART1, res); // 回显功能
			
        // 从寄存器读出数据，直接压入缓冲区
        AeroBuf_Push(&uart1_rx_buf, UART_ReceiveData_8bit(CW_UART1));
        UART_ClearITPendingBit(CW_UART1, UART_IT_RC);
    }
}

/**
 * @brief UART2 中断服务 (蓝牙)
 */
void UART2_IRQHandler(void)
{
    if(UART_GetITStatus(CW_UART2, UART_IT_RC) != RESET)
    {
        AeroBuf_Push(&uart2_rx_buf, UART_ReceiveData_8bit(CW_UART2));
        UART_ClearITPendingBit(CW_UART2, UART_IT_RC);
    }
}
