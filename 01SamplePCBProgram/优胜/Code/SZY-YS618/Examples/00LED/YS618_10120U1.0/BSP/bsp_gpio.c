#include "bsp_gpio.h"


void BSP_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ��ʱ��
    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_GPIOB_CLK_ENABLE();

    // CLK (PA03) + DIN (PA04) �� ���������ģ��������
    GPIO_InitStruct.Pins = TM1620_CLK_PIN | TM1620_DIN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(CW_GPIOA, &GPIO_InitStruct);

    // STB (PB01) �� ���
    GPIO_InitStruct.Pins = TM1620_STB_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(TM1620_STB_PORT, &GPIO_InitStruct);

    // PB00���ڲ��������루��ȫ���ӵ�Ϊ��=������
    GPIO_InitStruct.Pins = SAFETY_LOCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_Init(SAFETY_LOCK_PORT, &GPIO_InitStruct);

    // Ĭ�����ߣ���������Ч����
    STB_HIGH();
    CLK_HIGH();
    DIN_HIGH();
}

uint8_t BSP_SafetyLock_Closed(void)
{
    return (GPIO_ReadPin(SAFETY_LOCK_PORT, SAFETY_LOCK_PIN) == GPIO_Pin_RESET) ? 1u : 0u;
}
