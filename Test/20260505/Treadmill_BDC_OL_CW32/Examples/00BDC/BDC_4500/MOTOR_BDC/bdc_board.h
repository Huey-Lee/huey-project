/**
 * bdc_board.h — 下位机引脚与模拟通道映射（CW32L010）
 *
 * 硬件与 HA380A HC32 母控一致时，请按 PCB netlist 把下列宏改成实际接到
 * 电流 / M− / M+ / MOS 栅极 / 继电器 / UART 的引脚。
 *
 * 默认分配思路：UART2 走 PB02/PB03，UART1 不接；PWM 用 ATIM CH1；
 * ADC 用 PA04/05/06 三路与文档“PA0x=CHx”对齐，仅作起点，务必实测修正。
 */
#ifndef BDC_BOARD_H
#define BDC_BOARD_H

#include "cw32l010.h"
#include "cw32l010_gpio.h"
#include "cw32l010_sysctrl.h"

#define BDC_UART CW_UART2
#define BDC_UART_BAUD                (2400u)
#define BDC_UART_APB_PERIPH          SYSCTRL_APB1_PERIPH_UART2
#define BDC_UART_TX_AF()             PB02_AFx_UART2TXD()
#define BDC_UART_RX_AF()             PB03_AFx_UART2RXD()
#define BDC_UART_TX_PORT             CW_GPIOB
#define BDC_UART_TX_PIN              GPIO_PIN_2
#define BDC_UART_RX_PORT             CW_GPIOB
#define BDC_UART_RX_PIN              GPIO_PIN_3
#define BDC_UART_IRQn                UART2_IRQn

/* LED 心跳（可选；未接可改为空操作） */
#define BDC_LED_PORT                 CW_GPIOA
#define BDC_LED_PIN                  GPIO_PIN_3
#define BDC_LED_ON()                 GPIO_WritePin(BDC_LED_PORT, BDC_LED_PIN, GPIO_Pin_SET)
#define BDC_LED_OFF()                GPIO_WritePin(BDC_LED_PORT, BDC_LED_PIN, GPIO_Pin_RESET)

/* 继电器：参考 tim0run/stop 为持续吸合/释放（若原板用 TOG 激励需改驱动） */
#define BDC_RELAY_PORT               CW_GPIOA
#define BDC_RELAY_PIN                GPIO_PIN_7
#define BDC_RELAY_ON()               GPIO_WritePin(BDC_RELAY_PORT, BDC_RELAY_PIN, GPIO_Pin_SET)
#define BDC_RELAY_OFF()              GPIO_WritePin(BDC_RELAY_PORT, BDC_RELAY_PIN, GPIO_Pin_RESET)

/* ATIM CH1 — 默认 PB00（AF7）；须与功率板 MOS 驱动相连 */
#define BDC_PWM_ATIM_CH1_AF()        PB00_AFx_ATIMCH1()

/* ADC：库中通道与引脚对应见 cw32l010_adc.h；以下为示例 net 映射 */
#define BDC_ADC_CH_I                 ADC_InputCH4   /* PA04 电流 */
#define BDC_ADC_CH_MMINUS            ADC_InputCH5   /* PA05 M− */
#define BDC_ADC_CH_MPLUS             ADC_InputCH6   /* PA06 M+ */

#endif
