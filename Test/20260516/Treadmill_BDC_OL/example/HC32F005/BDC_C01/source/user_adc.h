/*
 * Function: 多通道 ADC 句柄、通道选择与对外 API 声明。
 * Method:   adc_t 描述采样状态；GET_ADC_* 定义通道数与缓冲深度；声明初始化与各轮询入口。
 * Name:     Huey
 * Date:     May 16, 2026 18:00
 */

#ifndef USER_ADC_H
#define USER_ADC_H
#include "adc.h"

typedef enum _adc_enum
{
	CH0StREALY=0,
	CH12StREALY,
	SELECT_CH0,
	SELECT_CH12,
}adc_enum;

#define GET_ADC_ch 3
#define GET_ADC_len 2

typedef struct _t_adc
{
	u8 status;
  u8 irq_flag;
	u8 DONE;
}adc_t;

extern volatile uint32_t adc_batch_seq;

void clear_adcbuf(void);
void Get_Proc_loop(void);
void App_AdcInit(void);
void user_adc_init(void);
void get_adc_value(void);
void get_adc_value_onech(void);
uint16_t ADC_Value_Handle(volatile uint16_t *data,uint8_t times);
uint16_t adc_zhonzhi(volatile uint16_t *data,uint8_t times);
#endif
