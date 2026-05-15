#ifndef __MY_ADC_H
#define __MY_ADC_H	
#include "adc.h"

typedef enum _adc_enum
{
	CH0StREALY=0,
	CH12StREALY,
	SELECT_CH0,
	SELECT_CH12,
}adc_enum;

/******************************************************
*
*
*全局变量
*
*
******************************************************/
#define GET_ADC_ch 3
#define GET_ADC_len 2

typedef struct _t_adc
{
	u8 status;
  u8 irq_flag;
	u8 DONE;
}adc_t;


/******************************************************
*
*
*函数申明
*
*
******************************************************/
void clear_adcbuf(void);
void Get_Proc_loop(void);
void App_AdcInit(void);
void MY_ADC_INIT(void);
void get_adc_value(void);
void get_adc_value_onech(void);
uint16_t ADC_Value_Handle(volatile uint16_t *data,uint8_t times);
uint16_t adc_zhonzhi(volatile uint16_t *data,uint8_t times);
#endif

