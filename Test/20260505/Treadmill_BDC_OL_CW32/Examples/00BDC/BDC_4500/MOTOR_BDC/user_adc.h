#ifndef USER_ADC_H
#define USER_ADC_H

#include "motor.h"

typedef enum _adc_enum {
    CH0StREALY = 0,
    CH12StREALY,
    SELECT_CH0,
    SELECT_CH12,
} adc_enum;

#define GET_ADC_ch  3
#define GET_ADC_len 2

typedef struct _t_adc {
    u8 status;
    u8 irq_flag;
    u8 DONE;
} adc_t;

extern adc_t adc_handle;

extern u16 adc0data[GET_ADC_len];
extern u16 adc1data[GET_ADC_len];
extern u16 adc2data[GET_ADC_len];

void clear_adcbuf(void);
void Get_Proc_loop(void);
void user_adc_init(void);
void Adc_Start(void);
void get_adc_value(void);
uint16_t ADC_Value_Handle(volatile u16 *data, u8 times);
void bdc_user_adc_irq_handler(void);

#endif
