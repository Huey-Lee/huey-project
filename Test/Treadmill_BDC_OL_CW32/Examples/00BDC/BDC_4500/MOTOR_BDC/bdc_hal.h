#ifndef BDC_HAL_H
#define BDC_HAL_H

#include <stdint.h>

void bdc_hal_init(void);
void delay10us(uint16_t n);
void delay1ms(uint32_t n);

void bdc_pwm_hw_init(void);
void bdc_pwm_enable(void);
void bdc_pwm_disable(void);
uint8_t bdc_pwm_running(void);
void bdc_pwm_set_compare(uint16_t duty_ticks);

void bdc_relay_on(void);
void bdc_relay_off(void);
uint8_t bdc_relay_on_readback(void);

void bdc_adc_arm_for_pwm_tick(void);

#endif
