#ifndef  __MY_PWM_H__
#define  __MY_PWM_H__


#include "pca.h"
#include "lpm.h"
#include "gpio.h"

#define CBValue 1500 //PWM比较基准值

//16khz pwm*2
//24mhz / 1500 = 16khz

//0->duty <1500 = 1  duty ->1500 = 0
//0,1,2,3,4,5,6,7,8.....duty  TIMX->CNT  duty
//0 -> 1500 高电平最后x us 去采样
//16khz 1s计数16000次 1us 走0.16次 CNT+0.16 → 1次6.25us
//20khz   1us 0.1次  5us 1个计数
//0-> duty -1 刚好差6.25us 电平转换，高电平的最后6.25us
//电机控制 5-15khz 
//+1， CNT +1 走了 6.25us

void App_PcaInit(void);
void set_pwm_pca(uint8_t sw,uint16_t value);
void set_pwm_beep(uint16_t value);
#endif /* __DDL_DEVICE_H__ */


