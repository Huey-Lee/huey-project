/*
 * AIP1628.c
 *
 *  Created on: 2018年8月1日
 *      Author: Administrator
 */
#include "aip1628.h"
#include "common.h"
#include "7_segment.h"
#include "my_gpio.h"

/**
 * 硬件接口连接定义
 */
#define TM_CS(a)   Gpio_WriteOutputIO(GpioPort1, GpioPin4, a);
#define TM_DIO(a)  Gpio_WriteOutputIO(GpioPort2, GpioPin4, a);
#define TM_CLK(a)  Gpio_WriteOutputIO(GpioPort2, GpioPin3, a);


/**
 * CS:低电平有效
 * CLK:上升沿有效，即：低电平时候需要稳定数据。
 */
static void wt_byte(u8 dat)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		TM_CLK(0);
		TM_DIO((dat &0x01));
		dat>>=1;
		TM_CLK(1);
	}
	TM_CLK(0);
}


void aip1628_write_byte(u8 addr,u8 dat)
{
	TM_CS(0);
	wt_byte(ADDR_CMD|addr);
	wt_byte(dat);
//	wt_byte((u8)(dat>>8));
	TM_CS(1);
}


void aip1628_write_nbytes(u8 addr,u8 *p, u8 len)
{
  TM_CS(0);
  wt_byte(ADDR_CMD|addr);
  for(;len>0;len--)
  {
    wt_byte(*p);
    p++;
  }
  TM_CS(1);
}


void set_aip1628_lighte_scale(u8 scale)  
{
	//打开显示
	TM_CS(0);
	wt_byte(DISP_CTR_CMD |DISP_ON |scale);
	TM_CS(1);
}


void aip1628_all_on(void)
{
	uint8_t buf[10];
	for(int i=0; i<10; i++)
		buf[i] = 0xFF;   // 每个字节全亮

	aip1628_write_nbytes(0x00, buf, 10);
}


void aip1628_all_off(void)
{
	uint8_t buf[10];
	for(int i=0; i<10; i++)
		buf[i] = 0x00;   // 每个字节全灭

	aip1628_write_nbytes(0x00, buf, 10);
}


void aip1628_init(void)
{
	//设置 显示模式：5位9段
	TM_CS(0);;
	wt_byte(DISP_MODLE_CMD |B5S9);;
	TM_CS(1);

	//数据模式
	TM_CS(0);
	wt_byte(DATA_CMD|WT_DISP_REG |ADDR_AUTO_INC);
	TM_CS(1);

	set_aip1628_lighte_scale(EXTINCTION_SCALE);
}
