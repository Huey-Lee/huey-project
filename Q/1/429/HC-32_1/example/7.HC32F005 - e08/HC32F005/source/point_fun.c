#include "point_fun.h"
#include "gpio.h"
#include "MY_ADC.h"
#include "motor.h"
c_stack gfun;
extern adc_t adc_handle;

int myfun(int data)
{

	MY_ADC_INIT();
	ctr_init();
	return (data*2);
}

int rt_data(int data,int (*tr_fun)())
{
	return ((*tr_fun)(data));
}  



void Select_Init(void)
{
	gfun.pfun = myfun;
}






int MAIN_INIT(int argc)
{
	int ret;
	Select_Init();
	ret = rt_data(100,gfun.pfun);

	return ret;
}
