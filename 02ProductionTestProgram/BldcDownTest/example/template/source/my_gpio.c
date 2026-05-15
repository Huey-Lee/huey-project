#include "my_gpio.h"

 
void GpioInit(void)
{
  stc_gpio_cfg_t stcGpioCfg;
    
  //< 打开GPIO外设时钟门控
  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 

  stcGpioCfg.enDir = GpioDirOut;

  stcGpioCfg.enPu = GpioPuDisable;
  stcGpioCfg.enPd = GpioPdDisable;
	stcGpioCfg.enDrv = GpioDrvH;
	stcGpioCfg.enOD = GpioOdEnable;//打开开漏
	

	Gpio_Init(GpioPort2, GpioPin4, &stcGpioCfg);
	Gpio_Init(GpioPort2, GpioPin3, &stcGpioCfg);
	Gpio_Init(GpioPort1, GpioPin4, &stcGpioCfg);
	Gpio_Init(GpioPort0, GpioPin3, &stcGpioCfg);
	Gpio_WriteOutputIO(GpioPort2, GpioPin4, 1);
	Gpio_WriteOutputIO(GpioPort2, GpioPin3, 1);
	Gpio_WriteOutputIO(GpioPort1, GpioPin4, 1);
	Gpio_WriteOutputIO(GpioPort0, GpioPin3, 1);
	stcGpioCfg.enDir = GpioDirIn;
  Gpio_Init(GpioPort3, GpioPin4, &stcGpioCfg);
}


