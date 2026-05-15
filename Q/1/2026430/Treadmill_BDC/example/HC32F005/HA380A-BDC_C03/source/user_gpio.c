/* user_gpio.c - e-stop, keys, GPIO in, IRQ entry. */
#include "user_gpio.h"
#include "gpio.h"
#include "user_adc.h"
#include "user_timer.h"

extern adc_t adc_handle;
//static void App_LedInit(void);
static void App_UserKeyInit(void);

void user_gpio_init(void)
{
    App_UserKeyInit();

//	App_LedInit();
    /* User key: rising edge IRQ */
    Gpio_EnableIrq(STK_USER_PORT, STK_USER_PIN, GpioIrqRising);
    /* NVIC: PORT3 */
    EnableNvic(PORT3_IRQn, IrqLevel0, TRUE);
}

static void App_UserKeyInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    /* GPIO clock on */
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    /* input, low drive, no pull, CMOS */
    stcGpioCfg.enDir = GpioDirIn;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.enOD = GpioOdDisable;

    Gpio_Init(STK_USER_PORT, STK_USER_PIN, &stcGpioCfg); 
}

/* PORT3 IRQ: user key */
void Port3_IRQHandler(void)
{
    if(TRUE == Gpio_GetIrqStatus(STK_USER_PORT, STK_USER_PIN))
    {
        Gpio_ClearIrq(STK_USER_PORT, STK_USER_PIN);
				if(adc_handle.status == SELECT_CH0)
				{
					if(M0P_ADC->IFR_f.CONT_INTF == 0)
					tim2run();
				}
				Get_Proc_loop();
    }
}
