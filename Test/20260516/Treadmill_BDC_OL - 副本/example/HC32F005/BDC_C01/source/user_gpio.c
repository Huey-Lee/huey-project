/*
 * Function: 用户按键 GPIO 与 PORT3 中断入口。
 * Method:   user_gpio_init 配输入沿触发并开 NVIC；中断内按 ADC 状态触发 tim2run 或 Get_Proc_loop。
 * Name:     Huey
 * Date:     May 16, 2026 18:00
 */

#include "user_gpio.h"
#include "gpio.h"
#include "user_adc.h"
#include "user_timer.h"

extern adc_t adc_handle;

static void App_UserKeyInit(void);

void user_gpio_init(void)
{
		App_UserKeyInit();
    Gpio_EnableIrq(STK_USER_PORT, STK_USER_PIN, GpioIrqRising);
    EnableNvic(PORT3_IRQn, IrqLevel0, TRUE);
}

static void App_UserKeyInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    stcGpioCfg.enDir = GpioDirIn;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.enOD = GpioOdDisable;

    Gpio_Init(STK_USER_PORT, STK_USER_PIN, &stcGpioCfg);
}

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
