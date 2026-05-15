#include "MY_GPIO.h"
#include "gpio.h"
#include "MY_ADC.h"
#include "MY_TIM.h"

extern adc_t adc_handle;
//static void App_LedInit(void);
static void App_UserKeyInit(void);

void MY_GPIO_INIT(void)
{
		App_UserKeyInit();

//	App_LedInit();
    ///< 打开并配置USER KEY为上升沿中断
    Gpio_EnableIrq(STK_USER_PORT, STK_USER_PIN, GpioIrqRising);
    ///< 使能端口PORT3系统中断
    EnableNvic(PORT3_IRQn, IrqLevel0, TRUE);
}

static void App_UserKeyInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    ///< 端口方向配置->输入
    stcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->高驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->无
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;

    ///< GPIO IO USER KEY初始化
    Gpio_Init(STK_USER_PORT, STK_USER_PIN, &stcGpioCfg); 
}

//< Port3中断服务函数
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
