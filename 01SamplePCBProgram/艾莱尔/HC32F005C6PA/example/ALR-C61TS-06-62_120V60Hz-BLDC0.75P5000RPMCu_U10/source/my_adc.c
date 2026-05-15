#include "my_adc.h"
#include "gpio.h"

volatile uint32_t u32AdcResultAcc,u16AdcResult;


void App_AdcInit(void)
{
    stc_adc_cfg_t             stcAdcCfg;
    stc_adc_cont_cfg_t        stcAdcContCfg;
    stc_adc_irq_t             stcAdcIrq;    
    
    DDL_ZERO_STRUCT(stcAdcCfg);
    DDL_ZERO_STRUCT(stcAdcContCfg);
    DDL_ZERO_STRUCT(stcAdcIrq);
     
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);  //ADCBGR 外设时钟使能
    
    //ADC配置
    Adc_Enable();
    Bgr_BgrEnable();
    
    stcAdcCfg.enAdcOpMode = AdcContMode;                //连续采样模式
    stcAdcCfg.enAdcClkSel = AdcClkSysTDiv1;             //PCLK
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime8Clk;       //8个采样时钟
    stcAdcCfg.enAdcRefVolSel = RefVolSelAVDD;           //参考电压:AVDD
    stcAdcCfg.bAdcInBufEn = FALSE;                      //电压跟随器如果使能，SPS采样速率 <=200K
    stcAdcCfg.enAdcTrig0Sel = AdcTrigDisable;           //ADC转换自动触发设置
    stcAdcCfg.enAdcTrig1Sel = AdcTrigDisable;
    Adc_Init(&stcAdcCfg);    
    
    stcAdcIrq.bAdcIrq = TRUE;                            //转换完成中断函数入口配置使能
    stcAdcIrq.bAdcRegCmp = FALSE;
    stcAdcIrq.bAdcHhtCmp = FALSE;
    stcAdcIrq.bAdcLltCmp = FALSE;
    Adc_CmpCfg(&stcAdcIrq);                              //结果比较中断使能/禁止配置
		 ///< 通道0 P26
    stcAdcContCfg.enAdcContModeCh = AdcExInputCH1;      //通道0 P26
    stcAdcContCfg.u8AdcSampCnt    = 199u;              	//P24 连续累加次数(次数 = 99+1)
    stcAdcContCfg.bAdcResultAccEn = TRUE;               //累加使能
    Adc_ConfigContMode(&stcAdcCfg, &stcAdcContCfg);

    Adc_EnableIrq();                                    //中断使能
    EnableNvic(ADC_IRQn, IrqLevel3, TRUE);


    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);    //GPIO 外设时钟使能

    Gpio_SetAnalogMode(GpioPort2, GpioPin6);
	
	Adc_Start();        //ADC开始转换
}



///< ADC 中断服务程序
void Adc_IRQHandler(void)
{
    if (TRUE == M0P_ADC->IFR_f.REG_INTF)
    {
        Adc_ClrRegIrqState();
    }

    if (TRUE == M0P_ADC->IFR_f.HHT_INTF)
    {
        Adc_ClrHhtIrqState();
    }

    if (TRUE == M0P_ADC->IFR_f.LLT_INTF)
    {
        Adc_ClrIrqLltState();
    }

    if (TRUE == M0P_ADC->IFR_f.CONT_INTF)
    {
        Adc_ClrContIrqState();
        
        Adc_GetAccResult((uint32_t *)&u32AdcResultAcc);
		u16AdcResult=u32AdcResultAcc*0.005;
        Adc_ClrAccResult();
		Adc_Start();
    }
}




uint16_t get_adc_value(void)
{
	return u16AdcResult;
}





