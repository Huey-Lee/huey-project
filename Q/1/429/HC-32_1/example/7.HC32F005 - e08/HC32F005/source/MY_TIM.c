#include "MY_TIM.h"
#include "adc.h"
#include "gpio.h"
#include "MY_ADC.h"

extern adc_t adc_handle;
volatile uint16_t ZHANKONBI = MT_START_PWM;//10 ~ 790
#define OUT_PUT (545 - ZHANKONBI)

tim_t tim_handle = {0};
uint8_t u8TestFlag;
/*******************************************************************************
 * BT定时功能测试 （重载模式）
 ******************************************************************************/
static en_result_t App_BtTimerTest(void)
{
    stc_bt_cfg_t   		stcCfg;
    en_result_t       enResult = Error;
//    uint16_t          u16ArrData = 0XC568;			//65536 - 30000      30000 * 8 /24M = 10mS
//    uint16_t          u16InitCntData = 0XC568;
		
    //打开BT外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE);
		
    stcCfg.enGateP = BtPositive;
    stcCfg.enGate  = BtGateDisable;
    stcCfg.enPRS   = BtPCLKDiv8;			//256分频
    stcCfg.enTog   = BtTogDisable;
    stcCfg.enCT    = BtTimer;
    stcCfg.enMD    = BtMode2;
    //Bt初始化
    if (Ok != Bt_Init(TIM2, &stcCfg))
    {
        enResult = Error;
    }
		
    //TIM1中断使能
    Bt_ClearIntFlag(TIM2);
    Bt_EnableIrq(TIM2);
    EnableNvic(TIM2_IRQn, IrqLevel0, TRUE);
		
    //设置重载值和计数值，启动计数
		
    return enResult;
}
/*******************************************************************************
 * BT Buzzer功能测试 （TOG）
 ******************************************************************************/
static en_result_t App_BtTogTest(void)
{
    stc_bt_cfg_t   stcCfg;
    en_result_t       enResult = Ok;
    ///<（4Mhz->1000Hz）
    uint16_t          u16ArrData = 0xF6A0;      //65536-2400    2400 * 1 /24M = 100us
    uint16_t          u16InitCntData = 0xF6A0;
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE);
    
    stcCfg.enGateP = BtPositive;
    stcCfg.enGate  = BtGateDisable;
    stcCfg.enPRS   = BtPCLKDiv1;
    stcCfg.enTog   = BtTogEnable;
    stcCfg.enCT    = BtTimer;
    stcCfg.enMD    = BtMode2;
    
    if (Ok != Bt_Init(TIM0, &stcCfg))
    {
        enResult = Error;
    }
    
    //设置重载值，计数值，启动计数
		Bt_ARRSet(TIM0, u16ArrData);
		Bt_Cnt16Set(TIM0, u16InitCntData);
    
    return enResult;
}
///< AdvTimer初始化
void App_AdvTimerInit(uint16_t u16Period, uint16_t u16CHA_PWMDuty, uint16_t u16CHB_PWMDuty)
{
    en_adt_compare_t          enAdtCompare;
    stc_adt_basecnt_cfg_t     stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIM6ACfg;
    
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIM6ACfg);
		
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);    //ADT外设时钟使能
    
    
    stcAdtBaseCntCfg.enCntMode = AdtTriangleModeA;               //三角波模式
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0;
    Adt_Init(M0P_ADTIM6, &stcAdtBaseCntCfg);                      //ADT载波、计数模式、时钟配置
    
    Adt_SetPeriod(M0P_ADTIM6, u16Period);                         //周期设置
    
    enAdtCompare = AdtCompareA;
    Adt_SetCompareValue(M0P_ADTIM6, enAdtCompare, u16CHA_PWMDuty);  //通用比较基准值寄存器A设置
    
    enAdtCompare = AdtCompareC;
    Adt_SetCompareValue(M0P_ADTIM6, enAdtCompare, u16CHA_PWMDuty);  //通用比较基准值寄存器C设置
    
    Adt_EnableValueBuf(M0P_ADTIM6, AdtCHxA, TRUE);              //CHA buffer 使能
		
    
    stcAdtTIM6ACfg.enCap = AdtCHxCompareOutput;            //比较输出
    stcAdtTIM6ACfg.bOutEn = TRUE;                          //CHA输出使能
    stcAdtTIM6ACfg.enPerc = AdtCHxPeriodKeep;              //计数值与周期匹配时CHA电平保持不变
    stcAdtTIM6ACfg.enCmpc = AdtCHxCompareInv;              //计数值与比较值A匹配时，CHA电平翻转
    stcAdtTIM6ACfg.enStaStp = AdtCHxStateSelSS;            //CHA起始结束电平由STACA与STPCA控制
    stcAdtTIM6ACfg.enStaOut = AdtCHxPortOutHigh;           //CHA起始电平为低
    stcAdtTIM6ACfg.enStpOut = AdtCHxPortOutHigh;           //CHA结束电平为低
    Adt_CHxXPortCfg(M0P_ADTIM6, AdtCHxA, &stcAdtTIM6ACfg); //端口CHA配置

    Adt_ClearAllIrqFlag(M0P_ADTIM6);
    Adt_CfgIrq(M0P_ADTIM6, AdtUDFIrq, TRUE);  						 //下溢中断配置
    EnableNvic(ADTIM6_IRQn, IrqLevel1, TRUE);
}
static void App_GpioInit(void)
{
    stc_gpio_cfg_t         stcTIM0Port;
	
    DDL_ZERO_STRUCT(stcTIM0Port);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);//端口外设时钟使能
    stcTIM0Port.enDir  = GpioDirOut;
    //P14设置为TIM0_TOGN
    Gpio_Init(GpioPort0, GpioPin1, &stcTIM0Port);
    Gpio_SetAfMode(GpioPort0,GpioPin1,GpioAf4);
}
///< AdvTimer端口初始化
void App_AdvTimerPortInit(void)
{
    stc_gpio_cfg_t         stcTIM6Port;
    
    DDL_ZERO_STRUCT(stcTIM6Port);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //端口外设时钟使能
    
    stcTIM6Port.enDir  = GpioDirOut;
    //P34设置为TIM4_CHA
    Gpio_Init(GpioPort0, GpioPin2, &stcTIM6Port);
    Gpio_SetAfMode(GpioPort0,GpioPin2,GpioAf5);
}
/*******************************************************************************
** 函数名称: void MY_TIM_INIT(void)
** 功能描述: TIM初始化
** 参数说明: None
** 返回说明: None
** 创建人员: 张涛
** 创建日期: 2019/6/24
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
void MY_TIM_INIT(void)
{
	if(Ok != App_BtTimerTest())
	{
			u8TestFlag |= 0x02;
	}
	App_GpioInit();
	
  App_AdvTimerPortInit();     //AdvTimer端口初始化
}
void tim0run(void)
{
	if(Ok != App_BtTogTest())
	{
		u8TestFlag |= 0x04;
	}
	Bt_Run(TIM0);
}
void tim0stop(void)
{
	Bt_Stop(TIM0);
}
void tim2run(void)
{
	Bt_Cnt16Set(TIM2, tim2_cnt);
	Bt_Run(TIM2);
}

void tim6run(void)
{
    App_AdvTimerInit(545, 544, 1);  //AdvTimer4初始化
		App_AdvTimerInit(545, 544, 1);  //AdvTimer4初始化
		Adt_ClearCount(M0P_ADTIM6);			//清除计数值
    //配置为三角波模式: 初始化周期0xC000, CHA占空比设置0x4000，CHB占空比设置0x8000
    Adt_StartCount(M0P_ADTIM6);
}

void tim6stop(void)
{
	Adt_ClearCount(M0P_ADTIM6);									//清除计数值
	Adt_StopCount(M0P_ADTIM6);
}

/*******************************************************************************
** 函数名称: void Tim2_IRQHandler(void)
** 功能描述: 定时器中断回调函数
** 参数说明: None
** 返回说明: None
** 创建人员: 张涛
** 创建日期: 2019/6/24
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
/*******************************************************************************
 * BT1中断服务函数
 ******************************************************************************/
void Tim2_IRQHandler(void)
{
	if (TRUE == Bt_GetIntFlag(TIM2))
	{
//		Bt_ClearIntFlag(TIM2);
//		Bt_Stop(TIM2);
		M0P_TIM2->ICLR_f.UIF = FALSE;
		M0P_TIM2->CR_f.CTEN = FALSE;
		Adc_Start();
//		port_on;
//		port_off;
	}
}

void Tim6_IRQHandler(void)
{
    if(TRUE == Adt_GetIrqFlag(M0P_ADTIM6, AdtUDFIrq))
    {
			Adt_ClearIrqFlag(M0P_ADTIM6, AdtUDFIrq);
      Adt_SetCompareValue(M0P_ADTIM6, AdtCompareC,OUT_PUT );//设置GCMCR寄存器，通过缓存传送GCMCR-->GCMAR，改变CHA通道的PWM占空比
			
			tim_handle.tim6_cur++;
			
//			if(adc_handle.status == SELECT_CH12 && M0P_ADC->IFR_f.CONT_INTF == 0)
//			{
//				 Adc_Start();
//			}
//			if(adc_handle.status == SELECT_CH0 && M0P_ADC->IFR_f.CONT_INTF == 0)
//			{
//					tim2run();
//			}
    }
}


