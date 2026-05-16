#include "common.h"
#include "my_pwm.h"
#include "decode.h"

uint8_t  ms10_flag=0;
uint8_t  ms100_flag=0;
uint8_t  ms250_flag=0;
uint8_t  ms500_flag=0;
uint8_t  ms1000_flag=0;

///< AdvTimer端口初始化
void App_AdvTimerPortInit(void)
{
    stc_gpio_cfg_t         stcTIMPort;
	
    DDL_ZERO_STRUCT(stcTIMPort);
 
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //端口外设时钟使能
    stcTIMPort.enDrv  = GpioDrvH;
    stcTIMPort.enDir  = GpioDirOut;	

    //设置为TIM4_CHB
    Gpio_Init(GpioPort1, GpioPin5, &stcTIMPort);
    Gpio_SetAfMode(GpioPort1,GpioPin5,GpioAf3);    
}


///< AdvTimer4初始化
void App_AdvTimer4Init(uint16_t u16Period, uint16_t u16CHA_PWMDuty, uint16_t u16CHB_PWMDuty)
{
    en_adt_compare_t          enAdtCompareB;

    stc_adt_basecnt_cfg_t     stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIM4ACfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIM4BCfg;   
    
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIM4ACfg);
    DDL_ZERO_STRUCT(stcAdtTIM4BCfg);   

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);    //ADT外设时钟使能
    
    stcAdtBaseCntCfg.enCntMode = AdtSawtoothMode;                 //锯齿波模式
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div4;
    
    Adt_Init(M0P_ADTIM4, &stcAdtBaseCntCfg);                      //ADT载波、计数模式、时钟配置
    
    Adt_SetPeriod(M0P_ADTIM4, u16Period);                         //周期设置

    enAdtCompareB = AdtCompareB;
    Adt_SetCompareValue(M0P_ADTIM4, enAdtCompareB, u16CHB_PWMDuty);  //通用比较基准值寄存器B设置    

    stcAdtTIM4BCfg.enCap = AdtCHxCompareOutput;
    stcAdtTIM4BCfg.bOutEn = TRUE;
    stcAdtTIM4BCfg.enPerc = AdtCHxPeriodLow;
    stcAdtTIM4BCfg.enCmpc = AdtCHxCompareHigh;
    stcAdtTIM4BCfg.enStaStp = AdtCHxStateSelSS;
    stcAdtTIM4BCfg.enStaOut = AdtCHxPortOutLow;
    stcAdtTIM4BCfg.enStpOut = AdtCHxPortOutLow;
        
    Adt_CHxXPortCfg(M0P_ADTIM4, AdtCHxB, &stcAdtTIM4BCfg);    //端口CHB配置
}

///< AdvTimer6初始化
void App_AdvTimer6Init(uint16_t u16Period, uint16_t u16CHA_PWMDuty, uint16_t u16CHB_PWMDuty)
{
    en_adt_compare_t          enAdtCompareA;
	
    stc_adt_basecnt_cfg_t     stcAdtBaseCntCfg;
	
    stc_adt_CHxX_port_cfg_t   stcAdtTIM6ACfg;
   
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIM6ACfg);    

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);    //ADT外设时钟使能
    
    stcAdtBaseCntCfg.enCntMode = AdtSawtoothMode;                 //锯齿波模式
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0;
	 
    Adt_Init(M0P_ADTIM6, &stcAdtBaseCntCfg);                      //ADT载波、计数模式、时钟配置
    
    Adt_SetPeriod(M0P_ADTIM6, u16Period);                         //周期设置
    
    enAdtCompareA = AdtCompareA;
    Adt_SetCompareValue(M0P_ADTIM6, enAdtCompareA, u16CHA_PWMDuty);  //通用比较基准值寄存器A设置   

    stcAdtTIM6ACfg.enCap = AdtCHxCompareOutput;            //比较输出
    stcAdtTIM6ACfg.bOutEn = TRUE;                          //CHA输出使能
    stcAdtTIM6ACfg.enPerc = AdtCHxPeriodLow;               //计数值与周期匹配时CHA电平保持不变
    stcAdtTIM6ACfg.enCmpc = AdtCHxCompareHigh;             //计数值与比较值A匹配时，CHA电平翻转
    stcAdtTIM6ACfg.enStaStp = AdtCHxStateSelSS;            //CHA起始结束电平由STACA与STPCA控制
    stcAdtTIM6ACfg.enStaOut = AdtCHxPortOutLow;            //CHA起始电平为低
    stcAdtTIM6ACfg.enStpOut = AdtCHxPortOutLow;            //CHA结束电平为低
    Adt_CHxXPortCfg(M0P_ADTIM6, AdtCHxA, &stcAdtTIM6ACfg);   //端口CHA配置   
}


//void App_BtTimerTest(void)
//{
//    stc_bt_cfg_t   	  stcCfg;
//    uint16_t          u16ArrData = 0x10000 - 24000;    // 65536-15000  15000*16/24M=10ms
//    uint16_t          u16InitCntData = 0x10000 - 10000;

//    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE);

//    stcCfg.enGateP = BtPositive;
//    stcCfg.enGate  = BtGateDisable;
//    stcCfg.enPRS   = BtPCLKDiv1;	// 不分频
//    stcCfg.enTog   = BtTogDisable;
//    stcCfg.enCT    = BtTimer; 		// 定时器功能
//    stcCfg.enMD    = BtMode2;		// 自动重装载16位定时器

//    //Bt配置初始化
//    if (Ok != Bt_Init(TIM0, &stcCfg))
//    {
//			
//    }

//    //INT ENABLE
//    Bt_ClearIntFlag(TIM0);
//    Bt_EnableIrq(TIM0);
//    EnableNvic(TIM0_IRQn, IrqLevel3, TRUE);

//    //设置重载值和计数值，启动计数
//    Bt_ARRSet(TIM0, u16ArrData);
//    Bt_Cnt16Set(TIM0, u16InitCntData);
//    Bt_Run(TIM0);
//}

void App_BtTimerTest2(void)
{
    stc_bt_cfg_t   	  stcCfg;
    uint16_t          u16ArrData = 0x10000 - 24000;//0x10000 - 1434;		// 65536-1440  1440/24M= =60us中断一次
    uint16_t          u16InitCntData = 0x10000 - 10000;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE);

    stcCfg.enGateP = BtPositive;
    stcCfg.enGate  = BtGateDisable;
    stcCfg.enPRS   = BtPCLKDiv1;	// 不分频
    stcCfg.enTog   = BtTogDisable;
    stcCfg.enCT    = BtTimer; 		// 定时器功能
    stcCfg.enMD    = BtMode2;		// 自动重装载16位定时器

    //Bt配置初始化
    if (Ok != Bt_Init(TIM2, &stcCfg))
    {
			
    }

    //INT ENABLE
    Bt_ClearIntFlag(TIM2);
    Bt_EnableIrq(TIM2);
    EnableNvic(TIM2_IRQn, IrqLevel1, TRUE);

    //设置重载值和计数值，启动计数
    Bt_ARRSet(TIM2, u16ArrData);
    Bt_Cnt16Set(TIM2, u16InitCntData);
    Bt_Run(TIM2);
}


uint8_t uart_time_error;
static uint16_t cnt1=0;
static uint16_t cnt2=0;
static uint16_t cnt3=0;
static uint16_t cnt4=0;
static uint16_t cnt5=0;
/*******************************************************************************
 * TIM0中断服务函数
 ******************************************************************************/
void Tim2_IRQHandler(void)
{
    if (TRUE == Bt_GetIntFlag(TIM2))
    {
				cnt1++;
				if(cnt1>10)//10ms
				{
					cnt1=0;
					ms10_flag=1;
				}
				
				cnt2++;
				if(cnt2>100)//100ms
				{
					cnt2=0;
					ms100_flag=1;
				}
				
				cnt3++;
				if(cnt3 >250)//250ms
				{
					cnt3=0;
					 ms250_flag=1;
				}
				
				cnt4++;
				if(cnt4>500)//500ms
				{
						cnt4=0;
						ms500_flag=1;
				}
				
				cnt5++;
				if(cnt5>1000)//1000ms
				{
						cnt5=0;
						ms1000_flag=1;
				}
				
        Bt_ClearIntFlag(TIM2);		
    }
}


//uint16_t add_tim=0;
//_Bool foha_led;


//void Tim2_IRQHandler(void)
//{
//  if (TRUE == Bt_GetIntFlag(TIM2))//60us进入一次
//  {
//    RF433M_RecevieDecode();
//    Bt_ClearIntFlag(TIM2);				
//  }
//}

void App_PcaInit(void)
{
	App_AdvTimer4Init(CBValue-1, CBValue, CBValue);
	
	Adt_StartCount(M0P_ADTIM4); //AdvTimer4运行
	App_AdvTimerPortInit();//端口初始化
	Adt_SetCompareValue(M0P_ADTIM4, AdtCompareB, CBValue);  //P24
	
//	App_BtTimerTest();//初始化定时器
	App_BtTimerTest2();//初始化定时器2
}


uint16_t pwm_vb=0;
void set_pwm_pca(uint8_t sw,uint16_t value)
{
	pwm_vb=value;
	
	if(value>CBValue)
	{
		value=CBValue;
	}
	
	if(sw==1)
	{
		Adt_SetCompareValue(M0P_ADTIM6, AdtCompareA, CBValue);  //P23
		Adt_SetCompareValue(M0P_ADTIM4, AdtCompareB, CBValue-value);  //P24
	}else if(sw==2)
	{	
		Adt_SetCompareValue(M0P_ADTIM4, AdtCompareB, CBValue);  //P24
		Adt_SetCompareValue(M0P_ADTIM6, AdtCompareA, CBValue-value);  //P23
	}else
	{
		Adt_SetCompareValue(M0P_ADTIM6, AdtCompareA, CBValue);
		Adt_SetCompareValue(M0P_ADTIM4, AdtCompareB, CBValue);
	}
}

void set_pwm_beep(uint16_t value)
{
	Adt_SetCompareValue(M0P_ADTIM4, AdtCompareB, CBValue-value);  //P23
}










