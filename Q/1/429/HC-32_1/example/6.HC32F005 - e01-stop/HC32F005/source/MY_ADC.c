#include "MY_ADC.h"
#include "bgr.h"
#include "gpio.h"
#include "motor_speed_pid.h"

adc_t adc_handle;
volatile uint32_t u32AdcResultAcc;
extern motor_t motor;
//使能ADC部分数组
//*******************************************************************************/
u16 adc0data[GET_ADC_len];
u16 adc1data[GET_ADC_len];
u16 adc2data[GET_ADC_len];

//u16 adc0val[GET_ADC_len];
//u16 adc1val[GET_ADC_len];
//u16 adc2val[GET_ADC_len];
//*******************************************************************************/
stc_adc_cfg_t             stcAdcCfg;
void clear_adcbuf(void)
{
	memset(adc0data,0,GET_ADC_len);
	memset(adc1data,0,GET_ADC_len);
	memset(adc2data,0,GET_ADC_len);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**********************************************************************************************************/
/**********************************************************************************************************/
/**********************************************************************************************************/
/**********************************************************************************************************/
/**********************************************************************************************************/
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void App_AdcInit(void)
{
//    stc_adc_cfg_t             stcAdcCfg;
    stc_adc_scan_cfg_t        stcAdcScanCfg;
    stc_adc_irq_t             stcAdcIrq;

    DDL_ZERO_STRUCT(stcAdcCfg);
    DDL_ZERO_STRUCT(stcAdcScanCfg);
    DDL_ZERO_STRUCT(stcAdcIrq);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);  //ADCBGR 外设时钟使能

    Adc_Enable();
    Bgr_BgrEnable();    ///< BGR必须使能

    stcAdcCfg.enAdcOpMode = AdcScanMode;               //扫描采样模式
    stcAdcCfg.enAdcClkSel = AdcClkSysTDiv1;            //PCLK
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime8Clk;      //8个采样时钟
    stcAdcCfg.enAdcRefVolSel = RefVolSelAVDD;      //参考电压:AVDD
    stcAdcCfg.bAdcInBufEn = FALSE;                     //电压跟随器如果使能，SPS采样速率 <=200K
    stcAdcCfg.enAdcTrig0Sel = AdcTrigDisable;          //ADC转换自动触发设置
    stcAdcCfg.enAdcTrig1Sel = AdcTrigDisable;
    Adc_Init(&stcAdcCfg);

    stcAdcIrq.bAdcIrq = TRUE;                          //转换完成中断函数入口配置使能
    stcAdcIrq.bAdcRegCmp = FALSE;
    stcAdcIrq.bAdcHhtCmp = FALSE;
    stcAdcIrq.bAdcLltCmp = FALSE;
    Adc_CmpCfg(&stcAdcIrq);                            //结果比较中断使能/禁止配置

    stcAdcScanCfg.u8AdcScanModeCh =  ADC_SCAN_CH0_EN;

    stcAdcScanCfg.u8AdcSampCnt = 1;                  	
    Adc_ConfigScanMode(&stcAdcCfg, &stcAdcScanCfg);

    Adc_EnableIrq();                                   //enable interrupt
    EnableNvic(ADC_IRQn, IrqLevel2, TRUE);
}
void App_AdcInitch12(void)
{

    stc_adc_scan_cfg_t        stcAdcScanCfg;
    stc_adc_irq_t             stcAdcIrq;

    DDL_ZERO_STRUCT(stcAdcCfg);
    DDL_ZERO_STRUCT(stcAdcScanCfg);
    DDL_ZERO_STRUCT(stcAdcIrq);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);  //ADCBGR 外设时钟使能

    Adc_Enable();
    Bgr_BgrEnable();    ///< BGR必须使能

    stcAdcCfg.enAdcOpMode = AdcScanMode;               //扫描采样模式
    stcAdcCfg.enAdcClkSel = AdcClkSysTDiv1;            //PCLK
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime8Clk;      //8个采样时钟
    stcAdcCfg.enAdcRefVolSel = RefVolSelAVDD;          //参考电压:AVDD
    stcAdcCfg.bAdcInBufEn = FALSE;                     //电压跟随器如果使能，SPS采样速率 <=200K
    stcAdcCfg.enAdcTrig0Sel = AdcTrigDisable;          //ADC转换自动触发设置
    stcAdcCfg.enAdcTrig1Sel = AdcTrigDisable;
    Adc_Init(&stcAdcCfg);

    stcAdcIrq.bAdcIrq = TRUE;                          //转换完成中断函数入口配置使能
    stcAdcIrq.bAdcRegCmp = FALSE;
    stcAdcIrq.bAdcHhtCmp = FALSE;
    stcAdcIrq.bAdcLltCmp = FALSE;
    Adc_CmpCfg(&stcAdcIrq);                            

    stcAdcScanCfg.u8AdcScanModeCh =  ADC_SCAN_CH1_EN
                                   | ADC_SCAN_CH2_EN;

    stcAdcScanCfg.u8AdcSampCnt = 0x1;                  //连续扫描转换次数，保持通道的倍数，6通道 = 0x5+1(1倍)，或者11+1(2倍)……
    Adc_ConfigScanMode(&stcAdcCfg, &stcAdcScanCfg);

    Adc_EnableIrq();                                   //中断使能
    EnableNvic(ADC_IRQn, IrqLevel2, TRUE);
}
/*******************************************************************************
** 函数名称: uint16_t get_rocker1(void)
** 功能描述: 获取ADC数据
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
void get_adc_value(void)
{
	if(adc_handle.irq_flag >= GET_ADC_len)
	{
		adc_handle.irq_flag = 0;
		
//		motor.valtage_cur = adc_zhonzhi(adc0data,GET_ADC_len);
//		motor.valtage_down = adc_zhonzhi(adc1data,GET_ADC_len);
//		motor.valtage_up = adc_zhonzhi(adc2data,GET_ADC_len);
		
//		motor.valtage_cur  = ADC_Value_Handle(adc0data,GET_ADC_len);
//		motor.valtage_down = ADC_Value_Handle(adc1data,GET_ADC_len);
//		motor.valtage_up   = ADC_Value_Handle(adc2data,GET_ADC_len);
		
//		motor.valtage_cur  = (adc0data[0]+adc0data[1])/2;
//		motor.valtage_down = (adc1data[0]+adc1data[1])/2;
//		motor.valtage_up   = (adc2data[0]+adc2data[1])/2;
			motor_speed_pid_isr(motor);

//		flag = !flag;
//		adc_handle.status = flag;
//		Get_Proc_loop();
	}
}
/*******************************************************************************
** 函数名称: uint16_t ADC_Value_Handle(uint16_t *data,uint8_t times)
** 功能描述: 处理ADC的数据
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
uint16_t ADC_Value_Handle(volatile uint16_t *data,uint8_t times)
{
	uint8_t k;
	uint16_t MAX=0,MIN=0;
	uint16_t adc_value_1;
	uint16_t adc_handle[GET_ADC_len];
	uint16_t adc_sum=0;

	for(k=0;k<times;k++)
	adc_handle[k] = data[k];

	MAX = adc_handle[0];
	MIN = adc_handle[0];
	for(uint8_t i=0;i<GET_ADC_len;i++)
  {
    if(adc_handle[i]>MAX)
    MAX=adc_handle[i];
    if(adc_handle[i]<MIN)
    MIN=adc_handle[i];
    adc_sum=adc_sum+adc_handle[i];
  }
	adc_value_1 = (adc_sum - MAX - MIN)/ (GET_ADC_len - 2);
/*****************************************************************************/
	return adc_value_1;
}

/*******************************************************************************
** 函数名称: uint8_t rocker_zhonzhi(uint8_t ch)
** 功能描述: 中值平均滤波
** 参数说明: data	:传递参数指针 times	:数据采集次数
** 返回说明: None
** 创建人员: 张涛
** 创建日期: 2019/6/24
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
uint16_t adc_zhonzhi(volatile uint16_t *data,uint8_t times)
{
	uint8_t i,j,k;
	uint16_t temp;
	uint16_t valuehandle[GET_ADC_len];
	for(k=0;k<times;k++)
	valuehandle[k] = data[k];
	for (j=1;j<times;j++)								//对传过来的一串数据经行排序(冒泡排序法)大的放在后面
	{
		 for (i=0;i<times-j;i++)					//5 - 1 = 4
		 {
					if (valuehandle[i]> valuehandle[i+1])
					{
							temp = valuehandle[i];
							valuehandle[i]= valuehandle[i+1];
							valuehandle[i+1] = temp;
					}
		 }
	}
	
	return valuehandle[times/2];
}
void Get_Proc_loop(void)
{
  stc_adc_scan_cfg_t        stcAdcScanCfg;
	switch(adc_handle.status)
	{
		case CH0StREALY:
//								App_AdcInit();
								stcAdcScanCfg.u8AdcScanModeCh =  ADC_SCAN_CH0_EN;
								stcAdcScanCfg.u8AdcSampCnt = 0x1;                  //连续扫描转换次数，保持通道的倍数，6通道 = 0x5+1(1倍)，或者11+1(2倍)……
								Adc_ConfigScanMode(&stcAdcCfg, &stcAdcScanCfg);
								adc_handle.status = SELECT_CH0;

			break;
		case SELECT_CH0:  


			break;
		case CH12StREALY:
//								App_AdcInitch12();
								stcAdcScanCfg.u8AdcScanModeCh =  ADC_SCAN_CH1_EN
																							 | ADC_SCAN_CH2_EN;
								stcAdcScanCfg.u8AdcSampCnt = 0x1;                  //连续扫描转换次数，保持通道的倍数，6通道 = 0x5+1(1倍)，或者11+1(2倍)……
								Adc_ConfigScanMode(&stcAdcCfg, &stcAdcScanCfg);
								adc_handle.status = SELECT_CH12;

			break;
		case SELECT_CH12:

			break;
		default:
			break;
	  }
}
/*******************************************************************************
** 函数名称: void MY_ADC_INIT(void)
** 功能描述: ADC数据初始化
** 参数说明: CH12StREALY    CH0StREALY
** 返回说明: None
** 创建人员: 张涛
** 创建日期: 2019/6/24
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
void MY_ADC_INIT(void)
{
	App_AdcInitch12();
	adc_handle.status = CH12StREALY;
	Get_Proc_loop();
	adc_handle.irq_flag=0;
//	Adc_Start();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**********************************************************************************************************/
/**********************************************************************************************************/
/**********************************************************************************************************/
/**********************************************************************************************************/
/**********************************************************************************************************/
////////////////////////////////////////////////////////////////////////////////////////////////////////////
///< ADC 中断服务程序
void Adc_IRQHandler(void)
{
//	static u8 CH;
//    if (TRUE == M0P_ADC->IFR_f.REG_INTF)
//    {
//        Adc_ClrRegIrqState();
//    }
//		
//    if (TRUE == M0P_ADC->IFR_f.HHT_INTF)
//    {
//        Adc_ClrHhtIrqState();
//    }
//		
//    if (TRUE == M0P_ADC->IFR_f.LLT_INTF)
//    {
//        Adc_ClrIrqLltState();
//    }
		
    if (TRUE == M0P_ADC->IFR_f.CONT_INTF)
    {
			
				switch(adc_handle.status)
				{
					case(SELECT_CH0):
					Adc_GetScanResult(0, &motor.valtage_cur);
					if(motor.valtage_cur>450)motor.valtage_cur=450;
					break;
					
					case(SELECT_CH12):
					Adc_GetScanResult(1, &motor.valtage_down);
					Adc_GetScanResult(2, &motor.valtage_up);
					break;
				}
				
//				flag = !flag;
				adc_handle.status = adc_handle.irq_flag;
				Get_Proc_loop();
				adc_handle.irq_flag++;
				get_adc_value();
				Adc_ClrContIrqState();     //最后清除中断标志位
    }
}



