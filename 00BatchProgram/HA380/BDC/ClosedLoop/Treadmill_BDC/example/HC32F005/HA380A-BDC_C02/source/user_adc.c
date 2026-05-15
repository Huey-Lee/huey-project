/* user_adc.c - motor current, bus: ADC sample, filter, scale. */
#include "user_adc.h"
#include "bgr.h"
#include "gpio.h"
#include "motor_speed_pid.h"

adc_t adc_handle;
volatile uint32_t u32AdcResultAcc;
extern motor_t motor;
//ʹ��ADC��������
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

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);  //ADCBGR ����ʱ��ʹ��

    Adc_Enable();
    Bgr_BgrEnable();    ///< BGR����ʹ��

    stcAdcCfg.enAdcOpMode = AdcScanMode;               //ɨ�����ģʽ
    stcAdcCfg.enAdcClkSel = AdcClkSysTDiv1;            //PCLK
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime8Clk;      //8������ʱ��
    stcAdcCfg.enAdcRefVolSel = RefVolSelAVDD;      //�ο���ѹ:AVDD
    stcAdcCfg.bAdcInBufEn = FALSE;                     //��ѹ���������ʹ�ܣ�SPS�������� <=200K
    stcAdcCfg.enAdcTrig0Sel = AdcTrigDisable;          //ADCת���Զ���������
    stcAdcCfg.enAdcTrig1Sel = AdcTrigDisable;
    Adc_Init(&stcAdcCfg);

    stcAdcIrq.bAdcIrq = TRUE;                          //ת������жϺ����������ʹ��
    stcAdcIrq.bAdcRegCmp = FALSE;
    stcAdcIrq.bAdcHhtCmp = FALSE;
    stcAdcIrq.bAdcLltCmp = FALSE;
    Adc_CmpCfg(&stcAdcIrq);                            //����Ƚ��ж�ʹ��/��ֹ����

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

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);  //ADCBGR ����ʱ��ʹ��

    Adc_Enable();
    Bgr_BgrEnable();    ///< BGR����ʹ��

    stcAdcCfg.enAdcOpMode = AdcScanMode;               //ɨ�����ģʽ
    stcAdcCfg.enAdcClkSel = AdcClkSysTDiv1;            //PCLK
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime8Clk;      //8������ʱ��
    stcAdcCfg.enAdcRefVolSel = RefVolSelAVDD;          //�ο���ѹ:AVDD
    stcAdcCfg.bAdcInBufEn = FALSE;                     //��ѹ���������ʹ�ܣ�SPS�������� <=200K
    stcAdcCfg.enAdcTrig0Sel = AdcTrigDisable;          //ADCת���Զ���������
    stcAdcCfg.enAdcTrig1Sel = AdcTrigDisable;
    Adc_Init(&stcAdcCfg);

    stcAdcIrq.bAdcIrq = TRUE;                          //ת������жϺ����������ʹ��
    stcAdcIrq.bAdcRegCmp = FALSE;
    stcAdcIrq.bAdcHhtCmp = FALSE;
    stcAdcIrq.bAdcLltCmp = FALSE;
    Adc_CmpCfg(&stcAdcIrq);                            

    stcAdcScanCfg.u8AdcScanModeCh =  ADC_SCAN_CH1_EN
                                   | ADC_SCAN_CH2_EN;

    stcAdcScanCfg.u8AdcSampCnt = 0x1;                  //����ɨ��ת������������ͨ���ı�����6ͨ�� = 0x5+1(1��)������11+1(2��)����
    Adc_ConfigScanMode(&stcAdcCfg, &stcAdcScanCfg);

    Adc_EnableIrq();                                   //�ж�ʹ��
    EnableNvic(ADC_IRQn, IrqLevel2, TRUE);
}
/*******************************************************************************
** ��������: uint16_t get_rocker1(void)
** ��������: ��ȡADC����
** ����˵��: None
** ����˵��: None
** ������Ա: ����
** ��������: 2019/6/24
**------------------------------------------------------------------------------
** �޸���Ա:
** �޸�����:
** �޸�����:
**------------------------------------------------------------------------------
********************************************************************************/
void get_adc_value(void)
{
	static uint8_t flag;
	if(adc_handle.irq_flag >= GET_ADC_len)
	{
		adc_handle.irq_flag = 0;
		
//		motor.valtage_cur = adc_zhonzhi(adc0data,GET_ADC_len);
//		motor.valtage_down = adc_zhonzhi(adc1data,GET_ADC_len);
//		motor.valtage_up = adc_zhonzhi(adc2data,GET_ADC_len);
		
		motor.valtage_cur  = ADC_Value_Handle(adc0data,GET_ADC_len);
		motor.valtage_down = ADC_Value_Handle(adc1data,GET_ADC_len);
		motor.valtage_up   = ADC_Value_Handle(adc2data,GET_ADC_len);
		
		motor_speed_pid_isr(motor);

		flag = !flag;
		if(flag == 0)adc_handle.status = CH0StREALY;
		else adc_handle.status = CH12StREALY;
	}
}
/*******************************************************************************
** ��������: uint16_t ADC_Value_Handle(uint16_t *data,uint8_t times)
** ��������: ����ADC������
** ����˵��: None
** ����˵��: None
** ������Ա: ����
** ��������: 2019/6/24
**------------------------------------------------------------------------------
** �޸���Ա:
** �޸�����:
** �޸�����:
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
	if(GET_ADC_len>2)
	adc_value_1 = (adc_sum - MAX - MIN)/ (GET_ADC_len - 2);
	else if(GET_ADC_len==2)
		adc_value_1 = adc_sum / 2;
/*****************************************************************************/
	return adc_value_1;
}

/*******************************************************************************
** ��������: uint8_t rocker_zhonzhi(uint8_t ch)
** ��������: ��ֵƽ���˲�
** ����˵��: data	:���ݲ���ָ�� times	:���ݲɼ�����
** ����˵��: None
** ������Ա: ����
** ��������: 2019/6/24
**------------------------------------------------------------------------------
** �޸���Ա:
** �޸�����:
** �޸�����:
**------------------------------------------------------------------------------
********************************************************************************/
uint16_t adc_zhonzhi(volatile uint16_t *data,uint8_t times)
{
	uint8_t i,j,k;
	uint16_t temp;
	uint16_t valuehandle[GET_ADC_len];
	for(k=0;k<times;k++)
	valuehandle[k] = data[k];
	for (j=1;j<times;j++)								//�Դ�������һ�����ݾ�������(ð������)��ķ��ں���
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
								stcAdcScanCfg.u8AdcSampCnt = 0x1;                  //����ɨ��ת������������ͨ���ı�����6ͨ�� = 0x5+1(1��)������11+1(2��)����
								Adc_ConfigScanMode(&stcAdcCfg, &stcAdcScanCfg);
								adc_handle.status = SELECT_CH0;

			break;
		case SELECT_CH0:  


			break;
		case CH12StREALY:
//								App_AdcInitch12();
								stcAdcScanCfg.u8AdcScanModeCh =  ADC_SCAN_CH1_EN
																							 | ADC_SCAN_CH2_EN;
								stcAdcScanCfg.u8AdcSampCnt = 0x1;                  //����ɨ��ת������������ͨ���ı�����6ͨ�� = 0x5+1(1��)������11+1(2��)����
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
** ��������: void user_adc_init(void)
** ��������: ADC���ݳ�ʼ��
** ����˵��: CH12StREALY    CH0StREALY
** ����˵��: None
** ������Ա: ����
** ��������: 2019/6/24
**------------------------------------------------------------------------------
** �޸���Ա:
** �޸�����:
** �޸�����:
**------------------------------------------------------------------------------
********************************************************************************/
void user_adc_init(void)
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
///< ADC �жϷ������
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
					Adc_GetScanResult(0, &adc0data[adc_handle.irq_flag]);
					break;
					
					case(SELECT_CH12):
					Adc_GetScanResult(1, &adc1data[adc_handle.irq_flag]);
					Adc_GetScanResult(2, &adc2data[adc_handle.irq_flag]);
					break;
				}
				adc_handle.irq_flag++;
				get_adc_value();
				Adc_ClrContIrqState();     //�������жϱ�־λ
    }
}



