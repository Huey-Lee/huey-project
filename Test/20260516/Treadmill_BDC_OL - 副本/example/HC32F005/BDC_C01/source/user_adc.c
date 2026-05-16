/*
 * Function: 电机电流与母线电压多通道 ADC：扫描模式、中断入环缓冲、滤波与驱动回调。
 * Method:   TIM6 底点触发扫描；ISR 写入 adc0/1/2data 并滑动均值；get_adc_value 选通 CH0/CH12 并调用 motor_drive_isr。
 * Name:     Huey
 * Date:     May 16, 2026 18:00
 */

#include "user_adc.h"
#include "bgr.h"
#include "gpio.h"
#include "motor.h"
#include "motor_drive.h"

adc_t adc_handle;
volatile uint32_t u32AdcResultAcc;
volatile uint32_t adc_batch_seq;

u16 adc0data[GET_ADC_len];
u16 adc1data[GET_ADC_len];
u16 adc2data[GET_ADC_len];

stc_adc_cfg_t             stcAdcCfg;

static u16  s_adc_ring0[GET_ADC_len];
static u16  s_adc_ring1[GET_ADC_len];
static u16  s_adc_ring2[GET_ADC_len];
static u8   s_adc_pos0;
static u8   s_adc_pos1;
static u8   s_adc_pos2;
static u32  s_adc_sum0;
static u32  s_adc_sum1;
static u32  s_adc_sum2;

static void adc_sliding_reset(void)
{
    memset(s_adc_ring0, 0, sizeof(s_adc_ring0));
    memset(s_adc_ring1, 0, sizeof(s_adc_ring1));
    memset(s_adc_ring2, 0, sizeof(s_adc_ring2));
    s_adc_pos0 = s_adc_pos1 = s_adc_pos2 = 0u;
    s_adc_sum0 = s_adc_sum1 = s_adc_sum2 = 0u;
}

static void adc_sliding_push0(u16 v)
{
    u16 old       = s_adc_ring0[s_adc_pos0];
    s_adc_ring0[s_adc_pos0] = v;
    s_adc_sum0   += (u32)v - (u32)old;
    s_adc_pos0   = (u8)((s_adc_pos0 + 1u) % (u8)GET_ADC_len);
}

static void adc_sliding_push1(u16 v)
{
    u16 old       = s_adc_ring1[s_adc_pos1];
    s_adc_ring1[s_adc_pos1] = v;
    s_adc_sum1   += (u32)v - (u32)old;
    s_adc_pos1   = (u8)((s_adc_pos1 + 1u) % (u8)GET_ADC_len);
}

static void adc_sliding_push2(u16 v)
{
    u16 old       = s_adc_ring2[s_adc_pos2];
    s_adc_ring2[s_adc_pos2] = v;
    s_adc_sum2   += (u32)v - (u32)old;
    s_adc_pos2   = (u8)((s_adc_pos2 + 1u) % (u8)GET_ADC_len);
}

void clear_adcbuf(void)
{
	memset(adc0data,0,GET_ADC_len * sizeof(u16));
	memset(adc1data,0,GET_ADC_len * sizeof(u16));
	memset(adc2data,0,GET_ADC_len * sizeof(u16));
	adc_sliding_reset();
}

void App_AdcInit(void)
{
    stc_adc_scan_cfg_t        stcAdcScanCfg;
    stc_adc_irq_t             stcAdcIrq;

    DDL_ZERO_STRUCT(stcAdcCfg);
    DDL_ZERO_STRUCT(stcAdcScanCfg);
    DDL_ZERO_STRUCT(stcAdcIrq);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);

    Adc_Enable();
    Bgr_BgrEnable();

    stcAdcCfg.enAdcOpMode = AdcScanMode;
    stcAdcCfg.enAdcClkSel = AdcClkSysTDiv1;
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime8Clk;
    stcAdcCfg.enAdcRefVolSel = RefVolSelAVDD;
    stcAdcCfg.bAdcInBufEn = FALSE;
    stcAdcCfg.enAdcTrig0Sel = AdcTrigTimer6;
    stcAdcCfg.enAdcTrig1Sel = AdcTrigDisable;
    Adc_Init(&stcAdcCfg);

    stcAdcIrq.bAdcIrq = TRUE;
    stcAdcIrq.bAdcRegCmp = FALSE;
    stcAdcIrq.bAdcHhtCmp = FALSE;
    stcAdcIrq.bAdcLltCmp = FALSE;
    Adc_CmpCfg(&stcAdcIrq);

    stcAdcScanCfg.u8AdcScanModeCh =  ADC_SCAN_CH0_EN;

    stcAdcScanCfg.u8AdcSampCnt = 1;
    Adc_ConfigScanMode(&stcAdcCfg, &stcAdcScanCfg);

    Adc_EnableIrq();
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
    stcAdcCfg.enAdcTrig0Sel = AdcTrigTimer6;
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

		if (adc_handle.status == SELECT_CH12) {
			motor.valtage_up   = (u16)(s_adc_sum2 / (u32)GET_ADC_len);
			motor.valtage_down = (u16)(s_adc_sum1 / (u32)GET_ADC_len);
		} else {
			motor.valtage_cur  = (u16)(s_adc_sum0 / (u32)GET_ADC_len);
		}

		motor_drive_isr();
		flag = !flag;
		if(flag == 0)adc_handle.status = CH0StREALY;
		else adc_handle.status = CH12StREALY;
		/* 纯硬件触发时无 TIM2/主循环替你切通道：在此提交下一组 SCAN */
		Get_Proc_loop();
		adc_batch_seq++;
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
    /* 用预编译分支替代运行时 if，避免编译器对死代码产生"除零"警告 */
#if GET_ADC_len > 2
    /* 去掉最大值和最小值后取均值（采样数 > 2 时有效） */
    adc_value_1 = (adc_sum - MAX - MIN) / (GET_ADC_len - 2);
#else
    /* 采样数 = 2：直接取均值 */
    adc_value_1 = adc_sum / 2;
#endif
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
	adc_batch_seq = 0u;
	adc_sliding_reset();
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
					adc_sliding_push0(adc0data[adc_handle.irq_flag]);
					break;
					
					case(SELECT_CH12):
					Adc_GetScanResult(1, &adc1data[adc_handle.irq_flag]);
					Adc_GetScanResult(2, &adc2data[adc_handle.irq_flag]);
					adc_sliding_push1(adc1data[adc_handle.irq_flag]);
					adc_sliding_push2(adc2data[adc_handle.irq_flag]);
					break;
				}
				adc_handle.irq_flag++;
				get_adc_value();
				Adc_ClrContIrqState();     //�������жϱ�־λ
    }
}



