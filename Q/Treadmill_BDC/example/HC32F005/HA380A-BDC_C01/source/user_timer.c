/* user_timer.c - board BT/ADT timers, PWM, tick, TIM init. */
#include "user_timer.h"
#include "motor.h"
#include "adc.h"
#include "gpio.h"
#include "user_adc.h"

extern adc_t adc_handle;
volatile uint16_t ZHANKONBI = MT_START_PWM;
#define OUT_PUT (545 - ZHANKONBI)

tim_t tim_handle = {0};
uint8_t u8TestFlag;
/*******************************************************************************
 * BT��ʱ���ܲ��� ������ģʽ��
 ******************************************************************************/
static en_result_t App_BtTimerTest(void)
{
    stc_bt_cfg_t   		stcCfg;
    en_result_t       enResult = Error;
//    uint16_t          u16ArrData = 0XC568;			//65536 - 30000      30000 * 8 /24M = 10mS
//    uint16_t          u16InitCntData = 0XC568;
		
    //��BT����ʱ��
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE);
		
    stcCfg.enGateP = BtPositive;
    stcCfg.enGate  = BtGateDisable;
    stcCfg.enPRS   = BtPCLKDiv8;			//256��Ƶ
    stcCfg.enTog   = BtTogDisable;
    stcCfg.enCT    = BtTimer;
    stcCfg.enMD    = BtMode2;
    //Bt��ʼ��
    if (Ok != Bt_Init(TIM2, &stcCfg))
    {
        enResult = Error;
    }
		
    //TIM1�ж�ʹ��
    Bt_ClearIntFlag(TIM2);
    Bt_EnableIrq(TIM2);
    EnableNvic(TIM2_IRQn, IrqLevel0, TRUE);
		
    //��������ֵ�ͼ���ֵ���������
		
    return enResult;
}
/*******************************************************************************
 * BT Buzzer���ܲ��� ��TOG��
 ******************************************************************************/
static en_result_t App_BtTogTest(void)
{
    stc_bt_cfg_t   stcCfg;
    en_result_t       enResult = Ok;
    ///<��4Mhz->1000Hz��
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
    
    //��������ֵ������ֵ���������
		Bt_ARRSet(TIM0, u16ArrData);
		Bt_Cnt16Set(TIM0, u16InitCntData);
    
    return enResult;
}
///< AdvTimer��ʼ��
void App_AdvTimerInit(uint16_t u16Period, uint16_t u16CHA_PWMDuty, uint16_t u16CHB_PWMDuty)
{
    en_adt_compare_t          enAdtCompare;
    stc_adt_basecnt_cfg_t     stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIM6ACfg;
    
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIM6ACfg);
		
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);    //ADT����ʱ��ʹ��
    
    
    stcAdtBaseCntCfg.enCntMode = AdtTriangleModeA;               //���ǲ�ģʽ
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0;
    Adt_Init(M0P_ADTIM6, &stcAdtBaseCntCfg);                      //ADT�ز�������ģʽ��ʱ������
    
    Adt_SetPeriod(M0P_ADTIM6, u16Period);                         //��������
    
    enAdtCompare = AdtCompareA;
    Adt_SetCompareValue(M0P_ADTIM6, enAdtCompare, u16CHA_PWMDuty);  //ͨ�ñȽϻ�׼ֵ�Ĵ���A����
    
    enAdtCompare = AdtCompareC;
    Adt_SetCompareValue(M0P_ADTIM6, enAdtCompare, u16CHA_PWMDuty);  //ͨ�ñȽϻ�׼ֵ�Ĵ���C����
    
    Adt_EnableValueBuf(M0P_ADTIM6, AdtCHxA, TRUE);              //CHA buffer ʹ��
		
    
    stcAdtTIM6ACfg.enCap = AdtCHxCompareOutput;            //�Ƚ����
    stcAdtTIM6ACfg.bOutEn = TRUE;                          //CHA���ʹ��
    stcAdtTIM6ACfg.enPerc = AdtCHxPeriodKeep;              //����ֵ������ƥ��ʱCHA��ƽ���ֲ���
    stcAdtTIM6ACfg.enCmpc = AdtCHxCompareInv;              //����ֵ��Ƚ�ֵAƥ��ʱ��CHA��ƽ��ת
    stcAdtTIM6ACfg.enStaStp = AdtCHxStateSelSS;            //CHA��ʼ������ƽ��STACA��STPCA����
    stcAdtTIM6ACfg.enStaOut = AdtCHxPortOutHigh;           //CHA��ʼ��ƽΪ��
    stcAdtTIM6ACfg.enStpOut = AdtCHxPortOutHigh;           //CHA������ƽΪ��
    Adt_CHxXPortCfg(M0P_ADTIM6, AdtCHxA, &stcAdtTIM6ACfg); //�˿�CHA����

    Adt_ClearAllIrqFlag(M0P_ADTIM6);
    Adt_CfgIrq(M0P_ADTIM6, AdtUDFIrq, TRUE);  						 //�����ж�����
    EnableNvic(ADTIM6_IRQn, IrqLevel1, TRUE);
}
static void App_GpioInit(void)
{
    stc_gpio_cfg_t         stcTIM0Port;
	
    DDL_ZERO_STRUCT(stcTIM0Port);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);//�˿�����ʱ��ʹ��
    stcTIM0Port.enDir  = GpioDirOut;
    //P14����ΪTIM0_TOGN
    Gpio_Init(GpioPort0, GpioPin1, &stcTIM0Port);
    Gpio_SetAfMode(GpioPort0,GpioPin1,GpioAf4);
}
///< AdvTimer�˿ڳ�ʼ��
void App_AdvTimerPortInit(void)
{
    stc_gpio_cfg_t         stcTIM6Port;
    
    DDL_ZERO_STRUCT(stcTIM6Port);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //�˿�����ʱ��ʹ��
    
    stcTIM6Port.enDir  = GpioDirOut;
    //P34����ΪTIM4_CHA
    Gpio_Init(GpioPort0, GpioPin2, &stcTIM6Port);
    Gpio_SetAfMode(GpioPort0,GpioPin2,GpioAf5);
}
/*******************************************************************************
** ��������: void user_timer_init(void)
** ��������: TIM��ʼ��
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
void user_timer_init(void)
{
	if(Ok != App_BtTimerTest())
	{
			u8TestFlag |= 0x02;
	}
	App_GpioInit();
	
  App_AdvTimerPortInit();     //AdvTimer�˿ڳ�ʼ��
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
    App_AdvTimerInit(545, 544, 1);  //AdvTimer4��ʼ��
		App_AdvTimerInit(545, 544, 1);  //AdvTimer4��ʼ��
		Adt_ClearCount(M0P_ADTIM6);			//�������ֵ
    //����Ϊ���ǲ�ģʽ: ��ʼ������0xC000, CHAռ�ձ�����0x4000��CHBռ�ձ�����0x8000
    Adt_StartCount(M0P_ADTIM6);
}

void tim6stop(void)
{
	Adt_ClearCount(M0P_ADTIM6);									//�������ֵ
	Adt_StopCount(M0P_ADTIM6);
}

/*******************************************************************************
** ��������: void Tim2_IRQHandler(void)
** ��������: ��ʱ���жϻص�����
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
/*******************************************************************************
 * BT1�жϷ�����
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
      Adt_SetCompareValue(M0P_ADTIM6, AdtCompareC,OUT_PUT );//����GCMCR�Ĵ�����ͨ�����洫��GCMCR-->GCMAR���ı�CHAͨ����PWMռ�ձ�
			
			tim_handle.tim6_cur++;
			
			if(adc_handle.status == SELECT_CH12 && M0P_ADC->IFR_f.CONT_INTF == 0)
			{
				 Adc_Start();
			}
    }
}


