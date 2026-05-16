/* user_timer.c - board BT/ADT timers, PWM, tick, TIM init. */
#include "user_timer.h"
#include "motor.h"
#include "gpio.h"

volatile uint16_t ZHANKONBI = MT_START_PWM;
#define OUT_PUT ((MOTOR_PWM_PERIOD_TICKS) - (ZHANKONBI))

tim_t tim_handle = {0};
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
    /* 【Bug修复】原代码：初值 enResult=Error，Bt_Init 成功时从未改为 Ok，
     * 导致 App_BtTimerTest() 恒返回 Error，user_timer_init 中判断分支始终为"失败"。
     * 修复：初始化成功后显式赋值 Ok */
    if (Ok != Bt_Init(TIM2, &stcCfg))
    {
        enResult = Error;
    }
    else
    {
        enResult = Ok;
    }

    /* TIM2 中断使能 */
    Bt_ClearIntFlag(TIM2);
    Bt_EnableIrq(TIM2);
    EnableNvic(TIM2_IRQn, IrqLevel0, TRUE);

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

    /* 三角波底点（UDF）硬件触发 ADC，避免 ISR 里 Adc_Start 的抖动 */
    {
        stc_adt_irq_trig_cfg_t stcAdtIrqTrig;
        DDL_ZERO_STRUCT(stcAdtIrqTrig);
        stcAdtIrqTrig.bAdtUnderFlowTrigEn = TRUE;
        (void)Adt_IrqTrigCfg(M0P_ADTIM6, &stcAdtIrqTrig);
    }
    
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
    (void)App_BtTimerTest();
    App_GpioInit();

    App_AdvTimerPortInit();     /* AdvTimer 端口 */
}
void tim0run(void)
{
    (void)App_BtTogTest();
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
    /* 与 HA380A-BDC_C03 一致：连续两次初始化，CHA 的比较缓冲(GCMCR→GCMAR)链路稳定。
       Adt_SetPeriod 实参 = MOTOR_PWM_PERIOD_TICKS（与 duty=V×PER/vbus、OUT_PUT=PER−ZHANKONBI 同源）。 */
    App_AdvTimerInit(MOTOR_PWM_PERIOD_TICKS, (uint16_t)(MOTOR_PWM_PERIOD_TICKS - 1u), 1);
    App_AdvTimerInit(MOTOR_PWM_PERIOD_TICKS, (uint16_t)(MOTOR_PWM_PERIOD_TICKS - 1u), 1);
    Adt_ClearCount(M0P_ADTIM6);   /* 清零计数器，从 0 开始同步输出 */
    Adt_StartCount(M0P_ADTIM6);   /* 启动 PWM 输出 */
}

void tim6stop(void)
{
    Adt_ClearCount(M0P_ADTIM6);  /* 清零计数器 */
    Adt_StopCount(M0P_ADTIM6);   /* 停止 PWM 输出 */
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
		/* ADC 由 ADTIM6 UDF 硬件触发；此处仅作 BT2 单次定时到期 */
//		port_on;
//		port_off;
	}
}

void Tim6_IRQHandler(void)
{
    if(TRUE == Adt_GetIrqFlag(M0P_ADTIM6, AdtUDFIrq))
    {
			Adt_ClearIrqFlag(M0P_ADTIM6, AdtUDFIrq);
      if (ZHANKONBI > (uint16_t)MOTOR_PWM_PERIOD_TICKS)
          ZHANKONBI = (uint16_t)MOTOR_PWM_PERIOD_TICKS;
      Adt_SetCompareValue(M0P_ADTIM6, AdtCompareC,OUT_PUT );//����GCMCR�Ĵ�����ͨ�����洫��GCMCR-->GCMAR���ı�CHAͨ����PWMռ�ձ�
			
			tim_handle.tim6_cur++;
    }
}


