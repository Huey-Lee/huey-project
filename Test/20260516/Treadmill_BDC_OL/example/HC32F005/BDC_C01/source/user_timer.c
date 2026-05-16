/*
 * Function: 板级基础定时器与 ADTIM6：TIM0/2、PWM 载波、TIM6 中断刷新占空。
 * Method:   BT 配 TIM2 与 TIM0；ADT 三角波、周期/比较、UDF 触发 NVIC；底点可配硬件 ADC 触发；tim6run 内双次 App_AdvTimerInit；TIM6 ISR 写比较值并递增 tim6_cur。
 * Name:     Huey
 * Date:     May 16, 2026 18:00
 */

#include "user_timer.h"
#include "motor.h"
#include "gpio.h"
#include "sysctrl.h"
#include "ddl.h"

volatile uint16_t ZHANKONBI = MT_START_PWM;
#define OUT_PUT ((MOTOR_PWM_PERIOD_TICKS) - (ZHANKONBI))

tim_t tim_handle = {0};

void user_pwm_pins_bootstrap_safe_low_first(void)
{
    stc_gpio_cfg_t cfg;

    (void)Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    DDL_ZERO_STRUCT(cfg);
    cfg.bOutputVal = FALSE; /* 低电平 */
    cfg.enDir      = GpioDirOut;
    cfg.enDrv      = GpioDrvH;
    cfg.enPu       = GpioPuDisable;
    cfg.enPd       = GpioPdDisable;
    cfg.enOD       = GpioOdDisable;

    /* 与 App_GpioInit / App_AdvTimerPortInit 同源：P01→TIM0、P02→ADTIM6 PWM */
    (void)Gpio_Init(GpioPort0, GpioPin1, &cfg);
    (void)Gpio_Init(GpioPort0, GpioPin2, &cfg);
}

static en_result_t App_BtTimerTest(void)
{
    stc_bt_cfg_t   		stcCfg;
    en_result_t       enResult = Error;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE);

    stcCfg.enGateP = BtPositive;
    stcCfg.enGate  = BtGateDisable;
    stcCfg.enPRS   = BtPCLKDiv8;
    stcCfg.enTog   = BtTogDisable;
    stcCfg.enCT    = BtTimer;
    stcCfg.enMD    = BtMode2;
    if (Ok != Bt_Init(TIM2, &stcCfg))
        enResult = Error;
    else
        enResult = Ok;

    Bt_ClearIntFlag(TIM2);
    Bt_EnableIrq(TIM2);
    EnableNvic(TIM2_IRQn, IrqLevel0, TRUE);

    return enResult;
}

static en_result_t App_BtTogTest(void)
{
    stc_bt_cfg_t   stcCfg;
    en_result_t       enResult = Ok;
    uint16_t          u16ArrData = 0xF6A0;
    uint16_t          u16InitCntData = 0xF6A0;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt, TRUE);

    stcCfg.enGateP = BtPositive;
    stcCfg.enGate  = BtGateDisable;
    stcCfg.enPRS   = BtPCLKDiv1;
    stcCfg.enTog   = BtTogEnable;
    stcCfg.enCT    = BtTimer;
    stcCfg.enMD    = BtMode2;

    if (Ok != Bt_Init(TIM0, &stcCfg))
        enResult = Error;

    Bt_ARRSet(TIM0, u16ArrData);
    Bt_Cnt16Set(TIM0, u16InitCntData);

    return enResult;
}

void App_AdvTimerInit(uint16_t u16Period, uint16_t u16CHA_PWMDuty, uint16_t u16CHB_PWMDuty)
{
    en_adt_compare_t          enAdtCompare;
    stc_adt_basecnt_cfg_t     stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t   stcAdtTIM6ACfg;

    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIM6ACfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);

    stcAdtBaseCntCfg.enCntMode = AdtTriangleModeA;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0;
    Adt_Init(M0P_ADTIM6, &stcAdtBaseCntCfg);

    {
        stc_adt_irq_trig_cfg_t stcAdtIrqTrig;
        DDL_ZERO_STRUCT(stcAdtIrqTrig);
        stcAdtIrqTrig.bAdtUnderFlowTrigEn = TRUE;
        (void)Adt_IrqTrigCfg(M0P_ADTIM6, &stcAdtIrqTrig);
    }

    Adt_SetPeriod(M0P_ADTIM6, u16Period);

    enAdtCompare = AdtCompareA;
    Adt_SetCompareValue(M0P_ADTIM6, enAdtCompare, u16CHA_PWMDuty);

    enAdtCompare = AdtCompareC;
    Adt_SetCompareValue(M0P_ADTIM6, enAdtCompare, u16CHA_PWMDuty);

    Adt_EnableValueBuf(M0P_ADTIM6, AdtCHxA, TRUE);

    stcAdtTIM6ACfg.enCap = AdtCHxCompareOutput;
    stcAdtTIM6ACfg.bOutEn = TRUE;
    stcAdtTIM6ACfg.enPerc = AdtCHxPeriodKeep;
    stcAdtTIM6ACfg.enCmpc = AdtCHxCompareInv;
    stcAdtTIM6ACfg.enStaStp = AdtCHxStateSelSS;
    stcAdtTIM6ACfg.enStaOut = AdtCHxPortOutHigh;
    stcAdtTIM6ACfg.enStpOut = AdtCHxPortOutHigh;
//		stcAdtTIM6ACfg.enStaOut = AdtCHxPortOutLow;
//    stcAdtTIM6ACfg.enStpOut = AdtCHxPortOutLow;
    Adt_CHxXPortCfg(M0P_ADTIM6, AdtCHxA, &stcAdtTIM6ACfg);

    Adt_ClearAllIrqFlag(M0P_ADTIM6);
    Adt_CfgIrq(M0P_ADTIM6, AdtUDFIrq, TRUE);
    EnableNvic(ADTIM6_IRQn, IrqLevel1, TRUE);
}

static void App_GpioInit(void)
{
    stc_gpio_cfg_t         stcTIM0Port;

    DDL_ZERO_STRUCT(stcTIM0Port);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    stcTIM0Port.enDir  = GpioDirOut;
    Gpio_Init(GpioPort0, GpioPin1, &stcTIM0Port);
    Gpio_SetAfMode(GpioPort0,GpioPin1,GpioAf4);
}

void App_AdvTimerPortInit(void)
{
    stc_gpio_cfg_t         stcTIM6Port;

    DDL_ZERO_STRUCT(stcTIM6Port);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    stcTIM6Port.enDir  = GpioDirOut;
    Gpio_Init(GpioPort0, GpioPin2, &stcTIM6Port);
    Gpio_SetAfMode(GpioPort0,GpioPin2,GpioAf5);
}

void user_timer_init(void)
{
    (void)App_BtTimerTest();
    App_GpioInit();

    App_AdvTimerPortInit();
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
    App_AdvTimerInit(MOTOR_PWM_PERIOD_TICKS, (uint16_t)(MOTOR_PWM_PERIOD_TICKS - 1u), 1);
    App_AdvTimerInit(MOTOR_PWM_PERIOD_TICKS, (uint16_t)(MOTOR_PWM_PERIOD_TICKS - 1u), 1);
    Adt_ClearCount(M0P_ADTIM6);
    Adt_StartCount(M0P_ADTIM6);
}

void tim6stop(void)
{
    Adt_ClearCount(M0P_ADTIM6);
    Adt_StopCount(M0P_ADTIM6);
}

void Tim2_IRQHandler(void)
{
	if (TRUE == Bt_GetIntFlag(TIM2))
	{
		M0P_TIM2->ICLR_f.UIF = FALSE;
		M0P_TIM2->CR_f.CTEN = FALSE;
	}
}

void Tim6_IRQHandler(void)
{
    if(TRUE == Adt_GetIrqFlag(M0P_ADTIM6, AdtUDFIrq))
    {
			Adt_ClearIrqFlag(M0P_ADTIM6, AdtUDFIrq);
      if (ZHANKONBI > (uint16_t)MOTOR_PWM_PERIOD_TICKS)
          ZHANKONBI = (uint16_t)MOTOR_PWM_PERIOD_TICKS;
      Adt_SetCompareValue(M0P_ADTIM6, AdtCompareC,OUT_PUT );

			tim_handle.tim6_cur++;
    }
}
