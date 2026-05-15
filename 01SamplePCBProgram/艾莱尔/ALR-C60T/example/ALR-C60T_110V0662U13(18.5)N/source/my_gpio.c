#include "my_gpio.h"
#include "decode.h"

void App_AdcInit(void)
{
  stc_adc_cfg_t      stcAdcCfg;
  stc_adc_norm_cfg_t stcAdcNormCfg;   

  Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);  //ADCBGR 外设时钟使能
    
  //ADC配置
  Adc_Enable();
  Bgr_BgrEnable();
   
  stcAdcCfg.enAdcOpMode = AdcNormalMode;          //单次采样模式
  stcAdcCfg.enAdcClkSel = AdcClkSysTDiv1;         //PCLK
  stcAdcCfg.enAdcSampTimeSel = AdcSampTime4Clk;   //4个采样时钟
  stcAdcCfg.enAdcRefVolSel = RefVolSelAVDD;       //参考电压:外部输入//内部2.5V(avdd>3V,SPS<=200kHz)  SPS速率 = ADC时钟 / (采样时钟 + 16CLK) 
  stcAdcCfg.bAdcInBufEn = FALSE;                  //电压跟随器如果使能，SPS采样速率 <=200K
  stcAdcCfg.u32AdcRegHighThd = 0u;                //比较阈值上门限
  stcAdcCfg.u32AdcRegLowThd = 0u;                 //比较阈值下门限
  stcAdcCfg.enAdcTrig0Sel = AdcTrigDisable;       //ADC转换自动触发设置
  stcAdcCfg.enAdcTrig1Sel = AdcTrigDisable;
  Adc_Init(&stcAdcCfg);
	
  ///< 通道0 P26
  stcAdcNormCfg.enAdcNormModeCh = AdcExInputCH1;
  stcAdcNormCfg.bAdcResultAccEn = FALSE;
  Adc_ConfigNormMode(&stcAdcCfg, &stcAdcNormCfg);	
}

void GpioInit(void)
{
  stc_gpio_cfg_t stcGpioCfg;
    
  //< 打开GPIO外设时钟门控
  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 

  stcGpioCfg.enDir = GpioDirOut;
  stcGpioCfg.enPu = GpioPuDisable;
  stcGpioCfg.enPd = GpioPdDisable;
	stcGpioCfg.enDrv = GpioDrvH;
	stcGpioCfg.enOD = GpioOdEnable;//打开开漏
	
	Gpio_Init(GpioPort2, GpioPin4, &stcGpioCfg);
	Gpio_Init(GpioPort2, GpioPin3, &stcGpioCfg);
	Gpio_Init(GpioPort1, GpioPin4, &stcGpioCfg);
	Gpio_Init(GpioPort3, GpioPin4, &stcGpioCfg);
	Gpio_WriteOutputIO(GpioPort2, GpioPin4, 1);
	Gpio_WriteOutputIO(GpioPort2, GpioPin3, 1);
	Gpio_WriteOutputIO(GpioPort1, GpioPin4, 1);
	Gpio_WriteOutputIO(GpioPort3, GpioPin4, 1);
	
	stcGpioCfg.enPu = GpioPuEnable;//开启上拉
	
	Gpio_Init(GpioPort2, GpioPin6, &stcGpioCfg);
	Gpio_Init(GpioPort3, GpioPin2, &stcGpioCfg);
	Gpio_Init(GpioPort3, GpioPin3, &stcGpioCfg);
	Gpio_WriteOutputIO(GpioPort2, GpioPin6, 1);
	Gpio_WriteOutputIO(GpioPort3, GpioPin2, 1);
	Gpio_WriteOutputIO(GpioPort3, GpioPin3, 1);

	stcGpioCfg.enDir = GpioDirIn;
	Gpio_Init(GpioPort3, GpioPin3, &stcGpioCfg);
    
  stcGpioCfg.enOD = GpioOdDisable;
  Gpio_Init(GpioPort2, GpioPin5, &stcGpioCfg);  
  
  Gpio_SetAnalogMode(GpioPort2, GpioPin6);
	App_AdcInit();
}


// 预设各通道中间值
const uint16_t adc_key_value[9] = {950, 1150, 1300, 1500, 1650, 1800, 2050, 3072, 3328};

uint16_t u16AdcResult = 0;
uint8_t  adc_key_index = 0;
uint16_t adc_key_cnt[9] = {0};   // PAD0~PAD8 按键计数器
uint8_t  adc_key_flag[9] = {0};  // 最终按键状态标志


/* 最终键值映射 */
void update_key_result(void)
{
  key_all.key0 = adc_key_flag[0];
  key_all.key1 = adc_key_flag[1];
  key_all.key2 = adc_key_flag[2];
  key_all.key3 = adc_key_flag[3];
  key_all.key4 = adc_key_flag[4];
  key_all.key5 = adc_key_flag[5];
  key_all.key6 = adc_key_flag[6];
   // 可扩展
}

void TouchKey_Scan(void)
{ // 启动一次ADC采样
  Adc_Start();
  while(FALSE != Adc_PollBusyState());
  Adc_GetResult(&u16AdcResult);
  uint8_t matched_key = 0xFF;

  // 判断按键值
  for(uint8_t i = 0; i < 9; i++)
  {
    if( (u16AdcResult > (adc_key_value[i] - KEY_ADC_THD)) 
      &&(u16AdcResult < (adc_key_value[i] + KEY_ADC_THD)))
    {
      matched_key = i;
      break;
    }
  }
  
  // 累加有效按键
  for(uint8_t i = 0; i < 9; i++)
  {
    if (i == matched_key)
    {
      if (adc_key_cnt[i] < KEY_COUNT_MAX)
        adc_key_cnt[i]++;
    }
    else
    {
      adc_key_cnt[i] = 0;
    }
    // 按键是否成立
    if (adc_key_cnt[i] >= KEY_COUNT_THD)
    {
      adc_key_flag[i] = 1;
    }
    else
    {
      adc_key_flag[i] = 0;
    }
  }
  update_key_result();  // 映射到结构体变量
}

