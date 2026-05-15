#include "MY_PCA.h"
#include "gpio.h"
void MY_PCA_INIT(void)
{
	App_PcaInit();
	///< PCA 运行
	Pca_StartPca(TRUE);
}
/**
 ******************************************************************************
 ** \brief  配置PCA
 **
 ** \return 无
 ******************************************************************************/
void App_PcaInit(void)
{
    stc_pcacfg_t  PcaInitStruct;
    
    //使能PCA外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralPca, TRUE);
    
    PcaInitStruct.pca_clksrc = PcaTim0ovf;
    PcaInitStruct.pca_cidl   = FALSE;
    PcaInitStruct.pca_ecom   = PcaEcomEnable;        //允许比较器功能
    PcaInitStruct.pca_capp   = PcaCappDisable;        //禁止上升沿捕获
    PcaInitStruct.pca_capn   = PcaCapnDisable;        //禁止下降沿捕获
    PcaInitStruct.pca_mat    = PcaMatEnable;        //禁止匹配功能
    PcaInitStruct.pca_tog    = PcaTogDisable;        //禁止翻转控制功能
    PcaInitStruct.pca_pwm    = PcaPwm8bitDisable;    //使能PWM控制输出

    PcaInitStruct.pca_ccap   = 10000;
    Pca_M4Init(&PcaInitStruct);   

    Pca_Set4Wdte(TRUE); ///< 看门狗使能
}

