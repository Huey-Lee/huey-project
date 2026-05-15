/******************************************************************************
* Copyright (C) 2017, Xiaohua Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Xiaohua Semiconductor Co.,Ltd ("XHSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with XHSC
* components. This software is licensed by XHSC to be adapted only
* for use in systems utilizing XHSC components. XHSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. XHSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* XHSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* XHSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/
/******************************************************************************/
/** \file main.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2017-05-17  1.0  cj First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "uart.h"
#include "bt.h"
#include "gpio.h"
#include "pca.h"
#include "my_usart.h"
#include "MY_TIM.h"
#include "MY_ADC.h"
#include "point_fun.h"
#include "MY_GPIO.h"
#include "uart_frame.h"
#include "motor_speed_pid.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define     T1_PORT                 (3)
#define     T1_PIN                  (3)
/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
u8 waitforamoment;
static u16 wait_time_start;
static void wait_for_motor(void)
{
	if(waitforamoment == 1)
	{
		wait_time_start++;
		delay10us(1);
		if(wait_time_start>20)
		{
			wait_time_start = 0;
			waitforamoment = 0;
			ctr.status = STATUS_TO_RUN;
		}
	}
}
u32 base_time=0;//62000	//without interrupt 500ms
								//20300	//with the motor on 500ms
//u8 flag;
u8 uart_time_error=0;
void _500ms_time_handle(void)
{
		base_time++;
		if(base_time>=61000&&(motor.status==NULL||motor.status==STATUS_MT_START))
		{
			base_time=0;

			uart_time_error++;
		}
		else if(base_time>=20300&&(motor.status!=NULL&&motor.status!=STATUS_MT_START))
		{
			base_time=0;

			uart_time_error++;
		}
		if(uart_time_error>5)//2.5S
		{
			uart_time_error=6;
      ctr.error_code=ERROR_01;
      ctr.status=STATUS_ERROR;
		}
}

/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
static void GpioInit(void)
{
		stc_gpio_cfg_t 		stcGpioCfg;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    Gpio_SetAnalogMode(GpioPort2, GpioPin4);
    Gpio_SetAnalogMode(GpioPort2, GpioPin6);
    Gpio_SetAnalogMode(GpioPort3, GpioPin2);

    stcGpioCfg.enDir = GpioDirOut;

    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdEnable;
		Gpio_Init(STK_LED_PORT, STK_LED_PIN, &stcGpioCfg);
}

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
extern tim_t tim_handle;
extern u16 timestest;
extern u8 valtage;
int32_t main(void)
{
	App_ClkInit();

	GpioInit();
	MY_UART_INIT();
	MY_TIM_INIT();
	MY_ADC_INIT();
	MY_GPIO_INIT();
	MAIN_INIT(100);
	while(1)
	{
		_500ms_time_handle();
		
		if(tim_handle.tim6_cur >= tim6_change_speed_time)
		{
//			printf("status:%d,sta:%d,val:%d\r\n",motor.status,motor.start_set_val,valtage);
//			printf("kiv_v:%d\r\n",motor.adjust_kiv_voltage);
//			printf("timestest:%d\r\n",timestest);
			timestest=0;
			tim_handle.tim6_cur=0;
			motor_speed();
			error_chick();
		}
		uart_frame_loop();
		
		ctr_proc_loop();
		
		wait_for_motor();
		
		communication_checkout();	
	}
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


