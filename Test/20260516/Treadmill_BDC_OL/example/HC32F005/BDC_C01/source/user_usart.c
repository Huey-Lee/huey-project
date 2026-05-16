/*
 * Function: UART1 初始化、GPIO 映射、波特率配置及打印重定向与工具函数。
 * Method:   BT1 产生波特时钟；Uart_Init + RX 中断；uart1_rx_ringfifo 独立缓冲；fputc 走轮询发送；提供整型/浮点/十六进制工具与 ringfifo 封装。
 * Name:     Huey
 * Date:     May 16, 2026 18:00
 */

#include "user_usart.h"
#include "ringfifo.h"
#include <stdlib.h>
#include <string.h>
#include "stdio.h"
#include "uart.h"
#include "gpio.h"
#include "uart_frame.h"

#define UART1_RX_RINGBUF_SIZE	64

ringfifo_t uart1_rx_ringfifo;
uint8_t uart1_rx_ringfifo_buffer[UART1_RX_RINGBUF_SIZE]={0};

int fputc(int ch, FILE *f)
{
	(void)f;
	Uart_SendDataPoll(M0P_UART1,(uint8_t)ch);
	return ch;
}

static void _UartBaudCfg(void)
{
    uint16_t timer=0;

    stc_uart_baud_cfg_t stcBaud;
    stc_bt_cfg_t stcBtCfg;

    DDL_ZERO_STRUCT(stcBaud);
    DDL_ZERO_STRUCT(stcBtCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt,TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);

    stcBaud.bDbaud  = 0u;
    stcBaud.u32Baud = 2400u;
    stcBaud.enMode  = UartMode1;
    stcBaud.u32Pclk = Sysctrl_GetPClkFreq();
    timer = Uart_SetBaudRate(M0P_UART1, &stcBaud);

    stcBtCfg.enMD = BtMode2;
    stcBtCfg.enCT = BtTimer;
    Bt_Init(TIM1, &stcBtCfg);
    Bt_ARRSet(TIM1,timer);
    Bt_Cnt16Set(TIM1,timer);
    Bt_Run(TIM1);
}

void App_UartInit(void)
{
    stc_uart_cfg_t  stcCfg;

    _UartBaudCfg();

    stcCfg.enRunMode = UartMode1;
    Uart_Init(M0P_UART1, &stcCfg);

    Uart_EnableIrq(M0P_UART1, UartRxIrq);
    Uart_ClrStatus(M0P_UART1, UartRC);
    EnableNvic(UART1_IRQn, IrqLevel3, TRUE);
}

void App_PortInit(void)
{
    stc_gpio_cfg_t stcUartCfg;

    DDL_ZERO_STRUCT(stcUartCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    stcUartCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPort3, GpioPin5, &stcUartCfg);
    Gpio_SetAfMode(GpioPort3, GpioPin5, GpioAf1);
    stcUartCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPort3, GpioPin6, &stcUartCfg);
    Gpio_SetAfMode(GpioPort3, GpioPin6, GpioAf1);
}

uint8_t uart_send(uint8_t *data, uint8_t len)
{
	for(uint8_t l=0;l<len;l++)
		Uart_SendDataPoll(M0P_UART1,data[l]);
	return len;
}

void uart1_ringfifo_init(void)
{
	memset(uart1_rx_ringfifo_buffer,0,UART1_RX_RINGBUF_SIZE);
	ringfifo_init(&uart1_rx_ringfifo, uart1_rx_ringfifo_buffer, UART1_RX_RINGBUF_SIZE);
}

void user_usart_init(void)
{
	uart_frame_init();
	App_PortInit();
	App_UartInit();
}

unsigned int uart1_rx_ringfifo_get(uint8_t* buffer, unsigned int len)
{
	return ringfifo_get(&uart1_rx_ringfifo,buffer,len);
}

unsigned int uart1_rx_ringfifo_dummy_get( unsigned int len)
{
	return ringfifo_dummy_get(&uart1_rx_ringfifo,len);
}

unsigned int uart1_rx_ringfifo_check(void)
{
	return (unsigned int)check_ringfifo_data(&uart1_rx_ringfifo);
}

unsigned int uart1_ringfifo_seek(const char *symbol)
{
	return ringfifo_seek(&uart1_rx_ringfifo, symbol);
}

int Int_To_String(signed long Int_Num,unsigned char String[])
{
  signed int     Num_Length    = 0;
  signed char    Negative_Flag = 0;
  unsigned char *Point_p       = String;
  unsigned char *Point_s       = String;

  if(Int_Num<0)
  {
    Negative_Flag = 1;
    Int_Num       = 0 - Int_Num;
  }
  if(Int_Num == 0)
		String[Num_Length++] = '0';

  while(Int_Num>0)
  {
    String[Num_Length++]=Int_Num%10+'0';
    Int_Num=Int_Num/10;
  }
  if(Negative_Flag)
		String[Num_Length++]='-';

  String[Num_Length]='\0';
  Point_p=String+Num_Length-1;

  for(;Point_p-Point_s>0;Point_p--,Point_s++)
  {
    *Point_s^=*Point_p;
    *Point_p^=*Point_s;
    *Point_s^=*Point_p;
  }
  return(Num_Length);
}

int Float_To_String(float fNum, unsigned char str[])
{
	int dotsize = 2;
	int iSize = 0;
	int n = 0;
	unsigned char *p = str;
	unsigned char *s = str;
	char isnegative = 0;
	unsigned long int i_predot;
	unsigned long int i_afterdot;
	float f_afterdot;

	if(fNum < 0)
	{
		isnegative = 1;
		fNum = 0 - fNum;
	}

	i_predot = (unsigned long int)fNum;
	f_afterdot = fNum - i_predot;

	for(n = dotsize;n > 0;n--)
	        f_afterdot = f_afterdot*10;
	i_afterdot = (unsigned long int)f_afterdot;

	n=dotsize;
	while(i_afterdot > 0 || n > 0)
	{
		n--;
	 	str[iSize++] = i_afterdot % 10 + '0';
	 	i_afterdot = i_afterdot / 10;
	}
        if(dotsize==0)
          str[iSize++] = ' ';
        else
	str[iSize++] = '.';

	if(i_predot == 0)
		str[iSize++] = '0';

	while(i_predot > 0)
	{
	 	str[iSize++] = i_predot%10 + '0';
	 	i_predot = i_predot / 10;
	}

	if(isnegative == 1)
	str[iSize++] = '-';
	str[iSize] = '\0';

	p = str + iSize - 1;

	for( ;p - s > 0;p--,s++)
	{
		*s^=*p;
		*p^=*s;
		*s^=*p;
	}

	return iSize;
}


unsigned char * str_hex(unsigned char *str)
{
	      unsigned char hex_buf[128];
	      unsigned char *hex = hex_buf;
        unsigned char ctmp, ctmp1,half;
        unsigned int num=0;
        do{
                do{
                    half = 0;
                    ctmp = *str;
                    if(!ctmp) break;
                    str++;
                }while((ctmp == 0x20)||(ctmp == 0x2c)||(ctmp == '\t'));
                if(!ctmp) break;
                if(ctmp>='a') ctmp = ctmp -'a' + 10;
             else if(ctmp>='A') ctmp = ctmp -'A'+ 10;
             else ctmp=ctmp-'0';
                ctmp=ctmp<<4;
                half = 1;
                ctmp1 = *str;
                if(!ctmp1) break;
                str++;
                if((ctmp1 == 0x20)||(ctmp1 == 0x2c)||(ctmp1 == '\t'))
                {
                        ctmp = ctmp>>4;
                        ctmp1 = 0;
                }
           else if(ctmp1>='a') ctmp1 = ctmp1 - 'a' + 10;
             else if(ctmp1>='A') ctmp1 = ctmp1 - 'A' + 10;
             else ctmp1 = ctmp1 - '0';
             ctmp += ctmp1;
                *hex = ctmp;
                hex++;
                num++;
         }while(1);
         if(half)
         {
                ctmp = ctmp>>4;
                *hex = ctmp;
                num++;
         }
         return(hex);
}
