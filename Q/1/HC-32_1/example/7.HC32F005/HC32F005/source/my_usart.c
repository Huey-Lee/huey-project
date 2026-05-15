#include "my_usart.h"	
#include "ringfifo.h"
#include <stdlib.h>
#include "stdio.h"
#include "uart.h"
#include "gpio.h"
#include "BSP_MODBUS.h"
#include "uart_frame.h"

#define UART1_RX_RINGBUF_SIZE	64							//sizeof of the array
extern modbus__ mod;
ringfifo_t uart1_rx_ringfifo;									//define structure
uint8_t uart1_rx_ringfifo_buffer[UART1_RX_RINGBUF_SIZE]={0};//cache array

int fputc(int ch, FILE *f)
{
//	  Uart_SendDataPoll(M0P_UART0,ch);
	Uart_SendDataPoll(M0P_UART1,ch);
		return ch;
}

static void _UartBaudCfg(void)
{
    uint16_t timer=0;

    stc_uart_baud_cfg_t stcBaud;
    stc_bt_cfg_t stcBtCfg;

    DDL_ZERO_STRUCT(stcBaud);
    DDL_ZERO_STRUCT(stcBtCfg);

    //peripheral clock is enable
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBt,TRUE);//mode 0/2 can be disable
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);

    stcBaud.bDbaud  = 0u;				//double baudrate functon
    stcBaud.u32Baud = 2400u;		//update baudrate
    stcBaud.enMode  = UartMode1;//select mode
    stcBaud.u32Pclk = Sysctrl_GetPClkFreq(); //get the PCLK
    timer = Uart_SetBaudRate(M0P_UART1, &stcBaud);

    stcBtCfg.enMD = BtMode2;
    stcBtCfg.enCT = BtTimer;
    Bt_Init(TIM1, &stcBtCfg);		//call the function Bt_Init to generate the baudrate
    Bt_ARRSet(TIM1,timer);
    Bt_Cnt16Set(TIM1,timer);
    Bt_Run(TIM1);
}
void App_UartInit(void)
{
    stc_uart_cfg_t  stcCfg;

    _UartBaudCfg();

    stcCfg.enRunMode = UartMode1;//测试项，更改此处来转换4种模式测试
    Uart_Init(M0P_UART1, &stcCfg);

    ///< UART中断配置
    Uart_EnableIrq(M0P_UART1, UartRxIrq);
    Uart_ClrStatus(M0P_UART1, UartRC);
    EnableNvic(UART1_IRQn, IrqLevel3, TRUE);
}
//串口引脚配置
void App_PortInit(void)
{
    stc_gpio_cfg_t stcUartCfg;

    DDL_ZERO_STRUCT(stcUartCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE); //使能GPIO模块时钟
    ///<TX
    stcUartCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPort3, GpioPin5, &stcUartCfg);
    Gpio_SetAfMode(GpioPort3, GpioPin5, GpioAf1);          //配置P35 端口为URART1_TX
    ///<RX
    stcUartCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPort3, GpioPin6, &stcUartCfg);
    Gpio_SetAfMode(GpioPort3, GpioPin6, GpioAf1);          //配置P36 端口为URART1_RX
}
uint8_t uart_send(uint8_t *data, uint8_t len)// uart 发送信息，主要给 printMsg 函数调用
{
	for(uint8_t l=0;l<len;l++)
	{
		Uart_SendDataPoll(M0P_UART1,data[l]);
	}
	return len;
}

/***************************************************************************
*
*
*UART1	环形队列初始化
*
*
***************************************************************************/
void uart1_ringfifo_init(void)
{
	memset(uart1_rx_ringfifo_buffer,0,UART1_RX_RINGBUF_SIZE);																//串口1缓存数组清空
//	memset(uart1_tx_ringfifo_buffer,0,UART1_TX_RINGBUF_SIZE);
	ringfifo_init(&uart1_rx_ringfifo, uart1_rx_ringfifo_buffer, UART1_RX_RINGBUF_SIZE);			//环形队列初始化
//	ringfifo_init(&uart1_tx_ringfifo, uart1_tx_ringfifo_buffer, UART1_TX_RINGBUF_SIZE);
}

void MY_UART_INIT(void)
{
//	uart1_ringfifo_init();
//	modbus_init(ID_master,ID_slave);
	uart_frame_init();
	App_PortInit();
	App_UartInit();
}
//void uart_frame_loop(void)  //从接收队列中获取 UART 数据
//{
//	 modbus_command_handle(&mod);
//}
/***************************************************************************
*
*
*从缓存当中取出长度为len的数据放到buffer里面
*
*
***************************************************************************/
unsigned int uart1_rx_ringfifo_get(uint8_t* buffer, unsigned int len)
{
	unsigned int ret = 0;
	ret = ringfifo_get(&uart1_rx_ringfifo,buffer,len);//从缓存当中取出长度为len的数据放到buffer里面
	return ret;																				//返回读取的数据长度
}

unsigned int uart1_rx_ringfifo_dummy_get( unsigned int len)

{
	unsigned int ret = 0;
	ret = ringfifo_dummy_get(&uart1_rx_ringfifo,len);
	return ret;
}

unsigned int uart1_rx_ringfifo_check(void)
{
	int ret;
	ret = check_ringfifo_data(&uart1_rx_ringfifo);
	return ret;
}

unsigned int uart1_ringfifo_seek(const char *symbol)
{
	unsigned int result=0;
	result = ringfifo_seek(&uart1_rx_ringfifo, symbol);
	return result;
}
/***************************************************************************
*
*
*串口中断接收数据，并且把数据放入环形队列当中
*
*
***************************************************************************/
//void Uart1_IRQHandler(void)
//{
//	uint8_t reg;
//	
//    if(TRUE == Uart_GetStatus(M0P_UART1, UartRC))
//    {
//        Uart_ClrStatus(M0P_UART1, UartRC);

//        reg = Uart_ReceiveData(M0P_UART1);
//				if(ua.receive_st == 0)
//				{
//					if(reg == buf_start_master && !(ua.rx_sta & 0x8000))
//						ua.receive_st = 1;
//					else goto end;
//				}
//			
//				if((ua.rx_sta&0x8000)==0)//接收未完成
//				{
//					if(ua.rx_sta&0x4000)
//					{
//						if(reg!=buf_end_2)
//						{
//							ua.rx_sta &= ~0x4000;
//							ringfifo_putin(&uart1_rx_ringfifo,&reg,1);
//							ua.rx_sta++;
//						}
//						else 
//						{ua.rx_sta|=0x8000;	//接收完成了 
//							ua.receive_st = 0;
//						}
//					}
//					else 
//					{
//						if(reg==buf_end_1)ua.rx_sta|=0x4000;
//						else
//						{
//							ringfifo_putin(&uart1_rx_ringfifo,&reg,1);
//							ua.rx_sta++;
//						}
//					}
//				}
//				end : if((ua.rx_sta & 0x3fff) >= 20)
//				{
//					ua.receive_st = 0;			//开始接收标志清除
//					ua.rx_sta = 0;					//接收数据量清零
//				}
//    }
//}

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
	{
		String[Num_Length++] = '0';
	}
	
  while(Int_Num>0)
  {
    String[Num_Length++]=Int_Num%10+'0';
    Int_Num=Int_Num/10;
  }
  if(Negative_Flag)
  {
		String[Num_Length++]='-';
	}
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
	{
	        f_afterdot = f_afterdot*10;
	}
	i_afterdot = (unsigned long int)f_afterdot;

	n=dotsize;
	while(i_afterdot > 0 |n > 0)
	{
		n--;
	 	str[iSize++] = i_afterdot % 10 + '0';
	 	i_afterdot = i_afterdot / 10;
	}
        if(dotsize==0)
        {
          str[iSize++] = ' ';
        }
        else
        {
	str[iSize++] = '.';
        }
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
	      unsigned char *hex;
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
