#include "common.h"
#include "uart.h"
#include "uart_frame.h"

u8 uart_time_error;
static bit busy=0;
static u8 tx_flag=0;
void uart_init(void)   //串口初始化
{
	InitialUART0_Timer1(9600);
  InitialUART1_Timer3(4800);
  busy=0;
  //UART0和UART1优先级设置
//  clr_PSH;    //等级1
//  set_PS;
//  clr_PSH_1;  //等级1
//  set_PS_1;
  //中断使能
  set_ES_1;				
	set_ES;
}


//unsigned char Receive_Data(unsigned char UARTPort)
//{
//  UINT8 c;
//  switch (UARTPort)
//  {
//    case UART0:
//      while (!RI);
//      c = SBUF;
//      RI = 0;
//      break;
//    case UART1:
//      while (!RI_1);
//      c = SBUF_1;
//      RI_1 = 0;
//      break;
//  }
//  return (c);
//}

void UART_Send_Data(UINT8 UARTPort, UINT8 c)
{
  u32 timeout=0xffff;
  switch (UARTPort)
  {
    case UART0:
      TI = 0;
      SBUF = c;
      while(TI==0);
      break;
    case UART1:
      tx_flag=1;
      SBUF_1 = c;
      while(tx_flag==1)
      {
        if(timeout){timeout--;}
        else{break;}
      }
      break;
  }
}


//void UART_Send_Value(UINT8 UARTPort, UINT16 val)
//{
//	UART_Send_Data(UARTPort,(val/10000)+0x30);
//	UART_Send_Data(UARTPort,((val%10000)/1000)+0x30);
//	UART_Send_Data(UARTPort,((val%1000)/100)+0x30);
//	UART_Send_Data(UARTPort,((val%100)/10)+0x30);
//	UART_Send_Data(UARTPort,(val%10)+0x30);
//}


void UART_Send_Buf(UINT8 UARTPort, UINT8 *ptr,UINT8 len) 
{
  u8 i;
  switch (UARTPort)
  {
    case UART0:
      for(i=0;i<len;i++)
      {
        busy = 1;
        SBUF = ptr[i];
        while (busy);
      }
      break;
    case UART1:
	    for(;len>0;len--)
      {
        UART_Send_Data(UART1,*ptr);
        ptr++;
      }
      break;
  }
}


void UART0_ISR(void) interrupt 4         //蓝牙模组
{    
  u8 dat;
  if (RI)
  {
    clr_RI; // Clear RI (Receive Interrupt).
    dat = SBUF;
    UART0_Enter_queue(&UART0_rx_queue,dat);
  }
  if (TI)
  { 
    clr_TI; 
    busy = 0; 
  }
}

void UART1_ISR(void) interrupt 15         //下控通讯
{ 
  u8 dat;
  if (RI_1==1) 
  {                                       /* if reception occur */
    clr_RI_1;                             /* clear reception flag for next reception SBUF_1 */
    dat = SBUF_1;
    uart_time_error=0;
    UART1_Enter_queue(&UART1_rx_queue,dat);
  }
  if(TI_1==1)
  {
    tx_flag=0;
    clr_TI_1;                             /* if emission occur */
  }  
}
