#include "common.h"
#include "uart_frame.h"
#include "queue.h"
#include "treadmills.h"
#include "decode.h"

UINT8  ms10_flag=0;
UINT8  ms100_flag=0;
UINT8  ms250_flag=0;
UINT8  ms500_flag=0;
UINT8  ms1000_flag=0;

void Timer0_ISR(void) interrupt 1        // Vector @  0x0B
{
	static UINT16 cnt1=0;
	static UINT16 cnt2=0;
	static UINT16 cnt3=0;
	static UINT16 cnt4=0;
	static UINT16 cnt5=0;
  
  TH0 = (65536-76)/256;   
	TL0 = (65536-76)%256;   //TIMER_DIV12_VALUE_60us  Êµ²â
  
  RF433M_RecevieDecode();
	
  cnt4++;
	if(cnt4>167)//10ms=166
	{
    cnt4=0;
    ms10_flag=1;
	}
  
	cnt1++;
	if(cnt1>1667)//100ms=1667
	{
    cnt1=0;
    ms100_flag=1;
	}
	
	cnt2++;
	if(cnt2>4180)//250ms=4180
	{
		cnt2=0;
		ms250_flag=1;
	}
	
  cnt5++;
	if(cnt5>8340)//500ms=8340
	{
		cnt5=0;
		ms500_flag=1;
	}
  
	cnt3++;
	if(cnt3>16600)//1000ms=16600
	{
    cnt3=0;
    ms1000_flag=1;
	}
}

