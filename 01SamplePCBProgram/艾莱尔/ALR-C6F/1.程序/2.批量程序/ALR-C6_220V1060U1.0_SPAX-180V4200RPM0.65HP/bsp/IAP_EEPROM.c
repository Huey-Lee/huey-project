/*
 * IAP_EEPROM.c
 *
 *  Created on: 2020年3月8日
 *      Author: Administrator
 */
#include "common.h"
#include "uart.h"
#include "iap_eeprom.h"

//Erase APROM DATAFLASH page
void erase_aprom_page(UINT8 page)
{
	UINT16 u16_addr=FLASH_ADDR_BASE + (0x80<<page);
	IAPAL = u16_addr&0xff;
	IAPAH = ((u16_addr)>>8)&0xff;
	IAPFD = 0xFF;
	set_IAPEN;
	set_APUEN;
	IAPCN = 0x22;
	set_IAPGO;
}


void write_aprom_page(UINT8 page, UINT8 *buf, UINT8 len)  
{
UINT8 i;
// bit BIT_TMP;//~2.2
UINT16 u16_addr=FLASH_ADDR_BASE +(0x80<<page);//(0x80<<page);/2023.1.30 //数组写 //(page)->(0x80<<page) 2023.2.6
set_IAPEN;
set_APUEN;
  
//BIT_TMP=EA; //2.2~~~~~~
//EA=0;
  
 IAPCN = 0x21;
 for(i=0;i<len;i++)
 {
   IAPAL = (u16_addr&0xff)+i;
   IAPAH = (u16_addr>>8)&0xff;
   IAPFD = buf[i];
   set_IAPGO;
 }
 

 
 clr_APUEN;
 clr_IAPEN;
 
// EA = BIT_TMP;//2.2~~~
 
}



void read_aprom_page(UINT8 page, UINT8 *buf, UINT8 len)
{
 UINT8 i;
 UINT8 code *ptr=FLASH_ADDR_BASE +(0x80<<page);//(0x80<<page);//2023.1.30，数组读 //(page)->(0x80<<page);2023.2.6
  for(i=0;i<len;i++)
  {
	  buf[i] = ptr[i];
  }
}




/**
 * 返回值:成功 1,失败0
 */
UINT8 test_iaprom(void)
{
#if(0)
	UINT8 i;
	UINT8 buf[128];
	erase_aprom_page(0);
	for(i=0;i<128;i++)
	{
	buf[i]=i;
	}
	write_aprom_page(0,&buf[0],128);
	for(i=0;i<128;i++)
	{
	buf[i]=0;
	}
	read_aprom_page(0,&buf[0],128);

	for(i=0;i<128;i++)
	{
		debug_printf("\r\n[");
		debug_printf_d(i);
		debug_printf("]= ");
		debug_printf_d(buf[i]);
	}

	for(i=0;i<128;i++)
	{
		if(buf[i] !=i)
		{
			debug_printf("\r\n test fail\r\n");
			return 0;
			//break;
		}
	}

	debug_printf("\r\n test successful\r\n");
	return 1;
#else
	return 0;
#endif
	
}
