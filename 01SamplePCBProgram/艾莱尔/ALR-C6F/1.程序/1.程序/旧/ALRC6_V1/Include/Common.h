#ifndef __COMMON_HH_
#define __COMMON_HH_

#include "N76E003.h"
#include "Delay.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "cfg.h"


typedef bit                   BIT;
typedef unsigned char         UINT8;
typedef unsigned int          UINT16;
typedef unsigned long         UINT32;

typedef unsigned char         uint8_t;
typedef unsigned int          uint16_t;
typedef unsigned long         uint32_t;


#define     CID_READ				0x0B
#define     DID_READ				0x0C

#define     ERASE_APROM				0x22
#define     READ_APROM				0x00
#define     PROGRAM_APROM			0x21
#define     ERASE_LDROM				
#define     READ_LDROM				
#define     PROGRAM_LDROM			
#define     READ_CFG					0xC0
#define     PROGRAM_CFG				0xE1
#define			READ_UID					0x04


void  InitialUART0_Timer1(UINT32 u32Baudrate); //T1M = 1, SMOD = 1
void  InitialUART0_Timer3(UINT32 u32Baudrate); //Timer3 as Baudrate, SMOD=1, Prescale=0
void  InitialUART1_Timer3(UINT32 u32Baudrate);
extern void SW_Reset(void);
extern bit BIT_TMP;


////---------------ADD BY BRANTYU -------------------------

#define  __IO  volatile
typedef unsigned char  u8;
typedef char  s8;
typedef unsigned short u16;
typedef short s16;
typedef unsigned long  u32;

//typedef char  int8;
//typedef short int16;
//typedef long  int32;

typedef __IO u8   vu8;
typedef __IO u16  vu16;




#define NOP_1()  {_nop_();}
#define NOP_2()  {NOP_1();NOP_1();}
#define NOP_3()  {NOP_1();NOP_2();}
#define NOP_5()  {NOP_2();NOP_3();}
#define NOP_10()  {NOP_5();NOP_5();}
#define NOP_15()  {NOP_10();NOP_5();}
#define NOP_20()  {NOP_10();NOP_10();}


#define EA_DISABLE(); {EA=0;}
#define EA_ENABLE();  {EA=1;}

#define IDEL  (0)

#if(0)

#else
	#define debug_printf(x)  /*x*/

	#define debug_printf_d(v) /*v*/


#define debug_printf_s(x)  
#define debug_printf_de(v) 
#endif

#endif







