#ifndef RF433MDecode_H_
#define RF433MDecode_H_
#include "common.h"
/***************************************************/
//Define I/O Register
 
#define  RF_REC      (P00)  //P06


//在60us定时器中断中调用433M接收解码函数

//同步码0.3ms高电平 + 10ms低电平   取8~14

#define RF_SYN_H_MIN    2     //同步码高电平下限
#define RF_SYN_H_MAX    10    //同步码高电平上限

#define RF_SYN_L_MIN    135     //8MS
#define RF_SYN_L_MAX    250     //14MS 


//同步码后24位，3个字节
#define RF_REC_BIT_LEN    24    
 
//"1" 1.2ms高电平+0.4ms低电平, "0" 0.4ms高电平+1.2ms低电平 
//所以可以只通过高电平来判断
 
//bit 1
#define RF_DATA1_MAX      24    //1.4ms  1.5
#define RF_DATA1_MIN      14    //0.8ms  0.9
 
//bit 0
#define RF_DATA0_MAX      10    //0.6ms  0.6
#define RF_DATA0_MIN      2     //0.2ms  0.3
 

extern u8 key_value; 

extern void RF433M_Init(void);
extern void RF433M_RecevieDecode(void);
#endif
