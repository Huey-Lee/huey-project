#ifndef _BSP_MODBUS_H
#define _BSP_MODBUS_H
#include "hc32f005.h"

#define MODBUS_EN 0
/******************************************************
*
*
*全局变量
*
*
******************************************************/
#define ID_master 0xFE
#define ID_slave  0xFE
#define REGISTERNUM     4
#define modbus_buflen   32 										//缓存大小

typedef struct{
	uint8_t master_id;
	uint8_t salver_id;
	uint8_t start;						//modbus一帧数据开始接收
	uint8_t end;							//modbus一帧数据接收完成标志
	
	uint8_t funcode;
	
	uint16_t startreg;
	uint8_t start_reg[2];
	
	uint16_t regnum;
	uint8_t reg_num[2];
	
	uint8_t Bitnum;
	
	uint16_t *registe;
	uint8_t  *buf;
	
	uint16_t countcrc;
	uint8_t count_crc[2];
	uint8_t master_crc[2];
	uint8_t salver_crc[2];
	
	uint8_t number;
}modbus__;


/******************************************************
*
*
*函数申明
*
*
******************************************************/
void u16_to_u8(uint8_t * data,uint16_t * buf,uint8_t u16len);
void u8_to_u16(uint8_t * data,uint16_t * buf,uint8_t u16len);
void modbus_init(uint8_t masterid,uint8_t salverid);
uint8_t modbus_command_handle(modbus__ * modbus);
void modbus_register_handle(void);
unsigned int Crc16(unsigned char *pBuf, unsigned char num);
void modbus_back_message(modbus__ * modbus);

#endif
