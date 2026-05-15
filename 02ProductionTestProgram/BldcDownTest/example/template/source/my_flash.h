

#ifndef  __MY_FLASH_H__
#define  __MY_FLASH_H__


#include "ddl.h"
#include "adc.h"
#include "gpio.h"
#include "ddl.h"
#include "bgr.h"

/* 控制包获取内容 */
typedef struct storage_struct
{
	uint32_t Encoder_1;
	uint32_t Encoder_2;
	uint32_t Encoder_3;
	uint32_t Encoder_4;
}storage_structParam;
	
typedef union
{
	storage_structParam mstruct;
	uint8_t data8[sizeof(storage_structParam)];

}storage_structunion;



typedef struct _ee_param_t
{
  uint8_t sof;                      //起始字节
  uint8_t beep_status;              //2023.3.24
}ee_param_t;


extern ee_param_t ee_param;


extern storage_structunion storage_data;
void flash_read(void);
void flash_write(void);
void flash_init(void);
void save_param(void);
void read_beep_status(void);
#endif /* __DDL_DEVICE_H__ */


