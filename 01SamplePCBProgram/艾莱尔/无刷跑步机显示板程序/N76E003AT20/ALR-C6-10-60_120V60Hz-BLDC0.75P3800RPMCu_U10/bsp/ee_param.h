/*
 * param.h
 *
 *  Created on: 2020年3月8日
 *      Author: Administrator
 */

#ifndef BSP_PARAM_H_
#define BSP_PARAM_H_
#include "common.h"
#include "iap_eeprom.h"

#define PARAM_SOF (0xA5)

#define KIV_NUM        12
#define EEPROM_BYTES   2 //31   //2023.3.24

typedef struct _ee_param_t
{
  u8 sof;                      //起始字节
  u8 beep_status;              //2023.3.24
  u8 mile_status;              //2023.3.24
  u8 key_add1;
  u8 key_add2;
}ee_param_t;


extern ee_param_t ee_param;
extern void read_param(void);//u8 read_param(void);
extern void write_param(void);//(s8 val);
extern void ee_param_init(void);

#endif /* BSP_PARAM_H_ */
