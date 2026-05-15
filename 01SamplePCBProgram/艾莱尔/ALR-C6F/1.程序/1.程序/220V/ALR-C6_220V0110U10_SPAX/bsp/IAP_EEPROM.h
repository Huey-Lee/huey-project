/*
 * IAP_EEPROM.h
 *
 *  Created on: 2020Äê3ÔÂ8ÈÕ
 *      Author: Administrator
 */

#ifndef BSP_IAP_EEPROM_H_
#define BSP_IAP_EEPROM_H_
#include "common.h"

#define FLASH_ADDR_BASE  0x4500//0x3800   2023.2.17

#define PAGE_SIZE (0X128)  //2023.1.30
#define PAGE0 (0)

extern void erase_aprom_page(UINT8 page);
extern void write_aprom_page(UINT8 page, UINT8 *buf, UINT8 len);
extern void read_aprom_page(UINT8 page, UINT8 *buf, UINT8 len);

extern UINT8 test_iaprom(void);
#endif /* BSP_IAP_EEPROM_H_ */
