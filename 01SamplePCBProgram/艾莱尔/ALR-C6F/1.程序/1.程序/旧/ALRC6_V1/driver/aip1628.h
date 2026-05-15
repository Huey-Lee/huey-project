/*
 * tm1668.h
 *
 *  Created on: 2018年8月1日
 *      Author: Administrator
 */
#ifndef __aip1628__HH
#define __aip1628__HH
#include "common.h"


/**
 * 初始化指令
*/
#define DISP_MODLE_CMD (0X00)
#define B4S13          (0X00)   /*4位13段*/
#define B5S12          (0X01)   /*5位12段*/
#define B6S11          (0X02)   /*6位11段*/
#define B7S10          (0X03)   /*7位10段*/

#define B5S9           (0X01)   /*5位9段*/

#define DATA_CMD       (0X40)
#define WT_DISP_REG	   (0X00)
#define RD_KEY_REG     (0X02)
#define ADDR_AUTO_INC	 (0X00)
#define ADDR_FIX	     (0X04)

#define DISP_CTR_CMD   (0X80)
#define EXTINCTION_SCALE (5) /*0~7*/
#define DISP_ON        (0X08)
#define DISP_OFF       (0X00)

#define ADDR_CMD       (0XC0)

/**
 * 六、 显示寄存器地址：
该寄存器存储通过串行接口接收从外部器件传送到TM1668的数据,最多有效地址从C0H-CDH共14字节单元,
分别与芯片SGE和GRID管脚对应,具体分配如图(2)：
写LED显示数据的时候,按照显示地址从低位到高位,数据字节从低位到高位操作。
SEG1
SEG2
SEG3
SEG4
SEG5
SEG6
SEG7
SEG8
SEG9
SEG10
X
SEG12
SEG13
SEG14
X X
xxHL（低四位） xxHU(高四位) xxHL(低四位) xxHU（高位）
B0 B1 B2 B3 B4 B5 B6 B7 B0 B1 B2 B3 B4 B5 B6 B7
C0HL C0HU C1HL C1HU GRID1
C2HL C2HU C3HL C3HU GRID2
C4HL C4HU C5HL C5HU GRID3
C6HL C6HU C7HL C7HU GRID4
C8HL C8HU C9HL C9HU GRID5
CAHL CAHU CBHL CBHU GRID6
CCHL CCHU CDHL CDHU GRID7

 芯片显示寄存器在上电瞬间其内部保存的值可能是随机不确定的,此时客户直接发送开屏命令,
将有可能出现显示乱码。所以我司建议客户对显示寄存器进行一次上电清零操作,即上电后向14位显存地址
（C0H-CDH）中全部写入数据0x00。
 */

extern void set_aip1628_lighte_scale(u8 scale);
extern void aip1628_write_byte(u8 addr,u8 dat);
extern void aip1628_init(void);
extern void aip1628_write_nbytes(u8 addr,u8 *p, u8 len);

#endif /* SOFTWARE_DRIVER_TM1688_H_ */
