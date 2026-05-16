/*
 * indecate.h
 *
 *  Created on: 2018年8月1日
 *      Author: Administrator
 */

#ifndef __indecate__HH
#define __indecate__HH
#include "common.h"
#include "aip1628.h"

#define  set_dp0()  {indecate.seg[0] |=(1<<7);indecate_update_flag=1;}    //小数点
#define  set_dp1()  {indecate.seg[1] |=(1<<7);indecate_update_flag=1;}    //小数点
#define  set_dp2()  {indecate.seg[2] |=(1<<7);indecate_update_flag=1;}    //小数点
#define  set_dp3()  {indecate.seg[3] |=(1<<7);indecate_update_flag=1;}    //小数点

#define  clr_dp0()  {indecate.seg[0] &= ~(1<<7);indecate_update_flag=1;} 
#define  clr_dp1()  {indecate.seg[1] &= ~(1<<7);indecate_update_flag=1;} 
#define  clr_dp2()  {indecate.seg[2] &= ~(1<<7);indecate_update_flag=1;} 
#define  clr_dp3()  {indecate.seg[3] &= ~(1<<7);indecate_update_flag=1;} 


typedef struct _indecate_t
{
	u8 seg[4];

//	u8 led9:1;   停止
//	u8 led10:1;
//	u8 led21:1;
//	u8 led31:1;  运行
//	u8 led30:1;
//	u8 led40:1;  停止
//	u8 led41:1;  运行
//	u8 :1;
  
//位域示例：
//u8 a:1; //a占字节的bit0
//u8 b:2; //b占字节的bit2- bit1
//u8 c:5; //c占字节的bit7- bit3
  
	u8 led_time:1;
	u8 led_distance:1;
	u8 led_calorie:1;
	u8 led_speed:1;
	u8 led_step:1;
	u8 led_updp:1;
	u8 led_downdp:1;
	u8 :1;

}indecate_t;

extern  indecate_t   indecate;

extern u8  indecate_update_flag;

#define set_light_scale(scale)  {set_aip1628_lighte_scale(scale);}  //设置亮度等级
extern void disp_update(void);

#endif /* SOFTWARE_BSP_INDECATE_H_ */