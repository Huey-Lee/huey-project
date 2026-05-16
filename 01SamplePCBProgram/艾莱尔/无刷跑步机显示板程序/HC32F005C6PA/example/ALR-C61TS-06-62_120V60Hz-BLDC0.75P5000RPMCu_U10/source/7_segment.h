/*
 * 7_segment.h
 *
 *  Created on: 2018年8月1日
 *      Author: Administrator
 */

#ifndef __7_SEGMENT__H__
#define __7_SEGMENT__H__
#include "common.h"


/**
 *       当前数码管的配置情况
 *         ---a0----
 *        |         |
 *        f6        b1
 *        |         |
 *        |---g5----|
 *        |         |
 *        e4        c2
 *        |         |
 *         ---d3----   .dp(nc)
 *
 */

#define a   (1<<1)
#define b   (1<<2)
#define c   (1<<3)
#define d   (1<<4)
#define e   (1<<5)
#define f   (1<<0)
#define g   (1<<6)


#define SEG_ALL_ON  (0XFF)
#define SEG_ALL_OFF (0x00)

#define SEG__       (g)
#define SEG_E       (a|f|g|e|d)
#define SEG_R       (g|e)
#define SEG_P       (a|b|e|f|g)     //显示P

#define SEG_UP_     (a)
#define SEG_DWN_    (d)

#define NUM_0		    (a|b|c|d|e|f)		//0
#define NUM_1		    (b|c)//1
#define NUM_2       (a|b|g|e|d)			//2
#define NUM_3       (a|b|c|d|g)			//3
#define NUM_8       (a|b|c|d|e|f|g)	//8

#define SEG_O       (a|b|c|d|e|f)   //O     //2023.3.24
#define SEG_N       (a|b|c|e|f)     //N n
#define SEG_F       (a|e|f|g)       //F

#define SEG_U       (b|c|d|e|f)   //U    //2023.3.24
#define SEG_C       (a|d|e|f)   //C     //2023.3.24

extern  const  u8 seg_num[];


#endif /* SOFTWARE_DRIVER_7_SEGMENT_H_ */
