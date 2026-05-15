/*
 * programe.h
 *
 *  Created on: 2021年4月29日
 *      Author: Administrator
 */

#ifndef USER_PROGRAME_H_
#define USER_PROGRAME_H_
#include "common.h"

#define SPEED_NUM_PERPROGRAME (20)	//每个程序包含的速度个数
typedef struct _t_prog_tab
{
	u8 speed[SPEED_NUM_PERPROGRAME];
}prog_tab_t;

extern const prog_tab_t  prog_tab[];


#define PROGRAME_INTERVAL_TIME (90)	//程序间隔时间1min   
typedef struct _t_programe
{
	u8 index;           //索引
	u8 item;					  //条目
	u16 total_time;     //总时间
	u8  interval_time;  //间隔时间
}programe_t;

extern programe_t  programe;

extern void programe_init(void);
#endif /* USER_PROGRAME_H_ */
