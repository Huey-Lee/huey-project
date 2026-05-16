/*
 * programe.h
 *
 *  Created on: 2021??4??29??
 *      Author: Administrator
 */

#ifndef USER_PROGRAME_H_
#define USER_PROGRAME_H_
#include "common.h"

#define SPEED_NUM_PERPROGRAME (10)	//??????????????????

typedef struct _t_prog_tab
{
	/* Whole km/h per segment (e.g. 3 = 3.0); converted via uart_prog_kmh_to_internal + same protocol as manual */
	u8 speed[SPEED_NUM_PERPROGRAME];
}prog_tab_t;

extern const prog_tab_t prog_tab[];


#define PROGRAME_INTERVAL_TIME (180)	// ?????????3min

typedef struct _t_programe
{
	u8 index;           //????
	u8 item;					  //???
	u16 total_time;     //?????
	u8  interval_time;  //??????
	u8  use_prog_speed;
	u8  run_use_tab_speed;  /* START˲�����棺������ prog ��һ�� */
	u8  session_is_program; /* RUNNING �жγ�ʽ��ʱ */
}programe_t;

extern programe_t  programe;

extern void programe_init(void);
#endif /* USER_PROGRAME_H_ */
