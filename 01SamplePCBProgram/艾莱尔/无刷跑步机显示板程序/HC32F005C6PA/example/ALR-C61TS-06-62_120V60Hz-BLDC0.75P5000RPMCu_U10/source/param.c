/*
 * param.c
 *
 *  Created on: 2020Äê3ÔÂ9ÈÕ
 *      Author: Administrator
 */
#include "common.h"
#include "param.h"

param_t param;


void param_init(void)
{
	param.en=0;
	param.index=0;
	param.speed_adc=0;
	param.book_en=0;
}
