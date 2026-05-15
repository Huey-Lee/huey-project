/*
 * 7_segment.c
 *
 *  Created on: 2018年8月1日
 *      Author: Administrator
 */
#include  "common.h"
#include "7_segment.h"


 const u8 seg_num[]=
{
#if(0)
  //共阳，低电平点亮
	~(a|b|c|d|e|f),//0
	~(b|c),//1
	~(a|b|g|e|d),//2
	~(a|b|c|d|g),//3
	~(f|g|b|c),//4
	~(a|f|g|c|d),//5
	~(a|f|e|d|c|g),//6
	~(a|b|c),//7
	~(a|b|c|d|e|f|g),//8,all on
	~(a|f|g|c|d|b),//9
	~0x00,//all off
	~(a|f|g|e|d),//e
	~(a|f|g|e)//f
#else
	//共阴，高电平点亮
		(a|b|c|d|e|f),//0
		(b|c),//1
		(a|b|g|e|d),//2
		(a|b|c|d|g),//3
		(f|g|b|c),//4
		(a|f|g|c|d),//5
		(a|f|e|d|c|g),//6
		(a|b|c),//7
		(a|b|c|d|e|f|g),//8,all on
		(a|f|g|c|d|b),//9
		0x00,//all off
		(a|f|g|e|d),//e
		(a|f|g|e)//f
#endif
};



