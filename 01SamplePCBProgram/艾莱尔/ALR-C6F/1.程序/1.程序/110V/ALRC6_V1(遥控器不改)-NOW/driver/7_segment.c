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
    (a|b|c|e|f|g),  //A
		(a|b|c|d|e|f|g),//B
		(a|d|e|f),      //C 
		(a|b|c|d|e|f),  //D
		(a|f|g|e|d),    //e
		(a|f|g|e),       //f
    (b|c|d|e|f)    //u
};

