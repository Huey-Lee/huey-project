/*
 * indecate.c
 *
 *  Created on: 2018Äê8ÔÂ1ÈÕ
 *      Author: Administrator
 */
#include "common.h"
#include "indecate.h"
#include "aip1628.h"
#include "7_segment.h"
#include "treadmills.h"

indecate_t   indecate;

u8  indecate_update_flag=0;


void disp_update(void)
{
  aip1628_init();
	if(indecate_update_flag)
	{
    indecate_update_flag=0;
    
    if(treadmills.error_code==0)
		{
      aip1628_write_byte(0,indecate.seg[0]);      //COM1
      aip1628_write_byte(2,indecate.seg[1]);      //COM2
      aip1628_write_byte(4,indecate.seg[2]);      //COM3
      aip1628_write_byte(6,indecate.seg[3]);      //COM4
      aip1628_write_byte(8,((u8*)&indecate)[4]);  //COM5
    }
    else
    {
      aip1628_write_byte(0,SEG_ALL_OFF);      //COM1
      aip1628_write_byte(2,SEG_E);      //COM2
      aip1628_write_byte(4,seg_num[0]); //COM3
      aip1628_write_byte(6,seg_num[treadmills.error_code]);//COM4
      aip1628_write_byte(8,0x00);  			//COM5
    }
  }
}
