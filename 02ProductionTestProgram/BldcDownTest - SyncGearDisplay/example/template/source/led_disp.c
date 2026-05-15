/*
 * led_disp.c
 *
 *  Created on: 2019年11月27日
 *      Author: Administrator
 */

#include "led_disp.h"
#include "7_segment.h"
#include "indecate.h"

static u8 *ptrseg;

static u8 buf[2][4];
static u8 _mode=0;
static u8 cnt=0,flash_cnt=0;


void led_display_init(void)
{
	ptrseg=(u8*)&indecate.seg[0];
	set_seg_mode(SEG_MODE_NORMAL);
}


void set_bit_seg(u8 index, u8 dat)
{
	ptrseg[index]=dat;
	indecate_update_flag=1;
}


void set_seg_val(u16 val)
{
	u8 dat=(val/1000);
	if(dat==0){dat=SEG_ALL_OFF;}
	ptrseg[0]=seg_num[dat];
	ptrseg[1]=seg_num[(val%1000)/100];
	ptrseg[2]=seg_num[(val%100)/10];
	ptrseg[3]=seg_num[val%10];

	indecate_update_flag=1;
}


void set_seg_two(u8 hi2,u8 lo2)
{
	u8 dat=(hi2/10);
	if(dat==0){dat=SEG_ALL_OFF;}
	ptrseg[0]=seg_num[dat];
	ptrseg[1]=seg_num[(hi2%10)];

	ptrseg[2]=seg_num[(lo2/10)];
	ptrseg[3]=seg_num[lo2%10];

	indecate_update_flag=1;
}


void set_seg_two_sc(u8 hi2,u8 lo2)
{
	u8 dat=(hi2/100);
	if(dat==0){dat=SEG_ALL_OFF;}
	ptrseg[0]=seg_num[dat];
	ptrseg[1]=seg_num[(hi2/100)];

	ptrseg[2]=seg_num[(hi2/10)];
	ptrseg[3]=seg_num[hi2%10];

	indecate_update_flag=1;
}


void set_seg_two_sp(u8 hi2,u8 lo2)
{
	u8 dat=(hi2/100);
	if(dat==0){dat=SEG_ALL_OFF;}
	ptrseg[0]=seg_num[dat];
	ptrseg[1]=seg_num[(hi2/10)];

	ptrseg[2]=seg_num[(hi2%10)];
	ptrseg[3]=seg_num[lo2/10];

	indecate_update_flag=1;
}

void set_seg_temp(u16 temp_raw)
{
    u8 bai = (temp_raw / 100) % 10;
    u8 shi = (temp_raw / 10) % 10;
    u8 ge  = temp_raw % 10;

    ptrseg[0] = (bai == 0) ? SEG_ALL_OFF : seg_num[bai];
    ptrseg[1] = seg_num[shi];
    ptrseg[2] = seg_num[ge] | 0x80; // 带小数点的个位
    ptrseg[3] = SEG_C; 
}

void set_seg_mode(u8 mode)
{
	_mode=mode;
	cnt=0;
	indecate_update_flag=1;
}

//void set_seg_toggle_val(u16 val0,u16 val1)
//{
//	u8 dat;
//	dat=(val0/10000);
//	if(dat==0){dat=SEG_ALL_OFF;}
//	buf[0][0]=seg_num[dat];
//	buf[0][1]=seg_num[(val0%1000)/100];
//	buf[0][2]=seg_num[(val0%100)/10];
//	buf[0][3]=seg_num[val0%10];


//	dat=(val1/1000);
//	if(dat==0){dat=SEG_ALL_OFF;}
//	buf[1][0]=seg_num[dat];
//	buf[1][1]=seg_num[(val1%1000)/100];
//	buf[1][2]=seg_num[(val1%100)/10];
//	buf[1][3]=seg_num[val1%10];

//	indecate_update_flag=1;
//}


void set_seg_flash_cnt(u8 cnt)
{
	flash_cnt=cnt-1;
	indecate_update_flag=1;
}


void seg_disp_proc(void)
{
	switch(_mode)
	{
		case SEG_MODE_NORMAL:
			break;
		case SEG_MODE_FLASH_SELF:
			if(cnt==0)
			{
				buf[0][0]=ptrseg[0];
				buf[0][1]=ptrseg[1];
				buf[0][2]=ptrseg[2];
				buf[0][3]=ptrseg[3];
			}
			else if(cnt==SEG_FLASH_FREQ)
			{
				ptrseg[0]=0x00;
				ptrseg[1]=0x00;
				ptrseg[2]=0x00;
				ptrseg[3]=0x00;
				indecate_update_flag=1;
			}
			else if(cnt==(SEG_FLASH_FREQ<<1))
			{
				ptrseg[0]=buf[0][0];
				ptrseg[1]=buf[0][1];
				ptrseg[2]=buf[0][2];
				ptrseg[3]=buf[0][3];
				cnt=0xff;
				indecate_update_flag=1;
				//if(flash_cnt){flash_cnt--;}
				//else{_mode=SEG_MODE_NORMAL;break;}
			}
			cnt++;
			break;
		case SEG_MODE_FLASH_TOGGLE:
			if(cnt==0)
			{
				ptrseg[0]=buf[0][0];
				ptrseg[1]=buf[0][1];
				ptrseg[2]=buf[0][2];
				ptrseg[3]=buf[0][3];
			}
			else if(cnt==SEG_FLASH_FREQ)
			{
				ptrseg[0]=buf[1][0];
				ptrseg[1]=buf[1][1];
				ptrseg[2]=buf[1][2];
				ptrseg[3]=buf[1][3];
				indecate_update_flag=1;
			}
			else if(cnt==(SEG_FLASH_FREQ<<1))
			{
				ptrseg[0]=buf[0][0];
				ptrseg[1]=buf[0][1];
				ptrseg[2]=buf[0][2];
				ptrseg[3]=buf[0][3];
				indecate_update_flag=1;
				cnt=0;
			}
			cnt++;
			break;
		case SEG_MODE_FLASH_COLON:
			if(cnt==0)
			{
				indecate.led_updp=LED_ON;
				indecate.led_downdp=LED_ON;
			}
			else if(cnt==SEG_FLASH_FREQ)
			{
				indecate.led_updp=LED_OFF;
				indecate.led_downdp=LED_OFF;
				indecate_update_flag=1;
			}
			else if(cnt==(SEG_FLASH_FREQ<<1))
			{
				indecate.led_updp=LED_ON;
				indecate.led_downdp=LED_ON;
				cnt=0xff;
				indecate_update_flag=1;
			}
			cnt++;
			break;
		default:
			break;
	}
}


//#if(0)
//void set_bit_led(u8 index,u8 status)
//{

//	indecate.led9=0;
//	indecate.led10=0;
//	indecate.led21=0;
//	indecate.led30=0;
//	indecate.led31=0;
//	indecate.led40=0;
//	indecate.led41=0;

//	if(index==0){indecate.led9=status==LED_ON?1:0;}
//	else if(index==1){indecate.led10=status==LED_ON?1:0;}
//	else if(index==2){indecate.led21=status==LED_ON?1:0;}
//	else if(index==3){indecate.led30=status==LED_ON?1:0;}
//	else if(index==4){indecate.led31=status==LED_ON?1:0;}
//	else if(index==5){indecate.led40=status==LED_ON?1:0;}
//	else if(index==6){indecate.led41=status==LED_ON?1:0;}
//	else{}
//	indecate_update_flag=1;
//}
//#endif




