/*
 * programe.c  ????
 *
 *  Created on: 2021??4??29??
 *      Author: Administrator
 */
#include "common.h"
#include "programe.h"

programe_t  programe;

const prog_tab_t prog_tab[] =
{
  {3, 3, 6, 5, 5, 4, 4, 4, 4, 3},   // P01
  {3, 3, 4, 4, 5, 5, 5, 6, 6, 4},   // P02
  {2, 4, 6, 6, 6, 6, 6, 2, 3, 2},   // P03
  {3, 3, 5, 6, 6, 6, 5, 4, 3, 3},   // P04
  {3, 6, 6, 6, 6, 6, 6, 5, 5, 4},   // P05
  {2, 6, 5, 4, 6, 6, 5, 3, 3, 2},   // P06
  {2, 6, 6, 6, 6, 6, 5, 3, 2, 2},   // P07
  {2, 4, 4, 4, 5, 6, 6, 6, 6, 2},   // P08
  {2, 4, 5, 5, 6, 5, 6, 3, 3, 2},   // P09
  {2, 5, 6, 5, 6, 6, 5, 2, 4, 3},   // P10
  {2, 3, 4, 5, 6, 6, 6, 5, 3, 2},   // P11
  {2, 3, 5, 6, 6, 6, 6, 6, 5, 3},   // P12
};


void programe_init(void)
{
	programe.index = 0;                   //????
  programe.item = 0;                    //???
	programe.total_time = 1800;           //????? 20*90=30min
  programe.interval_time = 0;           //?????? 1s
  programe.use_prog_speed = 0;
  programe.run_use_tab_speed = 0;
  programe.session_is_program = 0;
}
