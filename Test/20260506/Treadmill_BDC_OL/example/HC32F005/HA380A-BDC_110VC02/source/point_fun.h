/* point_fun.h - MAIN_INIT and function-pointer glue. */
#ifndef __POINT_FUN_H
#define __POINT_FUN_H	
#include "ddl.h"


/******************************************************
*
*
*GLOBAL VARIABLE
*
*
******************************************************/
typedef struct _c_stack{
    int base_size;
    int point;
    int * base;
    int size;
		int (*pfun)(int data);
    int  (*pop)(struct _c_stack *);
    int  (*push)(int,struct _c_stack *);
    int  (*get_top)(struct _c_stack);
}c_stack;


/******************************************************
*
*
*FUNCTION DECLARATION
*
*
******************************************************/
void Select_Init(void);
int MAIN_INIT(int argc);
#endif

