/*****************=======================================*********** 
 * 文件名  ：xxx.c
 * 描述    ：
 *          
 * 硬件平台：
 *         
 * 硬件连接：
 * 库版本  ：
 *
 * 作者    ：BrantYu
 * 日期    ：
 * 修改历史：
 * v1.0:
*******************************************************************/

/* common  Includes ------------------------------------------------------------------*/
#include "common.h"

/* User  Includes ------------------------------------------------------------------*/

#include "queue.h"

/* Micro  define  ------------------------------------------------------------------*/

/* Type  define  ------------------------------------------------------------------*/
// 函数指针定义
/*
  void (*pfunc) (void);
  u8   (*pfunc) (u8,u8);
*/


/*Eextern  variable  And function  Declaration------------------------------------------*/


/* Local   variable  And function  Declaration-------------------------------------------*/
//static 


/* Global   variable  And function  Declaration------------------------------------------*/



/* User    code  As below     ----------------------------------------------------------*/

void Create_Queue(T_QUEUE *Q,u8 *pbuff,u32 maxsize)
{
	Q->pBase=pbuff;	
	Q->front=0;         //初始化参数
	Q->rear=0;
	Q->maxsize=maxsize;
}


 u8  Is_Full_Queue(T_QUEUE *Q)
{
	if(Q->front==(Q->rear+1)%Q->maxsize)    //判断循环链表是否满，留一个预留空间不用
		return 1;
	else
		return 0;
}

u8 UART0_Is_Full_Queue(T_QUEUE *Q)
{
	if(Q->front==(Q->rear+1)%Q->maxsize)    //判断循环链表是否满，留一个预留空间不用
		return 1;
	else
		return 0;
}


static u8 Is_Empty_Queue(T_QUEUE *Q)
{
	if(Q->front==Q->rear)    //判断是否为空
		return 1;
	else
		return 0;
}


u8  Enter_queue(T_QUEUE *Q, u8 val)
{
	if(Is_Full_Queue(Q))
		return 0;
	else
	{
		Q->pBase[Q->rear]=val;
		Q->rear=(Q->rear+1)%Q->maxsize;
		return 1;
	}
}

u8 UART0_Enter_queue(T_QUEUE *Q, u8 val)   //加入队列
{
	if(UART0_Is_Full_Queue(Q))
		return 0;
	else
	{
		Q->pBase[Q->rear]=val;
		Q->rear=(Q->rear+1)%Q->maxsize;
		return 1;
	}
}

u8 Denter_queue(T_QUEUE *Q, u8 *val)
{
	if(Is_Empty_Queue(Q))
	{
		return 0;
	}
	else
	{
		*val=Q->pBase[Q->front];
		Q->front=(Q->front+1)%Q->maxsize;
		return 1;
	}
}




