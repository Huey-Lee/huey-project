#include "common.h"
#include "queue.h"

void Create_Queue(T_QUEUE *Q,u8 *pbuff,u32 maxsize)
{
	Q->pBase=pbuff;	
	Q->front=0;         //初始化参数
	Q->rear=0;
	Q->maxsize=maxsize;
}
//void Create_Queue_1(T_QUEUE_1 *Q,u8 *pbuff,u32 maxsize)
//{
//	Q->pBase_1=pbuff;	
//	Q->front_1=0;         //初始化参数
//	Q->rear_1=0;
//	Q->maxsize_1=maxsize;
//}

u8 UART1_Is_Full_Queue(T_QUEUE *Q)
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


u8 UART1_Enter_queue(T_QUEUE *Q, u8 val)   //加入队列
{
	if(UART1_Is_Full_Queue(Q))
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

u8 Denter_queue(T_QUEUE *Q, u8 *val)   //出队列
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

