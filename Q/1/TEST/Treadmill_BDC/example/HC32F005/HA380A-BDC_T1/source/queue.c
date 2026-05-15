/* queue.c - byte queue for UART RX. */
#include "queue.h"

void Create_Queue(T_QUEUE *Q,u8 *pbuff,u32 maxsize)
{
	Q->pBase=pbuff;	
	Q->front=0;         /* empty: front==rear */
	Q->rear=0;
	Q->maxsize=maxsize;
}


u8  Is_Full_Queue(T_QUEUE *Q)
{
	if(Q->front==(Q->rear+1)%Q->maxsize)    /* one slot unused */
		return 1;
	else
		return 0;
}


static u8 Is_Empty_Queue(T_QUEUE *Q)
{
	if(Q->front==Q->rear)    /* empty */
		return 1;
	else
		return 0;
}


u8  Enter_queue(T_QUEUE *Q, u8 val)   /* enqueue one byte */
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


u8 Denter_queue(T_QUEUE *Q, u8 *val)   /* dequeue one byte */
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


u8 get_queue_length(T_QUEUE *Q)
{
	return (Q->maxsize+Q->rear-Q->front)%Q->maxsize;
}
