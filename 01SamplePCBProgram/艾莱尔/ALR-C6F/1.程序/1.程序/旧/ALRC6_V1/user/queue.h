#ifndef  QUEUE_HH_
#define	 QUEUE_HH_ 

#include "common.h"

typedef struct _T_QUEUE 
{
	u8 *pBase;
	u32 front;    //指向队列第一个元素
	u32 rear;    //指向队列最后一个元素的下一个元素
	u32 maxsize; //循环队列的最大存储空间
}T_QUEUE;

typedef struct _T_QUEUE_1 
{
	u8 *pBase_1;
	u32 front_1;    //指向队列第一个元素
	u32 rear_1;    //指向队列最后一个元素的下一个元素
	u32 maxsize_1; //循环队列的最大存储空间
}T_QUEUE_1;

/*Eextern  variable  And function  Declaration------------------------------------------*/

extern  void Create_Queue(T_QUEUE *Q,u8 *pbuff,u32 maxsize);
extern  void Create_Queue_1(T_QUEUE_1 *Q,u8 *pbuff,u32 maxsize);
extern  u8 UART1_Enter_queue(T_QUEUE *Q, u8 val);
extern  u8 UART0_Enter_queue(T_QUEUE *Q, u8 val);
extern  u8 Denter_queue(T_QUEUE *Q, u8 *val);
extern  u8 UART1_Is_Full_Queue(T_QUEUE *Q);
extern  u8 UART0_Is_Full_Queue(T_QUEUE *Q);

#endif
