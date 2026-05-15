/* Ctrl + h 将QUEUE 替换 需要的 字符串*/
//QUEUE

#ifndef  __QUEUE_H__
#define	 __QUEUE_H__ 



/* common  Includes ------------------------------------------------------------------*/
#include "common.h"


/* User  Includes ------------------------------------------------------------------*/



/* Micro  define  ------------------------------------------------------------------*/


/* Type  define  ------------------------------------------------------------------*/

// 函数指针定义
/*
  void (*pfunc) (void);
  u8   (*pfunc) (u8,u8);
*/


typedef struct _T_QUEUE 
{
	u8 *pBase;
	u32 front;    //指向队列第一个元素
	u32 rear;    //指向队列最后一个元素的下一个元素
	u32 maxsize; //循环队列的最大存储空间
}T_QUEUE;



/*Eextern  variable  And function  Declaration------------------------------------------*/

extern  void Create_Queue(T_QUEUE *Q,u8 *pbuff,u32 maxsize);
extern  u8 Enter_queue(T_QUEUE *Q, u8 val);
extern  u8 UART0_Enter_queue(T_QUEUE *Q, u8 val);
extern  u8 Denter_queue(T_QUEUE *Q, u8 *val);
extern  u8 Is_Full_Queue(T_QUEUE *Q);
extern  u8 UART0_Is_Full_Queue(T_QUEUE *Q);


#endif
