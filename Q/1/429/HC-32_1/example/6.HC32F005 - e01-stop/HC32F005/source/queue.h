/* Ctrl + h 将QUEUE 替换 需要的 字符串*/

#ifndef  QUEUE_HH_
#define	 QUEUE_HH_ 

#include "hc32f005.h"
#include "ddl.h"

typedef struct _T_QUEUE 
{
	u8 *pBase;
	u32 front;    //指向队列第一个元素
	u32 rear;     //指向队列最后一个元素的下一个元素
	u32 maxsize;  //循环队列的最大存储空间
}T_QUEUE;


extern  void Create_Queue(T_QUEUE *Q,u8 *pbuff,u32 maxsize);
extern  u8 Enter_queue(T_QUEUE *Q, u8 val);
extern  u8 Denter_queue(T_QUEUE *Q, u8 *val);
extern  u8 Is_Full_Queue(T_QUEUE *Q);
extern  u8 get_queue_length(T_QUEUE *Q);

#endif
