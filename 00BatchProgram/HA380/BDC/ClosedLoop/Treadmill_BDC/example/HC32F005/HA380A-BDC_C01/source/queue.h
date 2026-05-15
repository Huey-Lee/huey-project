/* queue.h - T_QUEUE and inline byte queue API. */
#ifndef  QUEUE_HH_
#define	 QUEUE_HH_ 

#include "hc32f005.h"
#include "ddl.h"

typedef struct _T_QUEUE 
{
	u8 *pBase;
	u32 front;   /* read index */
	u32 rear;    /* write index */
	u32 maxsize; /* ring size */
}T_QUEUE;


extern  void Create_Queue(T_QUEUE *Q,u8 *pbuff,u32 maxsize);
extern  u8 Enter_queue(T_QUEUE *Q, u8 val);
extern  u8 Denter_queue(T_QUEUE *Q, u8 *val);
extern  u8 Is_Full_Queue(T_QUEUE *Q);
extern  u8 get_queue_length(T_QUEUE *Q);

#endif
