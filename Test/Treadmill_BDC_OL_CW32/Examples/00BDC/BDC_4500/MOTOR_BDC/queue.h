#ifndef QUEUE_HH_
#define QUEUE_HH_

#include <stdint.h>

typedef struct _T_QUEUE {
    uint8_t *pBase;
    uint32_t front;
    uint32_t rear;
    uint32_t maxsize;
} T_QUEUE;

void Create_Queue(T_QUEUE *Q, uint8_t *pbuff, uint32_t maxsize);
uint8_t Enter_queue(T_QUEUE *Q, uint8_t val);
uint8_t Denter_queue(T_QUEUE *Q, uint8_t *val);
uint8_t Is_Full_Queue(T_QUEUE *Q);
uint32_t get_queue_length(T_QUEUE *Q);

#endif
