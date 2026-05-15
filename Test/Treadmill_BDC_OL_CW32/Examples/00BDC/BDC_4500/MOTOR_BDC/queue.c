#include "queue.h"

void Create_Queue(T_QUEUE *Q, uint8_t *pbuff, uint32_t maxsize)
{
    Q->pBase   = pbuff;
    Q->front   = 0;
    Q->rear    = 0;
    Q->maxsize = maxsize;
}

uint8_t Is_Full_Queue(T_QUEUE *Q)
{
    if (Q->front == (Q->rear + 1u) % Q->maxsize) {
        return 1u;
    }
    return 0u;
}

static uint8_t Is_Empty_Queue(T_QUEUE *Q)
{
    if (Q->front == Q->rear) {
        return 1u;
    }
    return 0u;
}

uint8_t Enter_queue(T_QUEUE *Q, uint8_t val)
{
    if (Is_Full_Queue(Q)) {
        return 0u;
    }
    Q->pBase[Q->rear] = val;
    Q->rear           = (Q->rear + 1u) % Q->maxsize;
    return 1u;
}

uint8_t Denter_queue(T_QUEUE *Q, uint8_t *val)
{
    if (Is_Empty_Queue(Q)) {
        return 0u;
    }
    *val    = Q->pBase[Q->front];
    Q->front = (Q->front + 1u) % Q->maxsize;
    return 1u;
}

uint32_t get_queue_length(T_QUEUE *Q)
{
    return (Q->maxsize + Q->rear - Q->front) % Q->maxsize;
}
