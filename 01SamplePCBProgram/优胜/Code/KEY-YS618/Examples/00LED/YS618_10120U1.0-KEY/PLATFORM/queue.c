#include "queue.h"

/* 初始化 */
void queue_init(queue_t *q)
{
    q->write = 0;
    q->read  = 0;
}

/* 入队（中断可用） */
uint8_t queue_push(queue_t *q, uint8_t data)
{
    uint16_t next = (q->write + 1) & (QUEUE_SIZE - 1);

    /* 满了 */
    if (next == q->read)
        return 0;

    q->buf[q->write] = data;
    q->write = next;

    return 1;
}

/* 出队 */
uint8_t queue_pop(queue_t *q, uint8_t *data)
{
    if (q->read == q->write)
        return 0;

    *data = q->buf[q->read];
    q->read = (q->read + 1) & (QUEUE_SIZE - 1);

    return 1;
}

/* 是否为空 */
uint8_t queue_is_empty(queue_t *q)
{
    return (q->read == q->write);
}

/* 是否满 */
uint8_t queue_is_full(queue_t *q)
{
    return (((q->write + 1) & (QUEUE_SIZE - 1)) == q->read);
}

/* 当前数据量 */
uint16_t queue_size(queue_t *q)
{
    return (q->write - q->read) & (QUEUE_SIZE - 1);
}

