#ifndef __FAST_QUEUE_H
#define __FAST_QUEUE_H

#include <stdint.h>

/* Ðë2µÄÃÝ£º64 / 128 / 256 */
#define QUEUE_SIZE   128

typedef struct
{
    uint8_t buf[QUEUE_SIZE];

    volatile uint16_t write;
    volatile uint16_t read;

} queue_t;

/* ½Ó¿Ú */

void queue_init(queue_t *q);

uint8_t queue_push(queue_t *q, uint8_t data);

uint8_t queue_pop(queue_t *q, uint8_t *data);

uint8_t queue_is_empty(queue_t *q);

uint8_t queue_is_full(queue_t *q);

uint16_t queue_size(queue_t *q);

#endif
