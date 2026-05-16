/*
 * Function: 环形 FIFO 类型定义与字节流 API 声明。
 * Method:   ringfifo_t 含 buffer、2^n 容量与 volatile 双指针；对外 put/get/peek/find_header/seek 等与 ringfifo.c 一致。
 * Name:     Huey
 * Date:     May 16, 2026 18:00
 */

#ifndef _RINGFIFO_H_
#define _RINGFIFO_H_

#include "hc32f005.h"
#include <stdint.h>

#define RINGFIFO_MIN(a, b) (((a) < (b)) ? (a) : (b))

typedef struct ringfifo {
    uint8_t   *buffer;
    uint32_t   size;
    volatile uint32_t in;
    volatile uint32_t out;
} ringfifo_t;

void             ringfifo_init(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int size);
int              __ringfifo_putin(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int len);
int              ringfifo_putin(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int len);
int              __ringfifo_get(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int len);
int              __ringfifo_dummy_get(ringfifo_t *ringfifo, unsigned int len);
int              ringfifo_get(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int len);
int              ringfifo_dummy_get(ringfifo_t *ringfifo, unsigned int len);

unsigned int     check_ringfifo_data(ringfifo_t *ringfifo);

unsigned int     ringfifo_peek(ringfifo_t *ringfifo, unsigned int rel_offset,
                               uint8_t *buffer, unsigned int len);
unsigned int     ringfifo_find_header(ringfifo_t *ringfifo, uint8_t header);

unsigned int     ringfifo_seek(ringfifo_t *ringfifo, const char *str);
int              ringfifo_or_seek(ringfifo_t *ringfifo, unsigned char str1, unsigned char str2);

int              __ringfifo_dummy_putin(ringfifo_t *ringfifo, unsigned int len);
int              ringfifo_dummy_putin(ringfifo_t *ringfifo, unsigned int len);

uint8_t         *get_ringfifo_putin_entry(ringfifo_t *ringfifo);
uint8_t         *get_ringfifo_get_entry(ringfifo_t *ringfifo, unsigned int offset);

#endif
