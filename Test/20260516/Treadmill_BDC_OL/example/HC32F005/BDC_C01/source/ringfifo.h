/* ringfifo.h - ring FIFO and ringfifo_* API. */
#ifndef _RINGFIFO_H_
#define _RINGFIFO_H_

//#include "stm32f10x.h"
#include "hc32f005.h"
#include <stdint.h>

#define min(a, b) (((a) < (b)) ? (a) : (b))

typedef struct ringfifo {
    uint8_t   *buffer;           /* the buffer holding the data */
    uint32_t   size;            /* buffer length; MUST be 2^n（init 时会向下规范） */
    volatile uint32_t in;      /* 写指针：单调递增，按 (in & (size-1)) 落盘 */
    volatile uint32_t out;   /* 读指针：单调递增，ISR/主循环可见性勿优化掉 */
} ringfifo_t;

void             ringfifo_init(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int size);
int              __ringfifo_putin(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int len);
int              ringfifo_putin(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int len);
int              __ringfifo_get(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int len);
int              __ringfifo_dummy_get(ringfifo_t *ringfifo, unsigned int len);
int              ringfifo_get(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int len);
int              ringfifo_dummy_get(ringfifo_t *ringfifo, unsigned int len);

unsigned int     check_ringfifo_data(ringfifo_t *ringfifo);

/* 从 out+rel_offset 起拷贝 len 字节到 buffer，不移动 out（先偷看再解析帧头） */
unsigned int     ringfifo_peek(ringfifo_t *ringfifo, unsigned int rel_offset,
                               uint8_t *buffer, unsigned int len);
/* 在当前可读窗口内查找首字节为 header 的相对偏移；未找到返回 (unsigned int)-1 */
unsigned int     ringfifo_find_header(ringfifo_t *ringfifo, uint8_t header);

unsigned int     ringfifo_seek(ringfifo_t *ringfifo, const char *str);
int              ringfifo_or_seek(ringfifo_t *ringfifo, unsigned char str1, unsigned char str2);

int              __ringfifo_dummy_putin(ringfifo_t *ringfifo, unsigned int len);
int              ringfifo_dummy_putin(ringfifo_t *ringfifo, unsigned int len);

uint8_t         *get_ringfifo_putin_entry(ringfifo_t *ringfifo);
uint8_t         *get_ringfifo_get_entry(ringfifo_t *ringfifo, unsigned int offset);

#endif
