#ifndef __SYS_AEROBUF_H
#define __SYS_AEROBUF_H

#include <stdint.h>
#include <stdbool.h>


// 环形缓冲区结构体
typedef struct {
    uint8_t *pBuf;      // 缓冲区指针
    uint16_t mask;      // 掩码 (size - 1)
    volatile uint16_t head; // 写指针
    volatile uint16_t tail; // 读指针
} AeroBuf_t;

// API 定义
void AeroBuf_Init(AeroBuf_t *q, uint8_t *pPool, uint16_t size);
bool AeroBuf_Push(AeroBuf_t *q, uint8_t val);
bool AeroBuf_Pop(AeroBuf_t *q, uint8_t *pOut);
uint16_t AeroBuf_GetLen(AeroBuf_t *q);
void AeroBuf_Flush(AeroBuf_t *q);

#endif
