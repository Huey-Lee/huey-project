#include "sys_aerobuf.h"

/**
 * @brief 初始化缓冲区
 * @param size 必须是 2 的阶乘 (16, 32, 64, 128, 256...)
 */
void AeroBuf_Init(AeroBuf_t *q, uint8_t *pPool, uint16_t size) {
    q->pBuf = pPool;
    q->mask = size - 1; // 关键：利用掩码代替取模
    q->head = 0;
    q->tail = 0;
}

/**
 * @brief 入队 (写入)
 */
bool AeroBuf_Push(AeroBuf_t *q, uint8_t val) {
    uint16_t next_head = (q->head + 1) & q->mask;
    
    if (next_head == q->tail) { // 缓冲区满
        return false;
    }
    
    q->pBuf[q->head] = val;
    q->head = next_head;
    return true;
}

/**
 * @brief 出队 (读取)
 */
bool AeroBuf_Pop(AeroBuf_t *q, uint8_t *pOut) {
    if (q->head == q->tail) { // 缓冲区空
        return false;
    }
    
    *pOut = q->pBuf[q->tail];
    q->tail = (q->tail + 1) & q->mask;
    return true;
}

/**
 * @brief 获取当前缓冲区剩余有效数据长度
 */
uint16_t AeroBuf_GetLen(AeroBuf_t *q) {
    return (q->head - q->tail) & q->mask;
}

/**
 * @brief 清空缓冲区
 */
void AeroBuf_Flush(AeroBuf_t *q) {
    q->head = q->tail = 0;
}

