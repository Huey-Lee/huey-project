/* ringfifo.c - ring buffer between UART ISR and main. */
#include "ringfifo.h"
#include <string.h>
#include "hc32f005.h"

/* 将 size 规范为 ≤ 原值的最大 2 的幂；<2 视为无效 */
static uint32_t ringfifo_floor_pow2_u32(uint32_t v)
{
    uint32_t p = 1u;

    if (v < 2u)
        return 0u;
    while (p <= v / 2u)
        p <<= 1u;
    return p;
}

/* Init：size 须为 2^n 才安全用于掩码；非幂则向下取整，避免越界 */
void ringfifo_init(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int size)
{
    uint32_t sz;

    if (ringfifo == 0 || buffer == 0) {
        return;
    }
    sz = ringfifo_floor_pow2_u32((uint32_t)size);
    if (sz < 2u) {
        ringfifo->buffer = 0;
        ringfifo->size   = 0u;
        ringfifo->in = ringfifo->out = 0u;
        return;
    }
    ringfifo->buffer = buffer;
    ringfifo->size   = sz;
    ringfifo->in     = 0u;
    ringfifo->out    = 0u;
}

/* 数据先 memcpy，最后一行再 in += len，避免主循环看到半包 */
int __ringfifo_putin(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int len)
{
    unsigned int  l;
    uint32_t      in  = ringfifo->in;
    uint32_t      out = ringfifo->out;
    unsigned int  free_bytes;
    uint32_t      szm;

    if (ringfifo->size < 2u || ringfifo->buffer == 0)
        return 0;

    free_bytes = (unsigned int)(ringfifo->size - (unsigned int)(in - out));
    len        = min(len, free_bytes);
    if (len == 0u)
        return 0;

    szm = ringfifo->size - 1u;
    l   = min(len, ringfifo->size - (unsigned int)(in & szm));
    memcpy(ringfifo->buffer + (in & szm), buffer, l);
    memcpy(ringfifo->buffer, buffer + l, len - l);

    ringfifo->in = in + (uint32_t)len;
    return (int)len;
}

int __ringfifo_dummy_putin(ringfifo_t *ringfifo, unsigned int len)
{
    uint32_t     in  = ringfifo->in;
    uint32_t     out = ringfifo->out;
    unsigned int free_bytes;

    if (ringfifo->size < 2u)
        return 0;

    free_bytes = (unsigned int)(ringfifo->size - (unsigned int)(in - out));
    len        = min(len, free_bytes);
    ringfifo->in = in + (uint32_t)len;
    return (int)len;
}

int ringfifo_putin(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int len)
{
    return __ringfifo_putin(ringfifo, buffer, len);
}

int ringfifo_dummy_putin(ringfifo_t *ringfifo, unsigned int len)
{
    return __ringfifo_dummy_putin(ringfifo, len);
}

/* Pop：先 memcpy，再 out += len；禁止 in/out 手动归零（竞态会丢包） */
int __ringfifo_get(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int len)
{
    unsigned int l;
    uint32_t     in  = ringfifo->in;
    uint32_t     out = ringfifo->out;
    unsigned int avail;
    uint32_t     szm;

    if (ringfifo->size < 2u || ringfifo->buffer == 0)
        return 0;

    avail = (unsigned int)(in - out);
    len   = min(len, avail);
    if (len == 0u)
        return 0;

    szm = ringfifo->size - 1u;
    l   = min(len, ringfifo->size - (unsigned int)(out & szm));
    memcpy(buffer, ringfifo->buffer + (out & szm), l);
    memcpy(buffer + l, ringfifo->buffer, len - l);

    ringfifo->out = out + (uint32_t)len;
    return (int)len;
}

int __ringfifo_dummy_get(ringfifo_t *ringfifo, unsigned int len)
{
    uint32_t     in  = ringfifo->in;
    uint32_t     out = ringfifo->out;
    unsigned int avail;

    avail = (unsigned int)(in - out);
    len   = min(len, avail);
    ringfifo->out = out + (uint32_t)len;
    return (int)len;
}

int ringfifo_get(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int len)
{
    return __ringfifo_get(ringfifo, buffer, len);
}

int ringfifo_dummy_get(ringfifo_t *ringfifo, unsigned int len)
{
    return __ringfifo_dummy_get(ringfifo, len);
}

/* 可读字节数：uint32 差自动回绕，勿在 in==out 时强行 in=out=0 */
unsigned int check_ringfifo_data(ringfifo_t *ringfifo)
{
    return (unsigned int)(ringfifo->in - ringfifo->out);
}

unsigned int ringfifo_peek(ringfifo_t *ringfifo, unsigned int rel_offset,
                           uint8_t *buffer, unsigned int len)
{
    unsigned int avail;
    unsigned int l;
    uint32_t     out;
    uint32_t     pos;
    uint32_t     szm;

    if (ringfifo->size < 2u || ringfifo->buffer == 0 || buffer == 0)
        return 0u;

    avail = check_ringfifo_data(ringfifo);
    if (rel_offset >= avail)
        return 0u;

    len = min(len, avail - rel_offset);
    if (len == 0u)
        return 0u;

    out = ringfifo->out;
    pos = out + rel_offset;
    szm = ringfifo->size - 1u;
    l   = min(len, ringfifo->size - (unsigned int)(pos & szm));
    memcpy(buffer, ringfifo->buffer + (pos & szm), l);
    memcpy(buffer + l, ringfifo->buffer, len - l);
    return len;
}

unsigned int ringfifo_find_header(ringfifo_t *ringfifo, uint8_t header)
{
    unsigned int avail;
    unsigned int k;
    uint32_t     out;
    uint32_t     szm;

    if (ringfifo->size < 2u || ringfifo->buffer == 0)
        return (unsigned int)-1;

    avail = check_ringfifo_data(ringfifo);
    out   = ringfifo->out;
    szm   = ringfifo->size - 1u;
    for (k = 0u; k < avail; k++) {
        if (ringfifo->buffer[(out + k) & szm] == header)
            return k;
    }
    return (unsigned int)-1;
}

unsigned int ringfifo_seek(ringfifo_t *ringfifo, const char *str)
{
    unsigned int i, j, avail, slen;

    if (str == 0 || ringfifo->size < 2u)
        return 0;
    avail = check_ringfifo_data(ringfifo);
    slen  = (unsigned int)strlen(str);
    if (slen == 0 || avail < slen)
        return 0;
    for (i = 0; i + slen <= avail; i++) {
        for (j = 0; j < slen; j++) {
            unsigned int pos =
                (unsigned int)((ringfifo->out + i + j) & (ringfifo->size - 1U));
            if (ringfifo->buffer[pos] != (uint8_t)str[j])
                break;
        }
        if (j == slen)
            return i + 1U;
    }
    return 0U;
}

int ringfifo_or_seek(ringfifo_t *ringfifo, unsigned char str1, unsigned char str2)
{
    unsigned int n, k;

    if (check_ringfifo_data(ringfifo) == 0U)
        return 0;
    n = (unsigned int)(ringfifo->in - ringfifo->out);
    for (k = 0; k < n; k++) {
        uint8_t c = ringfifo->buffer[(ringfifo->out + k) & (ringfifo->size - 1U)];
        if (c == str1)
            return 1;
        if (c == str2)
            return 2;
    }
    return 0;
}

uint8_t *get_ringfifo_putin_entry(ringfifo_t *ringfifo)
{
    if (ringfifo->size < 2u)
        return 0;
    return ringfifo->buffer + (ringfifo->in & (ringfifo->size - 1U));
}

uint8_t *get_ringfifo_get_entry(ringfifo_t *ringfifo, unsigned int offset)
{
    if (ringfifo->size < 2u)
        return 0;
    return ringfifo->buffer + ((ringfifo->out + offset) & (ringfifo->size - 1U));
}
