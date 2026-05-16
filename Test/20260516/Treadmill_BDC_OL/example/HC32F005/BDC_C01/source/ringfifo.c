/*
 * Function: 双索引环形字节 FIFO：可跨中断与主循环安全传递流数据。
 * Method:   volatile in/out 单调递增，容量为 2^n 时用掩码寻址；写满丢弃尾部；提供 put/get/peek/按字节搜帧头及字符串匹配偏移。
 * Name:     Huey
 * Date:     May 16, 2026 18:00
 */

#include "ringfifo.h"
#include <string.h>
#include "hc32f005.h"

static uint32_t ringfifo_floor_pow2_u32(uint32_t v)
{
    uint32_t p = 1u;

    if (v < 2u)
        return 0u;
    while (p <= v / 2u)
        p <<= 1u;
    return p;
}

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
    len        = RINGFIFO_MIN(len, free_bytes);
    if (len == 0u)
        return 0;

    szm = ringfifo->size - 1u;
    l   = RINGFIFO_MIN(len, ringfifo->size - (unsigned int)(in & szm));
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
    len        = RINGFIFO_MIN(len, free_bytes);
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
    len   = RINGFIFO_MIN(len, avail);
    if (len == 0u)
        return 0;

    szm = ringfifo->size - 1u;
    l   = RINGFIFO_MIN(len, ringfifo->size - (unsigned int)(out & szm));
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
    len   = RINGFIFO_MIN(len, avail);
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

    len = RINGFIFO_MIN(len, avail - rel_offset);
    if (len == 0u)
        return 0u;

    out = ringfifo->out;
    pos = out + rel_offset;
    szm = ringfifo->size - 1u;
    l   = RINGFIFO_MIN(len, ringfifo->size - (unsigned int)(pos & szm));
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
