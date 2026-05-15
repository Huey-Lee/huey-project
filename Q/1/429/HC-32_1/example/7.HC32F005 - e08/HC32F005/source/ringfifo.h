#ifndef _RINGFIFO_H_
#define _RINGFIFO_H_

//#include "stm32f10x.h"
#include "hc32f005.h"

#define min(a, b) (((a) < (b)) ? (a) : (b))
typedef struct ringfifo{
    uint8_t *buffer;           /* the buffer holding the data */
    unsigned int size;         /* the size of the allocated buffer */
    unsigned int in;           /* data is added at offset (in % size) */
    unsigned int out;          /* data is extracted from off. (out % size) */
} ringfifo_t;

void ringfifo_init(ringfifo_t *ringfifo, uint8_t *buffer, unsigned int size);
int __ringfifo_putin(ringfifo_t *ringfifo,  uint8_t *buffer, unsigned int len);
int ringfifo_putin(ringfifo_t *ringfifo,  uint8_t *buffer, unsigned int len);
int __ringfifo_get(ringfifo_t *ringfifo,  uint8_t *buffer, unsigned int len);
int __ringfifo_dummy_get(ringfifo_t *ringfifo,  unsigned int len);

int ringfifo_get(ringfifo_t *ringfifo,  uint8_t *buffer, unsigned int len);
int ringfifo_dummy_get(ringfifo_t *ringfifo, unsigned int len);

unsigned int check_ringfifo_data(ringfifo_t *ringfifo);
unsigned int ringfifo_seek(ringfifo_t *ringfifo, const char *str);
//unsigned int ringfifo_seek(ringfifo_t *ringfifo, unsigned char str);

int ringfifo_or_seek(ringfifo_t *ringfifo, unsigned char str1,unsigned char str2);

int __ringfifo_dummy_putin(ringfifo_t *ringfifo, unsigned int len);
int ringfifo_dummy_putin(ringfifo_t *ringfifo, unsigned int len);

 uint8_t *get_ringfifo_putin_entry(ringfifo_t *ringfifo);

 uint8_t *get_ringfifo_get_entry(ringfifo_t *ringfifo, unsigned int offset);

#endif
