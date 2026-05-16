/* ringfifo.c - ring buffer between UART ISR and main. */
#include "ringfifo.h"
#include <string.h>
#include "hc32f005.h"

#define RINGFIFO_SUPPORT

/* Init buffer, size (power of 2), empty. */
void ringfifo_init(ringfifo_t *ringfifo,uint8_t *buffer, unsigned int size){

	ringfifo->buffer = buffer;
	ringfifo->size = size;
	ringfifo->in = ringfifo->out =0;
}

/* Push len bytes; may wrap at end of buffer. */
int __ringfifo_putin(ringfifo_t *ringfifo,  uint8_t *buffer, unsigned int len){
	unsigned int l;
	len = min(len, ringfifo->size - ringfifo->in + ringfifo->out);
	l = min(len, ringfifo->size - (ringfifo->in & (ringfifo->size - 1)));
	memcpy(ringfifo->buffer + (ringfifo->in & (ringfifo->size-1)),buffer,l);
	memcpy(ringfifo->buffer, buffer + l, len - l);
	ringfifo->in +=len;
	return len;
}

int __ringfifo_dummy_putin(ringfifo_t *ringfifo, unsigned int len){
	
	len = min(len, ringfifo->size - ringfifo->in + ringfifo->out);
	
	ringfifo->in +=len;
	return len;
}

int ringfifo_putin(ringfifo_t *ringfifo,  uint8_t *buffer, unsigned int len){
	unsigned int ret;
	ret=__ringfifo_putin(ringfifo,buffer,len);
	return ret;
}


int ringfifo_dummy_putin(ringfifo_t *ringfifo,unsigned int len){
	int ret;
	ret = __ringfifo_dummy_putin(ringfifo,len);
	return ret;
}



/* Pop up to len bytes to buffer. */
int __ringfifo_get(ringfifo_t *ringfifo,  uint8_t *buffer, unsigned int len){
	unsigned int l;
	len = min(len, ringfifo->in - ringfifo->out);
	l = min(len, ringfifo->size - (ringfifo->out & (ringfifo->size - 1)));
	memcpy(buffer, ringfifo->buffer + (ringfifo->out & (ringfifo->size - 1)), l);
	memcpy(buffer + l, ringfifo->buffer, len - l);
	ringfifo->out += len;
	return len;
}

int __ringfifo_dummy_get(ringfifo_t *ringfifo,  unsigned int len){
	
	len = min(len, ringfifo->in - ringfifo->out);

	ringfifo->out += len; 
	return len;
}

int ringfifo_get(ringfifo_t *ringfifo,  uint8_t *buffer, unsigned int len){
 int ret;

ret = __ringfifo_get(ringfifo, buffer, len);

if (ringfifo->in == ringfifo->out)
         ringfifo->in = ringfifo->out = 0;

return ret;

}


int ringfifo_dummy_get(ringfifo_t *ringfifo, unsigned int len){
 int ret;
ret = __ringfifo_dummy_get(ringfifo,len);

if (ringfifo->in == ringfifo->out)
         ringfifo->in = ringfifo->out = 0;

return ret;
}

/* Bytes available: in - out (same model as __ringfifo_get). */
unsigned int check_ringfifo_data(ringfifo_t *ringfifo)
{
	if (ringfifo->in < ringfifo->out)
		return 0;
	return (unsigned int)(ringfifo->in - ringfifo->out);
}

/* Find nul-terminated pattern in ring; return 0 = not found, else 1-based start offset+1. */
unsigned int ringfifo_seek(ringfifo_t *ringfifo, const char *str)
{
	unsigned int i, j, avail, slen;

	if (str == 0)
		return 0;
	avail = check_ringfifo_data(ringfifo);
	slen = (unsigned int)strlen(str);
	if (slen == 0 || avail < slen)
		return 0;
	for (i = 0; i + slen <= avail; i++) {
		for (j = 0; j < slen; j++) {
			unsigned int pos = (ringfifo->out + i + j) & (ringfifo->size - 1U);
			if (ringfifo->buffer[pos] != (uint8_t)str[j])
				break;
		}
		if (j == slen)
			return i + 1U;
	}
	return 0U;
}

/* Return first of str1 or str2; 0=none, 1=first match str1, 2=first match str2. */
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
	return ringfifo->buffer + (ringfifo->in & (ringfifo->size - 1U));
}

uint8_t *get_ringfifo_get_entry(ringfifo_t *ringfifo, unsigned int offset)
{
	return ringfifo->buffer + ((ringfifo->out + offset) & (ringfifo->size - 1U));
}
