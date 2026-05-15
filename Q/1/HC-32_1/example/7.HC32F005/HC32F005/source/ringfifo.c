#include "ringfifo.h"
#include <string.h>
#include "hc32f005.h"

#define RINGFIFO_SUPPORT

/***************************************************************************
*
*
*环形队列初始化
*
*
***************************************************************************/
void ringfifo_init(ringfifo_t *ringfifo,uint8_t *buffer, unsigned int size){

	ringfifo->buffer = buffer;									//指针地址复制
	ringfifo->size = size;											//数组大小
	ringfifo->in = ringfifo->out =0;						//起始位置和结束位置清零
}

/***************************************************************************
*
*
*往环形队列放入数据
*
*
***************************************************************************/
int __ringfifo_putin(ringfifo_t *ringfifo,  uint8_t *buffer, unsigned int len){
	unsigned int l;
	len = min(len, ringfifo->size - ringfifo->in + ringfifo->out);						//获取当前剩余内存大小
	l = min(len, ringfifo->size - (ringfifo->in & (ringfifo->size - 1)));			//获取当前指针指向位置	---》 后面的空间大小
	memcpy(ringfifo->buffer + (ringfifo->in & (ringfifo->size-1)),buffer,l);	//向buffer的指定位置，当中写入长度为l的数据
	memcpy(ringfifo->buffer, buffer + l, len - l);														//循环接续（从头获取后面的数据）
	ringfifo->in +=len;																												//输入指针向后位移len个单位
	return len;
}

int __ringfifo_dummy_putin(ringfifo_t *ringfifo, unsigned int len){
	
	len = min(len, ringfifo->size - ringfifo->in + ringfifo->out);
	
	ringfifo->in +=len;
	return len;
}

/***************************************************************************
*
*
*往环形队列放入数据
*
*
***************************************************************************/
int ringfifo_putin(ringfifo_t *ringfifo,  uint8_t *buffer, unsigned int len){
	unsigned int ret;
	ret=__ringfifo_putin(ringfifo,buffer,len);
	return ret;
}


int ringfifo_dummy_putin(ringfifo_t *ringfifo,unsigned int len){
	int ret;
	ret=__ringfifo_dummy_putin(ringfifo,len);
	return ret;
}



/***************************************************************************
*
*
*从缓存当中取出长度为len的数据放到buffer里面
*
*
***************************************************************************/
int __ringfifo_get(ringfifo_t *ringfifo,  uint8_t *buffer, unsigned int len){
	unsigned int l;
	len = min(len, ringfifo->in - ringfifo->out);																	//获取已有数据长度
	l = min(len, ringfifo->size - (ringfifo->out & (ringfifo->size - 1)));				//获取读取指针	---》后面的数据大小
	memcpy(buffer, ringfifo->buffer + (ringfifo->out & (ringfifo->size - 1)), l);	//获取指定位置，长度为l的数据内容
	memcpy(buffer + l, ringfifo->buffer, len - l);																//循环接续（获取后面的数据）
	ringfifo->out += len; 																												//读取指针向后位移len个单位
	return len;																																		//返回读取的数据长度
}

int __ringfifo_dummy_get(ringfifo_t *ringfifo,  unsigned int len){
	
	len = min(len, ringfifo->in - ringfifo->out);

	ringfifo->out += len; 
	return len;
}

/***************************************************************************
*
*
*从缓存当中取出长度为len的数据放到buffer里面
*
*
***************************************************************************/
int ringfifo_get(ringfifo_t *ringfifo,  uint8_t *buffer, unsigned int len){
 int ret;

ret = __ringfifo_get(ringfifo, buffer, len);//从环形队列里面读取长度为len的数据放到buffer里面，并且返回读取的数据长度

if (ringfifo->in == ringfifo->out)					//如果输入的指针地址等于输出的指针地址
         ringfifo->in = ringfifo->out = 0;	//输入输出的指针地址都清零

return ret;																	//返回读取的数据长度大小

}


int ringfifo_dummy_get(ringfifo_t *ringfifo, unsigned int len){
 int ret;
ret = __ringfifo_dummy_get(ringfifo,len);

if (ringfifo->in == ringfifo->out)
         ringfifo->in = ringfifo->out = 0;

return ret;

}

unsigned int check_ringfifo_data(ringfifo_t *ringfifo)
{

return (ringfifo->in - ringfifo->out);
}


int ringfifo_or_seek(ringfifo_t *ringfifo, unsigned char str1,unsigned char str2)
{
	int ret=0;
	
	 unsigned char *pos;
	pos = ringfifo->buffer+(ringfifo->out  & (ringfifo->size - 1));

	if(*pos == str1 || *pos == str2)
		return 0;

	for(ret=0;ret<(ringfifo->in-ringfifo->out);ret++)
		{
			pos = ringfifo->buffer + ((ringfifo->out+ret)  & (ringfifo->size - 1));
			if((*pos == str1 || *pos == str2))
				return ret;
		}
	return -1;
}

unsigned int ringfifo_seek(ringfifo_t *ringfifo, const char *str)
{
	unsigned int offset=0;
	unsigned char *pos;
	unsigned int pOut=ringfifo->out;
	unsigned int fifoEnd = ringfifo->in - ringfifo->out;
	const char *pDelim = NULL;
	pDelim = str;
	for(offset=0;offset<fifoEnd;offset++){
	
		pos = ringfifo->buffer+((offset + pOut) & (ringfifo->size -1));
		pDelim = str;
			do{
				if(*pos == *pDelim){
					return offset+1;
				}
			}while(*(++pDelim) != 0);
		
	}

	return 0;

}


 uint8_t *get_ringfifo_putin_entry(ringfifo_t *ringfifo){
return ringfifo->buffer + (ringfifo->in & (ringfifo->size-1));
}


 uint8_t *get_ringfifo_get_entry(ringfifo_t *ringfifo, unsigned int offset){

return ringfifo->buffer+((ringfifo->out+offset) & (ringfifo->size-1));


}


