#include "ringbuffer.h"

void ring_buffer_init(struct ring_buffer_s *buffer,char *mem,int size)
{
	buffer->head = 0;
	buffer->tail = 0;
	buffer->buffer = mem;
	buffer->size = size;
}

int ring_buffer_put(struct ring_buffer_s *buffer,char *ch,int size)
{
	int i;
	int buffer_size = buffer->head - buffer->tail;

    if(buffer->size < 1){
		return 0;
	}
    if(size > (buffer->size  - buffer_size)){
        //printf("buffer overflowed %d %d %d %d %d \r\n",size,buffer->size,buffer_size,buffer->head ,buffer->tail);
        return 0;
    }
	
	for(i = 0;i < size;i++){
		buffer->buffer[buffer->head % buffer->size] = ch[i];
		buffer->head++;
	}
	if(buffer->head > 104857600){
		buffer->head -= 104857600;
		buffer->tail -= 104857600;
	}
	return i;
}

int ring_buffer_get(struct ring_buffer_s *buffer,char *ch,int size)
{
	int i;
	int buffer_size = buffer->head - buffer->tail;
	int get_size;
	
	if(buffer_size < 0){
		buffer->head = 0;
		buffer->tail = 0;
		return -1;
	}
	
	get_size = buffer_size > size ? size : buffer_size;
	
	for(i = 0;i < get_size;i++){
		ch[i] = buffer->buffer[buffer->tail % buffer->size];
		buffer->tail++;
	}
	
	return get_size;
}

int ring_buffer_length(struct ring_buffer_s *buffer)
{
	return buffer->head - buffer->tail;
}

