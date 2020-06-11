#ifndef _RINGBUFFER_H_
#define _RINGBUFFER_H_


struct ring_buffer_s
{
    int            inited;
	volatile int   head;   /* Index to the head [IN] index in the buffer */
	volatile int   tail;   /* Index to the tail [OUT] index in the buffer */
	int            size;   /* The allocated size of the buffer */
	char          *buffer; /* Pointer to the allocated buffer memory */
};

void ring_buffer_init(struct ring_buffer_s *buffer,char *mem,int size);
int  ring_buffer_put(struct ring_buffer_s *buffer,char *ch,int size);
int  ring_buffer_get(struct ring_buffer_s *buffer,char *ch,int size);


#endif
