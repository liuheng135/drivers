#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "hal_device.h"

#ifndef NULL
#define NULL 0
#endif

typedef int serial_lock_t;

#define SERIAL_BUFFER_LENGTH  64

#define DATA_BITS_5                     5
#define DATA_BITS_6                     6
#define DATA_BITS_7                     7
#define DATA_BITS_8                     8
#define DATA_BITS_9                     9

#define STOP_BITS_1                     0
#define STOP_BITS_2                     1
#define STOP_BITS_3                     2
#define STOP_BITS_4                     3

#define PARITY_NONE                     0
#define PARITY_ODD                      1
#define PARITY_EVEN                     2

#define BIT_ORDER_LSB                   0
#define BIT_ORDER_MSB                   1

#define NRZ_NORMAL                      0       /* Non Return to Zero : normal mode */
#define NRZ_INVERTED                    1       /* Non Return to Zero : inverted mode */

#define SERIAL_INT_TXDONE     			(uint8_t)0x01
#define SERIAL_INT_RXDONE 				(uint8_t)0x02
#define SERIAL_INT_DMATXDONE			(uint8_t)0x03
#define SERIAL_INT_DMARXDONE			(uint8_t)0x04

struct serial_buffer_s
{
	volatile unsigned short   head;   /* Index to the head [IN] index in the buffer */
	volatile unsigned short   tail;   /* Index to the tail [OUT] index in the buffer */
	short            size;   /* The allocated size of the buffer */
	char             buffer[SERIAL_BUFFER_LENGTH]; /* Pointer to the allocated buffer memory */
};

struct hal_serial_cfg_s
{
	unsigned int buad_rate                 ;
    unsigned int data_bits               :4;
    unsigned int stop_bits               :2;
    unsigned int parity                  :2;
    unsigned int bit_order               :1;
    unsigned int invert                  :1;
	unsigned int bufsz					 :16;
    unsigned int reserved                :4;
};

extern struct hal_serial_cfg_s serial_defauhal_cfg;

struct hal_serial_s
{
	struct   hal_dev_s dev;
#ifdef CONFIG_SERIAL_TERMIOS
	/* Terminal control flags */

	tcflag_t             tc_iflag;     /* Input modes */
	tcflag_t             tc_oflag;     /* Output modes */
	tcflag_t             tc_lflag;     /* Local modes */
#endif
	serial_lock_t        lock;
	
#ifdef CONFIG_SERIAL_USING_OS

#else
	int                  tx_buf_empty;
#endif
	
	struct serial_buffer_s recv_buffer;
	struct serial_buffer_s send_buffer;
	
	int (*init)(struct hal_serial_s *serial);
    int (*configure)(struct hal_serial_s *serial,struct hal_serial_cfg_s *cfg);

	/* for poll */
    int (*putc)(struct hal_serial_s *serial, char ch);
    int (*getc)(struct hal_serial_s *serial, char *ch);

	/* for dma */
    int (*dma_send)(struct hal_serial_s *serial, int size);
	int (*dma_recv)(struct hal_serial_s *serial, int size);
	
	/* for irq */
	void (*enable_irq)(struct hal_serial_s *serial,int irq);
	void (*disable_irq)(struct hal_serial_s *serial,int irq);
	void            *priv;         /* Used by the arch-specific logic */
};

void serial_buffer_init(struct serial_buffer_s *buffer);
int  serial_buffer_put(struct serial_buffer_s *buffer,char *ch,int size);
int  serial_buffer_get(struct serial_buffer_s *buffer,char *ch,int size);
void serial_device_register(struct hal_serial_s *serial,char *name,uint16_t flag);
void serial_isr(struct hal_serial_s *serial,int event);

#endif
