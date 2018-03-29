#include "hal_serial.h"
#include "hal_device.h"


 struct hal_serial_cfg_s serial_defauhal_cfg = {
	.buad_rate  = 115200,
	.data_bits  = DATA_BITS_8,
	.stop_bits  = STOP_BITS_1,
	.parity     = PARITY_NONE,
	.bit_order  = BIT_ORDER_LSB,
	.invert     = NRZ_NORMAL,
	.bufsz      = 0,
	.reserved   = 0,
};

serial_lock_t serial_lock_create(void)
{
	return (serial_lock_t)1;
}
int serial_lock(serial_lock_t lock,uint32_t milisec)
{
	return 0;
}
int serial_unlock(serial_lock_t lock)
{
	return 0;
}

void serial_buffer_init(struct serial_buffer_s *buffer)
{
	buffer->head = 0;
	buffer->tail = 0;
}

int serial_buffer_put(struct serial_buffer_s *buffer,char *ch,int size)
{
	int i;
	for(i = 0;i < size;i++){
		buffer->buffer[buffer->head % SERIAL_BUFFER_LENGTH] = ch[i];
		buffer->head++;
	}
	if(buffer->head > 65000){
		buffer->head -= 65000;
		buffer->tail -= 65000;
	}
	return i;
}

int serial_buffer_get(struct serial_buffer_s *buffer,char *ch,int size)
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
		ch[i] = buffer->buffer[buffer->tail % SERIAL_BUFFER_LENGTH];
		buffer->tail++;
	}
	return get_size;
}
int serial_buffer_length(struct serial_buffer_s *buffer)
{
	return buffer->head - buffer->tail;
}

static int serial_init( struct hal_dev_s *dev)
{
	int ret;
	struct hal_serial_s *serial = (struct hal_serial_s *)dev;
	
	if(serial == NULL)
		return -1;
	
	serial_buffer_init(&serial->recv_buffer);
	serial_buffer_init(&serial->send_buffer);
	
	if(serial->init != NULL){
		ret = serial->init(serial);
	}
	
	if(ret == 0){
		if(serial->configure != NULL){
			serial->configure(serial,&serial_defauhal_cfg);
		}
		
#ifdef CONFIG_SERIAL_USING_OS
#else			
		serial->tx_buf_empty = 1;
#endif
		serial->lock  = serial_lock_create();
		if(serial->lock == NULL){
			return -1;
		}
	}
	return ret;
}

static int serial_open( struct hal_dev_s *dev, uint16_t oflag)
{
	int ret = 0;
	struct hal_serial_s *serial = (struct hal_serial_s *)dev;
	
	if(serial == NULL)
		return -1;
	
	if(serial_lock(serial->lock,10) == 0){
		if((oflag & HAL_DEV_INT_TX) && !(dev->flag & HAL_DEV_INT_TX)){
			ret = -1;
		}
		if((oflag & HAL_DEV_INT_RX) && !(dev->flag & HAL_DEV_INT_RX)){
			ret = -1;
		}
		if((oflag & HAL_DEV_DMA_TX) && !(dev->flag & HAL_DEV_DMA_TX)){
			ret = -1;
		}
		if((oflag & HAL_DEV_DMA_RX) && !(dev->flag & HAL_DEV_DMA_RX)){
			ret = -1;
		}
	
		if(ret > 0)
			dev->open_flag = oflag & 0xff;
		
		serial_unlock(serial->lock);
	}
	return ret;
}

static int serial_close(struct hal_dev_s *dev)
{
	int ret = -1;
	struct hal_serial_s *serial = (struct hal_serial_s *)dev;
	
	if(serial == NULL)
		return -1;
	
	if(serial_lock(serial->lock,10) == 0){
		dev->open_flag = 0x00;
		serial_unlock(serial->lock);
		ret = 0;
	}
	return ret;
}

static int serial_read(struct hal_dev_s *dev, void *buffer, int buflen,int pos)
{   
	int i;
	int ret = -1;
	struct hal_serial_s *serial = (struct hal_serial_s *)dev;
	char *chbuf = (char*)buffer;
	
	if(serial == NULL)
		return -1;
	if(serial_lock(serial->lock,10) == 0){
		if(serial->dev.open_flag & HAL_DEV_DMA_RX){
			if(serial->dma_recv != NULL){
				serial->dma_recv(serial,buflen);
				ret =  serial_buffer_get(&serial->recv_buffer,chbuf,buflen);
			}
		}else if(serial->dev.open_flag & HAL_DEV_INT_RX){
				ret = serial_buffer_get(&serial->recv_buffer,chbuf,buflen);
		}else if(serial->dev.open_flag & HAL_DEV_BLOCK){
			if(serial->getc != NULL){
				serial->disable_irq(serial,SERIAL_INT_RXDONE);
				ret = 0;
				for(i = 0;i < buflen;i++){
					ret += serial->getc(serial,&chbuf[i]);
				}
				serial->enable_irq(serial,SERIAL_INT_RXDONE);
			}
		}
		serial_unlock(serial->lock);
	}
	return ret;
}

static int serial_write(struct hal_dev_s *dev, const void *buffer, int buflen,int pos)
{   
	int i;
	int ret = -1;
	struct hal_serial_s *serial = (struct hal_serial_s *)dev;
	char *chbuf = (char*)buffer;
	char ch;
	
	if(serial == NULL)
		return -1;
	
	if(serial_lock(serial->lock,10) == 0){
		if(serial->dev.open_flag & HAL_DEV_DMA_TX){
			if(serial->dma_send != NULL){
				serial->enable_irq(serial,SERIAL_INT_DMATXDONE);
				serial_buffer_put(&serial->send_buffer,chbuf,buflen);
#ifdef CONFIG_SERIAL_USING_OS
#else			
				while(serial->tx_buf_empty == 0);
                serial->tx_buf_empty = 0;
#endif
				ret = serial->dma_send(serial,buflen);
			}
		}else if(serial->dev.open_flag & HAL_DEV_INT_TX){
			
			serial_buffer_put(&serial->send_buffer,chbuf,buflen);
			serial->enable_irq(serial,SERIAL_INT_TXDONE);
#ifdef CONFIG_SERIAL_USING_OS
#else
			while(serial->tx_buf_empty == 0);
			serial->tx_buf_empty = 0;
			serial_buffer_get(&serial->send_buffer,&ch,1);	
			ret = serial->putc(serial,ch);
#endif
		}else if(serial->dev.open_flag & HAL_DEV_BLOCK){
			if(serial->putc != NULL){
				serial->disable_irq(serial,SERIAL_INT_TXDONE);
				ret = 0;
				for(i = 0;i < buflen;i++){
					ret += serial->putc(serial,chbuf[i]);
				}
				serial->enable_irq(serial,SERIAL_INT_TXDONE);
			}
		}
		serial_unlock(serial->lock);
	}
	return -1;
}

static int serial_ioctl(struct hal_dev_s *dev,uint8_t cmd,void *arg)
{
	struct hal_serial_s *serial = (struct hal_serial_s *)dev;
	struct hal_serial_cfg_s *cfg;
	int ret = -1;
	
	if(serial == NULL)
		return ret;
	
	switch(cmd){
		case HAL_DEV_CTRL_CONFIG:
			if(serial_lock(serial->lock,10) == 0){
				cfg = (struct hal_serial_cfg_s *)arg;
				if(cfg == NULL)
					ret = -1;
				if(serial->configure != NULL){
					ret = serial->configure(serial,cfg);
				}else{
					ret = -1;
				}
				serial_unlock(serial->lock);
			}
			break;
		default:
			break;
	}
	return ret;
}

void serial_isr(struct hal_serial_s *serial,int event)
{
	char ch;
	
	switch(event){
		case SERIAL_INT_TXDONE:
			if(serial_buffer_get(&serial->send_buffer,&ch,1) == 1){
				serial->putc(serial,ch);
				serial_buffer_put(&serial->recv_buffer,&ch,1);
			}else{
				/* all bytes in buffer are send by TC interrupt, disable tx done interrupt */
				serial->tx_buf_empty = 1;
				serial->disable_irq(serial,SERIAL_INT_TXDONE);
			}
			break;
		case SERIAL_INT_RXDONE:
			serial->getc(serial,&ch);
			serial_buffer_put(&serial->recv_buffer,&ch,1);
			break;
		case SERIAL_INT_DMATXDONE:
			serial->tx_buf_empty = 1;
			serial->disable_irq(serial,SERIAL_INT_DMATXDONE);
			break;
		default:
			break;
	};
}

void serial_device_register(struct hal_serial_s *serial,char *name,uint16_t flag)
{
	serial->dev.type   = hal_device_class_char;
	serial->dev.init   = serial_init;
	serial->dev.open   = serial_open;
	serial->dev.close  = serial_close;
	serial->dev.read   = serial_read;
	serial->dev.write  = serial_write;
	serial->dev.ioctl  = serial_ioctl;
	
	hal_dev_register(&serial->dev,name,flag);
}
