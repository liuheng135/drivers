#include "hal_spi.h"

static struct hal_spi_adapter_s *spi_adapter_list[SPI_ADAPTER_NUM_MAX];
static struct hal_lock_s spi_adapter_list_lock;

int hal_spi_init(void)
{
	int i;
    
    hal_dev_lock_init(&spi_adapter_list_lock);

	hal_dev_lock(&spi_adapter_list_lock,1000);
	for(i = 0;i < SPI_ADAPTER_NUM_MAX;i++){
		spi_adapter_list[i] = (struct hal_spi_adapter_s *)0;
	}
	hal_dev_unlock(&spi_adapter_list_lock);
	
	return 0;
}

int hal_spi_adapter_register(struct hal_spi_adapter_s *adpt,int port)
{
	int ret = -1;

	if(hal_dev_lock(&spi_adapter_list_lock,5000) == 0){
		if(spi_adapter_list[port] == 0){
			hal_dev_lock_init(&adpt->lock);
			ret = adpt->init(adpt);
			spi_adapter_list[port] = adpt;
		}else{
			ret = -1;
		}
	}else{
		ret = -1;
	}
	hal_dev_unlock(&spi_adapter_list_lock);

	return ret;
}

int hal_spi_device_register(struct hal_spi_dev_s *spi_dev,char *name,uint16_t flags)
{
	if(spi_dev == NULL){
		return -1;
	}
	spi_dev->dev.type = hal_device_class_spislave;
	return hal_dev_register(&(spi_dev->dev),name,flags);
}

int hal_spi_transfer(struct hal_dev_s *dev,struct hal_spi_msg_s *msg)
{
	int transferd = 0;
	struct hal_spi_dev_s  *spi_dev = (struct hal_spi_dev_s *)dev;
	struct hal_spi_adapter_s *adpt;
	
	if(dev == NULL){
		return -1;
	}
	
	if(dev->type != hal_device_class_spislave){
		HAL_DEBUG("[SPI] %s is not a spi slave device\n",dev->name);
		return -1;
	}
	
	
	if(spi_dev->port < -1 && spi_dev->port > SPI_ADAPTER_NUM_MAX){
		return -1;
	}
	if(hal_dev_lock(&spi_adapter_list_lock,1000) == 0){
        adpt = spi_adapter_list[spi_dev->port];
        hal_dev_unlock(&spi_adapter_list_lock);
    }else{
        return -1;
    }
	
	if(adpt == NULL){
		return -1;
	}
	
	if(hal_dev_lock(&adpt->lock,10) == 0){
		if(adpt->owner != spi_dev){
			adpt->owner = spi_dev;
			adpt->configure(adpt,&(spi_dev->cfg));
		}
		
		if(msg->cs_take == 1){
			spi_dev->cs_take();
		}
		transferd = adpt->transfer(adpt,msg);
		if(msg->cs_release == 1){
			spi_dev->cs_release();
		}
		
		hal_dev_unlock(&adpt->lock);
		return transferd;
	}else{
		return -1;
	}
	
}

int hal_spi_configure(struct hal_dev_s *dev,struct hal_spi_cfg_s *cfg)
{
	struct hal_spi_dev_s  *spi_dev = (struct hal_spi_dev_s *)dev;
	struct hal_spi_adapter_s *adpt;
	if(spi_dev == NULL){
		return -1;
	}
	if(spi_dev->port < -1 && spi_dev->port > SPI_ADAPTER_NUM_MAX){
		return -1;
	}
    
	if(hal_dev_lock(&spi_adapter_list_lock,1000) == 0){
        adpt = spi_adapter_list[spi_dev->port];
        hal_dev_unlock(&spi_adapter_list_lock);
    }else{
        return -1;
    }
    
	if(hal_dev_lock(&adpt->lock,10) == 0){
		adpt->configure(adpt,&(spi_dev->cfg));
		if(adpt->owner != spi_dev){
			adpt->owner = spi_dev;
		}
		hal_dev_unlock(&adpt->lock);
		return 0;
	}
	return -1;
}

int hal_spi_write_then_read(struct hal_dev_s *dev,
		void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	struct hal_spi_msg_s msg;
	msg.cs_take      = 1;
	msg.cs_release   = 0;
	msg.send_buffer  = (unsigned char *)txbuf;
	msg.recv_buffer  = NULL; 
	msg.length       = n_tx;
	
	hal_spi_transfer(dev,&msg);
	
	msg.cs_take      = 0;
	msg.cs_release   = 1;
	msg.send_buffer  = NULL;
	msg.recv_buffer  = (unsigned char *)rxbuf;
	msg.length       = n_rx;
	
	return hal_spi_transfer(dev,&msg);
}

int hal_spi_write_then_write(struct hal_dev_s *dev,
		void *txbuf1, unsigned n_tx1,
		void *txbuf2, unsigned n_tx2)
{
	struct hal_spi_msg_s msg;
	msg.cs_take      = 1;
	msg.cs_release   = 0;
	msg.send_buffer  = (unsigned char *)txbuf1;
	msg.recv_buffer  = NULL; 
	msg.length       = n_tx1;
	
	hal_spi_transfer(dev,&msg);
	
	msg.cs_take      = 0;
	msg.cs_release   = 1;
	msg.send_buffer  = (unsigned char *)txbuf2;
	msg.recv_buffer  = NULL; 
	msg.length       = n_tx2;
	
	return hal_spi_transfer(dev,&msg);
}

int hal_spi_write(struct hal_dev_s *dev,void *buf, unsigned n_tx)
{
	struct hal_spi_msg_s msg;
	msg.cs_take      = 1;
	msg.cs_release   = 1;
	msg.send_buffer  = (unsigned char *)buf;
	msg.recv_buffer  = NULL; 
	msg.length       = n_tx;
	
	return hal_spi_transfer(dev,&msg);
}

int hal_spi_read(struct hal_dev_s *dev,void *buf, unsigned n_rx)
{
	struct hal_spi_msg_s msg;
	msg.cs_take      = 1;
	msg.cs_release   = 1;
	msg.send_buffer  = NULL;
	msg.recv_buffer  = (unsigned char *)buf;
	msg.length       = n_rx;
	
	return hal_spi_transfer(dev,&msg);
}
