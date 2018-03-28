#include "hal_i2c.h"

static struct hal_i2c_adapter_s *hal_i2c_adapter_list[HAL_I2C_ADAPTER_NUM_MAX];
static struct hal_lock_s  hal_i2c_adapter_list_lock;

void hal_i2c_init(void)
{
	int i;
	hal_dev_lock_init(&hal_i2c_adapter_list_lock);
	if(hal_dev_lock(&hal_i2c_adapter_list_lock,5000) == 0){
		for(i = 0;i < HAL_I2C_ADAPTER_NUM_MAX;i++){
			hal_i2c_adapter_list[i] = (struct hal_i2c_adapter_s *)NULL;
		}
		hal_dev_unlock(&hal_i2c_adapter_list_lock);
	}
}

int32_t hal_i2c_adapter_register(struct hal_i2c_adapter_s *adpt,int port)
{
	int ret = -1;
	if(hal_dev_lock(&hal_i2c_adapter_list_lock,5000) == 0){
		if(hal_i2c_adapter_list[port] == (struct hal_i2c_adapter_s *)NULL){
			hal_dev_lock_init(&(adpt->lock ));
	        ret = adpt->init(adpt);
	        hal_i2c_adapter_list[port] = adpt;
		}else{
			HAL_DEBUG("[I2C] I2C%d is registered before\n",port);
			ret = -1;
		}
		hal_dev_unlock(&hal_i2c_adapter_list_lock);
	}
	return ret;
}

int32_t hal_i2c_adapter_unregister(int port)
{
	int ret = -1;
	if(hal_dev_lock(&hal_i2c_adapter_list_lock,5000) == 0){
		if(hal_i2c_adapter_list[port] == (struct hal_i2c_adapter_s *)NULL){
			HAL_DEBUG("[I2C] I2C%d is already null\n",port);
		}else{
			hal_i2c_adapter_list[port] = (struct hal_i2c_adapter_s *)NULL;
			ret = -1;
		}
		hal_dev_unlock(&hal_i2c_adapter_list_lock);
	}
	return ret;
}



int32_t hal_i2c_device_register(struct hal_i2c_dev_s *i2c_dev,char *name,uint16_t flags)
{
	if(i2c_dev == NULL){
		return -1;
	}
	i2c_dev->dev.type = hal_device_class_i2cslave;
	return hal_dev_register(&(i2c_dev->dev),name,flags);
}

int hal_i2c_transfer(struct hal_dev_s *dev,struct hal_i2c_msg_s *msg,int num)
{
	int transferd = 0;
	int i;
	struct hal_i2c_dev_s  *i2c_dev = (struct hal_i2c_dev_s *)dev;
	struct hal_i2c_adapter_s *adpt;
	
	if(dev == NULL){
		HAL_DEBUG("[I2C] I2C device is null\n");
		return -1;
	}

	if(dev->type != hal_device_class_i2cslave){
		HAL_DEBUG("[I2C] %s is not a i2c slave device\n",dev->name);
		return -1;
	}
	
	if(i2c_dev->port < -1 && i2c_dev->port > HAL_I2C_ADAPTER_NUM_MAX){
		HAL_DEBUG("[I2C] I2C port is out of range\n");
		return -1;
	}
	if(hal_dev_lock(&hal_i2c_adapter_list_lock,5000) == 0){
		adpt = hal_i2c_adapter_list[i2c_dev->port];
		hal_dev_unlock(&hal_i2c_adapter_list_lock);
	}else{
		HAL_DEBUG("[I2C] lock hal_i2c_adapter_list_lock failed\n");
		return -1;
	}
	if(adpt == NULL){
		HAL_DEBUG("[I2C] I2C%d adapter can not be found\n",i2c_dev->port);
		return -1;
	}
	
	if(hal_dev_lock(&(adpt->lock),5000) == 0){
        if(adpt->owner != i2c_dev){
            adpt->owner = i2c_dev;
			if(adpt->configure != NULL){
            	adpt->configure(adpt,&(i2c_dev->cfg));
			}
        }
        for(i = 0;i < num;i++){
			msg[i].addr = i2c_dev->address;
        }
        transferd = adpt->transfer(adpt,msg,num);
        
        hal_dev_unlock(&(adpt->lock));
        return transferd;
    }else{
		HAL_DEBUG("[I2C] lock I2C%d adapter failed\n",i2c_dev->port);
    }
    return -1;
	
}

int hal_i2c_configure(struct hal_dev_s *dev,struct hal_i2c_cfg_s *cfg)
{
	struct hal_i2c_dev_s  *i2c_dev = (struct hal_i2c_dev_s *)dev;
	struct hal_i2c_adapter_s *adpt;
	if(i2c_dev == NULL){
		HAL_DEBUG("[I2C] I2C device is null\n");
		return -1;
	}
	if(i2c_dev->port < -1 && i2c_dev->port > HAL_I2C_ADAPTER_NUM_MAX){
		HAL_DEBUG("[I2C] I2C port is out of range\n");
		return -1;
	}
	if(hal_dev_lock(&hal_i2c_adapter_list_lock,5000) == 0){
		adpt = hal_i2c_adapter_list[i2c_dev->port];
		hal_dev_unlock(&hal_i2c_adapter_list_lock);
	}else{
		HAL_DEBUG("[I2C] lock hal_i2c_adapter_list_lock failed\n");
		return -1;
	}
	if(hal_dev_lock(&(adpt->lock),5000) == 0){
        adpt->configure(adpt,&(i2c_dev->cfg));
        if(adpt->owner != i2c_dev){
            adpt->owner = i2c_dev;
        }
        hal_dev_unlock(&(adpt->lock));
        return 0;
    }
    return -1;
}


int32_t hal_i2c_master_send(struct hal_dev_s *dev,uint8_t *buffer,int32_t size)
{
	struct hal_i2c_msg_s msg;

	msg.buff   = buffer;
	msg.length = size;
	msg.flags  = HAL_I2C_WR;

	return hal_i2c_transfer(dev,&msg,1) * size;
	
}

int32_t hal_i2c_master_recv(struct hal_dev_s *dev,uint8_t *buffer,int32_t size)
{
	struct hal_i2c_msg_s msg;

	msg.buff   = buffer;
	msg.length = size;
	msg.flags  = HAL_I2C_RD;

	return hal_i2c_transfer(dev,&msg,1) * size;
}

int32_t hal_i2c_read_from_addr(struct hal_dev_s *dev,uint8_t addr,uint8_t *buffer,int32_t size)
{
	struct hal_i2c_msg_s msg[2];

	uint8_t reg = addr;
	
	msg[0].buff   = &reg;
	msg[0].length = 1;
	msg[0].flags  = HAL_I2C_WR;


	msg[1].buff   = buffer;
	msg[1].length = size;
	msg[1].flags  = HAL_I2C_RD;

	if(hal_i2c_transfer(dev,msg,2) > 0){
		return size;
	}
	return 0;
}

int32_t hal_i2c_write_to_addr(struct hal_dev_s *dev,uint8_t addr,uint8_t *buffer,int32_t size)
{
	struct hal_i2c_msg_s msg;
	int32_t write_len = 0;
	uint8_t write_buffer[HAL_I2C_MSG_LEN_MAX +1];
	int i;

	write_len = size > HAL_I2C_MSG_LEN_MAX ? HAL_I2C_MSG_LEN_MAX : size;

	write_buffer[0] = addr;
	for(i = 0;i < write_len;i++){
		write_buffer[i+1] = buffer[i];
	}
	
	msg.buff   = write_buffer;
	msg.length = write_len+1;
	msg.flags  = HAL_I2C_WR;

	return hal_i2c_transfer(dev,&msg,1) * size;
}

