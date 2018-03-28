/*
 * @file          device.c
 * @author        Gavin Liu
 * @version       v0.1
 * @brief         a implementation of device management,it is easy to port rtos environment.    
 *                
 * Change Logs:
 * Date           Author       Notes
 * 2016-05-22     Gavin Liu    release 
 */
 
#include "hal_device.h"
#include "string.h"
    
static struct hal_dev_s *device_address_list[HAL_DEV_NUM_MAX] = {0};
static struct hal_lock_s device_address_list_lock;

#ifndef HAL_DEV_USING_OS
int32_t hal_dev_lock_init(struct hal_lock_s *lock)
{
	lock->id = 0;
	return 0;
}
int32_t  hal_dev_lock(struct hal_lock_s *lock,uint32_t time_out_ms)
{
	lock->id = 1;
	return 0;
}
void  hal_dev_unlock(struct hal_lock_s *lock)
{
	lock->id = 0;
	
}
#endif

void hal_device_init(void){
	memset(device_address_list,NULL,sizeof(device_address_list));
	hal_dev_lock_init(&device_address_list_lock);
}

int32_t hal_dev_find(const char* name)
{
    int32_t i;
	int32_t ret = -1;
	
	hal_dev_lock(&device_address_list_lock,1000000);

    for(i = 0;i < HAL_DEV_NUM_MAX;i++){
        if(device_address_list[i] != NULL){
            if(strcmp(name,device_address_list[i]->name) == 0){
                ret = i;
                break;
            }
        }
    }
	hal_dev_unlock(&device_address_list_lock);
	return ret;
}

int32_t hal_dev_register(struct hal_dev_s *dev,const char *name,uint16_t flags)
{
    int32_t i;
	int32_t ret = -1;
    int32_t name_length = 0;
	
	if(hal_dev_find(name) >= 0){
		return -1;
	}
	
	hal_dev_lock(&device_address_list_lock,1000000); 
    for(i = 0;i < HAL_DEV_NUM_MAX;i++){
        if(device_address_list[i] == NULL){
            dev->flag = flags;
            device_address_list[i] = dev;
            name_length = strlen(name);
            name_length = name_length > HAL_DEV_NAME_LENGTH_MAX ?  HAL_DEV_NAME_LENGTH_MAX : name_length;
            strncpy(device_address_list[i]->name,name,name_length);
            ret = 0;
            break;
        }
    }
   
	hal_dev_unlock(&device_address_list_lock);
	return ret;
}
int32_t hal_dev_unregister(struct hal_dev_s *dev)
{
	int32_t i;
	int32_t ret = -1;
	
	hal_dev_lock(&device_address_list_lock,1000000);
    for(i = 0;i < HAL_DEV_NUM_MAX;i++){
        if(device_address_list[i] != NULL){
            if(device_address_list[i] == dev){
                device_address_list[i] = NULL;
                ret = 0;
                break;
            }
        }
    }
	hal_dev_unlock(&device_address_list_lock);
	return ret;
}


int32_t hal_dev_open(char *name,uint16_t oflag)
{
	struct hal_dev_s *dev = NULL;
	int32_t ret = 0;
	int32_t fd = -1;
	
	fd = hal_dev_find(name);
    
    if(fd < 0){
        return -1;
    }
    hal_dev_lock(&device_address_list_lock,1000000);
    dev = device_address_list[fd];
    
    if(!(dev->flag & HAL_DEV_ACTIVATED)){
        if(dev->init != NULL){
            hal_dev_unlock(&device_address_list_lock);
            ret = dev->init(dev);
            hal_dev_lock(&device_address_list_lock,1000000);
            if(ret != 0){
                fd = ret;
                goto _out;
            }else{
                dev->flag |= HAL_DEV_ACTIVATED;
            }
        }else{
            fd = -1;
            goto _out;
        }
        
    }
    if((dev->flag & HAL_DEV_STANDALONE)){
        if(dev->ref_count > 0){
            fd = -2;
            goto _out;
        }
    }
    
    if(dev->open != NULL){
        hal_dev_unlock(&device_address_list_lock);
        ret = dev->open(dev,oflag);
        hal_dev_lock(&device_address_list_lock,1000000);
        if(ret == 0){
            dev->ref_count++;
            dev->open_flag |= oflag;
            
        }else{
            fd = -1;
        }
    }else{
        fd = -1;
    }
    
_out:
    hal_dev_unlock(&device_address_list_lock);
    return fd;
	
}
int32_t hal_dev_close(int32_t fd)
{
	int32_t ret = 0;
	struct hal_dev_s *dev = NULL;
	
	if(fd >= HAL_DEV_NAME_LENGTH_MAX){
		return -1;
	}
	hal_dev_lock(&device_address_list_lock,1000000);
	dev = device_address_list[fd];
	
	if(dev == NULL || dev->close == NULL){
		return -1;
	}  
	if(dev->ref_count > 0){
		dev->ref_count--;
		if(dev->ref_count == 0){
			ret = dev->close(dev);
			dev->open_flag = 0x00;
		}else{
            ret = 0;
        }
	}
    
    hal_dev_unlock(&device_address_list_lock);
	return ret;
}

int32_t hal_dev_read(int32_t fd, void *buffer, int32_t size,int32_t pos)
{
	struct hal_dev_s *dev = NULL;
	
	if(fd >= HAL_DEV_NUM_MAX || fd < 0){
		return -1;
	}
	hal_dev_lock(&device_address_list_lock,1000000);
	dev = device_address_list[fd];
	hal_dev_unlock(&device_address_list_lock);
	if(dev == NULL){
		return -1;
	}
	
	if(dev->ref_count > 0){
		if(dev->read != NULL){
			return dev->read(dev,buffer,size,pos);
		}			
	}else{
		return -1;
	}
	return -1;
}

int32_t hal_dev_write(int32_t fd, const void *buffer, int32_t size,int32_t pos)
{
	struct hal_dev_s *dev = NULL;
	
	if(fd >= HAL_DEV_NUM_MAX){
		return -1;
	}
	hal_dev_lock(&device_address_list_lock,1000000);
	dev = device_address_list[fd];
	hal_dev_unlock(&device_address_list_lock);
	if(dev == NULL){
		return -1;
	}
	
	if(dev->ref_count > 0){
		if(dev->write != NULL){
			return dev->write(dev,buffer,size,pos);
		}			
	}else{
		return -1;
	}
	return -1;
}

int32_t hal_dev_ioctl(int32_t fd, uint8_t cmd, void *args)
{
	struct hal_dev_s *dev = NULL;
	
	if(fd >= HAL_DEV_NUM_MAX){
		return -1;
	}
	hal_dev_lock(&device_address_list_lock,1000000);
	dev = device_address_list[fd];
	hal_dev_unlock(&device_address_list_lock);
	if(dev == NULL){
		return -1;
	}
	
	if(dev->ref_count > 0){
		if(dev->ioctl != NULL){
			return dev->ioctl(dev,cmd,args);
		}			
	}else{
		return -1;
	}
	return -1;
}

