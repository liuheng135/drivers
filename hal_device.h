#ifndef _HAL_DEVICE_H_
#define _HAL_DEVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdio.h>


#define HAL_DEV_NAME_LENGTH_MAX   16
#define	HAL_DEV_NUM_MAX           32

		
#define HAL_O_RDONLY             (uint16_t)0x001         
#define HAL_O_WRONLY             (uint16_t)0x002         
#define HAL_O_RDWR	             (uint16_t)0x003         
	
#define HAL_DEV_CLOSE            (uint16_t)0x0000          /**< device is closed */	
#define HAL_DEV_RDONLY           (uint16_t)0x0001          /**< read only access */	
#define HAL_DEV_WRONLY           (uint16_t)0x0002          /**< write only access */	
#define HAL_DEV_RDWR             (uint16_t)0x0003          /**< read and write */
#define HAL_DEV_BLOCK            (uint16_t)0x0004          /**< block */
	
#define HAL_DEV_REMOVABLE        (uint16_t)0x0008          /**< removable device */	
#define HAL_DEV_STANDALONE       (uint16_t)0x0010          /**< standalone device */	
#define HAL_DEV_ACTIVATED        (uint16_t)0x0020          /**< device is activated */	
#define HAL_DEV_SUSPENDED        (uint16_t)0x0040          /**< device is suspended */	
#define HAL_DEV_STREAM           (uint16_t)0x0080          /**< stream mode */	
#define HAL_DEV_INT_RX           (uint16_t)0x0100          /**< INT mode on Rx */	
#define HAL_DEV_DMA_RX           (uint16_t)0x0200          /**< DMA mode on Rx */	
#define HAL_DEV_INT_TX           (uint16_t)0x0400          /**< INT mode on Tx */	
#define HAL_DEV_DMA_TX           (uint16_t)0x0800          /**< DMA mode on Tx */
	
#define HAL_DEV_CTRL_CONFIG           (uint16_t)0x0001    		/* configure device */	
#define HAL_DEV_CTRL_SET_INT          (uint16_t)0x0002    		/* enable receive irq */	
#define HAL_DEV_CTRL_CLR_INT          (uint16_t)0x0004    		/* disable receive irq */	
#define HAL_DEV_CTRL_GET_INT          (uint16_t)0x0008	
#define HAL_DEV_CTRL_BLK_GETGEOME     (uint16_t)0x0010


//#define HAL_DEBUG(s,...)	{printf(s,## __VA_ARGS__);}
#define HAL_DEBUG(s,...)
	
enum hal_device_class_type
{
    hal_device_class_char = 0,                          
    hal_device_class_block,                              
    hal_device_class_netIf,   
    hal_device_class_i2cbus,
    hal_device_class_spibus,
    hal_device_class_i2cslave,
    hal_device_class_spislave,
};

struct hal_lock_s{
	int id;
};

struct hal_dev_s
{
	char name[HAL_DEV_NAME_LENGTH_MAX];
    enum hal_device_class_type type;                     /**< device type */
    uint16_t                   flag;                     /**< device flag */
    uint16_t                   open_flag;                /**< device open flag */

    uint8_t                	   ref_count;                /**< reference count */
    uint8_t                	   device_id;                /**< 0 - 255 */


    /* common device interface */
    int32_t  (*init)   (struct hal_dev_s *dev);
    int32_t  (*open)   (struct hal_dev_s *dev, uint16_t oflag);
    int32_t  (*close)  (struct hal_dev_s *dev);
    int32_t  (*read)   (struct hal_dev_s *dev, void *buffer, int size,int pos);
    int32_t  (*write)  (struct hal_dev_s *dev, const void *buffer, int size,int pos);
    int32_t  (*ioctl)  (struct hal_dev_s *dev, uint8_t cmd, void *args);

    void                     *priv_data;                /**<  private data */
};

struct hal_device_node_s
{
	struct hal_dev_s *device;
	struct hal_device_node_s *next;
};


int32_t  hal_dev_lock_init(struct hal_lock_s *lock);
int32_t  hal_dev_lock(struct hal_lock_s *lock,uint32_t time_out_ms);
void     hal_dev_unlock(struct hal_lock_s *lock);

int32_t  hal_dev_register(struct hal_dev_s *dev,const char *name,uint16_t flags);
int32_t  hal_dev_unregister(struct hal_dev_s *dev);


void     hal_dev_init(void);
int32_t  hal_dev_open(char *name,uint16_t oflag);
int32_t  hal_dev_close(int32_t fd);
int32_t  hal_dev_read(int32_t fd, void *buffer, int32_t size,int32_t pos);
int32_t  hal_dev_write(int32_t fd, const void *buffer, int32_t size,int32_t pos);
int32_t  hal_dev_ioctl(int32_t fd, uint8_t cmd, void *args);

#ifdef __cplusplus
}
#endif

#endif

