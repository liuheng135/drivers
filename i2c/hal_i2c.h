#ifndef _HAL_I2C_H_
#define _HAL_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_I2C_ADAPTER_NUM_MAX  5

#include "hal_device.h"

#define HAL_I2C_WR                 0x0000
#define HAL_I2C_RD                 0x0001
#define HAL_I2C_ADDR_10BIT         0x0002
#define HAL_I2C_NO_START           0x0004
#define HAL_I2C_IGNORE_NACK        0x0008
#define HAL_I2C_NO_READ_ACK        0x0010

#define HAL_I2C_MSG_LEN_MAX        128   /* This should not be larger than 2048*/

/**
 * I2C configuration structure
 */
struct hal_i2c_cfg_s{
	int32_t  speed;
	int32_t  width;
	uint16_t flags;
};

/**
 * I2C message
 */
struct hal_i2c_msg_s{
	uint8_t *buff;
	uint16_t addr;
    int      flags;
	int      length;
};

struct hal_i2c_dev_s{
	struct hal_dev_s      dev;
	int                   port;
	unsigned short        address;
	struct hal_i2c_cfg_s  cfg;
	void                 *user_data;
};

struct hal_i2c_adapter_s{
	int                  (*init)     (struct hal_i2c_adapter_s *adpt);
	int                  (*configure)(struct hal_i2c_adapter_s *adpt,struct hal_i2c_cfg_s *cfg);
	int                  (*transfer) (struct hal_i2c_adapter_s *adpt,struct hal_i2c_msg_s *msg,int num);
	void                 (*exit)     (struct hal_i2c_adapter_s *adpt);
	struct hal_i2c_dev_s  *owner;
	struct hal_lock_s     lock;
	void                  *user_data;
};

/*  for i2c adapter drivers */
void    hal_i2c_init(void);
int32_t hal_i2c_adapter_register(struct hal_i2c_adapter_s *adpt,int port);
int32_t hal_i2c_adapter_unregister(int port);

/*  for i2c slave address drivers */
int32_t hal_i2c_device_register(struct hal_i2c_dev_s *i2c_dev,char *name,uint16_t flags);
int     hal_i2c_transfer(struct hal_dev_s *dev,struct hal_i2c_msg_s *msg,int num);
int32_t hal_i2c_configure(struct hal_dev_s *dev,struct hal_i2c_cfg_s *cfg);

int32_t hal_i2c_master_send(struct hal_dev_s *dev,uint8_t *buffer,int32_t size);
int32_t hal_i2c_master_recv(struct hal_dev_s *dev,uint8_t *buffer,int32_t size);
int32_t hal_i2c_read_from_addr(struct hal_dev_s *dev,uint8_t addr,uint8_t *buffer,int32_t size);
int32_t hal_i2c_write_to_addr(struct hal_dev_s *dev,uint8_t addr,uint8_t *buffer,int32_t size);

#ifdef __cplusplus
}
#endif

#endif
