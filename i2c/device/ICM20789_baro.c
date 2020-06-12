#include "hal_i2c.h"
#include <stdbool.h>
#include <stdint.h>
#include <hal_sensor.h>

#define ICM20789_BARO_MEASURE_LP    0x609C
#define ICM20789_BARO_MEASURE_N     0x6825
#define ICM20789_BARO_MEASURE_LN    0x70DF
#define ICM20789_BARO_MEASURE_ULN   0x7866
#define ICM20789_BARO_RESET         0x805D
#define ICM20789_BARO_READ_ID       0xEFC8

struct icm20789_baro_report_s{
	int16_t mag[3];
	int16_t temp;
};

static void icm20789_baro_delay_us(uint32_t ms)
{
    volatile int i,j;
    for(i = ms;i > 0;i--){
        for(j = 20;j > 0;j--);
    }
}

int32_t icm20789_i2c_send_cmd(struct hal_dev_s *dev,uint16_t cmd)
{
	struct hal_i2c_msg_s msg[2];

	uint8_t buf[2];
	
	buf[0] = (uint8_t)(cmd / 256);
	buf[1] = (uint8_t)(cmd % 256);
	
	msg[0].buff   = buf;
	msg[0].length = 2;
	msg[0].flags  = HAL_I2C_WR;

	if(hal_i2c_transfer(dev,msg,1) > 0){
		return 1;
	}
	return 0;
}

int32_t icm20789_i2c_recv_data(struct hal_dev_s *dev,uint8_t *buf,uint8_t len)
{
	struct hal_i2c_msg_s msg[2];
	
	msg[0].buff   = buf;
	msg[0].length = len;
	msg[0].flags  = HAL_I2C_RD;

	if(hal_i2c_transfer(dev,msg,1) > 0){
		return 1;
	}
	return 0;
}


int icm20789_baro_init(struct hal_dev_s *dev)
{
	uint8_t buf[2];
	icm20789_i2c_send_cmd(dev,ICM20789_BARO_READ_ID);
	icm20789_i2c_recv_data(dev,buf,2);
	
	if((buf[1] & 0x3F) != 0x08){
		return -1;
	}
	
	return 0;
}

int icm20789_baro_open(struct hal_dev_s *dev, uint16_t oflag)
{
	return 0;
}

int icm20789_baro_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{
	struct baro_report_s *data = (struct baro_report_s *)buffer;
	uint8_t buff[6];
	
	if(size == sizeof(struct baro_report_s)){
		return -1;
	}
	
	
	return size;

}

int icm20789_baro_write(struct hal_dev_s *dev, const void *buffer, int size,int pos)
{
	return size;
}

int icm20789_baro_ioctl(struct hal_dev_s *dev, uint8_t cmd, void *args)
{
	return 0;
}

int icm20789_baro_close(struct hal_dev_s *dev)
{
	/* we can sleep mpu6050 here for saving power */
	return 0;
}

struct hal_i2c_dev_s icm20789_baro;
	
int icm20789_baro_register(void)
{
	icm20789_baro.port	  = 0;
	icm20789_baro.address   = 0x63;
	icm20789_baro.cfg.flags = HAL_I2C_WR | HAL_I2C_RD;
	icm20789_baro.cfg.speed = 400000;
	icm20789_baro.cfg.width = 8;

	icm20789_baro.dev.init  = icm20789_baro_init;
	icm20789_baro.dev.open  = icm20789_baro_open;
	icm20789_baro.dev.close = icm20789_baro_close;
	icm20789_baro.dev.read  = icm20789_baro_read;
	icm20789_baro.dev.write = icm20789_baro_write;
	icm20789_baro.dev.ioctl = icm20789_baro_ioctl;

	hal_i2c_device_register(&icm20789_baro,"icm20789-baro",HAL_O_RDWR | HAL_DEV_STANDALONE);
	return 0;
};

