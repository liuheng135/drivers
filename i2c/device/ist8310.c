#include "hal_i2c.h"
#include <stdbool.h>
#include <stdint.h>
#include <hal_sensor.h>

#define SENSOR_NAME "ist8310"
#define SENSOR_I2C_SLAVE_ADDRESS 0x0E
#define IST8310_DATA_UPDATE_TIME 0.005


#define STM8s_ID_VAL	0xFF
#define IST8310_WHO_AM_I           0x00
#define IST8310_WHO_AM_I_VALUE     0x10

#define IST8310_STATUS_REGISTER1   0x02
#define STAT1_DRDY_SHFITS      0x0
#define STAT1_DRDY             (1 << STAT1_DRDY_SHFITS)
#define STAT1_DRO_SHFITS       0x1
#define STAT1_DRO              (1 << STAT1_DRO_SHFITS)

#define IST8310_OUTPUT_VALUE_X_L   0x03
#define IST8310_OUTPUT_VALUE_X_H   0x04
#define IST8310_OUTPUT_VALUE_Y_L   0x05
#define IST8310_OUTPUT_VALUE_Y_H   0x06
#define IST8310_OUTPUT_VALUE_Z_L   0x07
#define IST8310_OUTPUT_VALUE_Z_H   0x08

#define IST8310_STATUS_REGISTER2   0x09
#define STAT2_INT_SHFITS       3
#define STAT2_INT              (1 << STAT2_INT_SHFITS)

#define IST8310_CONTROL_REGISTER1  0x0A
#define CTRL1_MODE_SHFITS      0
#define CTRL1_MODE_STDBY       (0 << CTRL1_MODE_SHFITS)
#define CTRL1_MODE_SINGLE      (1 << CTRL1_MODE_SHFITS)

#define IST8310_CONTROL_REGISTER2  0x0B
#define CTRL2_SRST_SHFITS      0   /* Begin POR (auto cleared) */
#define CTRL2_SRST             (1 << CTRL2_SRST_SHFITS)
#define CTRL2_DRP_SHIFTS       2
#define CTRL2_DRP              (1 << CTRL2_DRP_SHIFTS)
#define CTRL2_DREN_SHIFTS      3
#define CTRL2_DREN             (1 << CTRL2_DREN_SHIFTS)

#define IST8310_STR                0x0c
#define STR_SELF_TEST_SHFITS   6
#define STR_SELF_TEST_ON       (1 << STR_SELF_TEST_SHFITS)
#define STR_SELF_TEST_OFF      (0 << STR_SELF_TEST_SHFITS)

#define IST8310_OUTPUT_VALUE_T_L   0x1C
#define IST8310_OUTPUT_VALUE_T_H   0x1D

#define IST8310_CONTROL_REGISTER3  0x41
#define CTRL3_SAMPLEAVG_16		0x24	
#define CTRL3_SAMPLEAVG_8		0x1b	
#define CTRL3_SAMPLEAVG_4		0x12	
#define CTRL3_SAMPLEAVG_2		0x09
#define CTRL3_SAMPLEAVG_0		0x00

#define IST8310_CONTROL_REGISTER4  0x42
#define CTRL4_SRPD				0xC0

struct ist8310_report_s{
	int16_t mag[3];
	int16_t temp;
};

static void ist8310_delay_us(uint32_t ms)
{
    volatile int i,j;
    for(i = ms;i > 0;i--){
        for(j = 20;j > 0;j--);
    }
}

int ist8310_init(struct hal_dev_s *dev)
{
	uint8_t reg_val;
	hal_i2c_read_from_addr(dev,IST8310_WHO_AM_I,&reg_val,1);
	if(reg_val != IST8310_WHO_AM_I_VALUE){
		return -1;
	}
	
	reg_val = CTRL4_SRPD;
	hal_i2c_write_to_addr(dev,IST8310_CONTROL_REGISTER4, &reg_val,1); //enter standby
	ist8310_delay_us(2000);
	reg_val = CTRL1_MODE_SINGLE;
	hal_i2c_write_to_addr(dev,IST8310_CONTROL_REGISTER1, &reg_val,1); //enter standby
	reg_val = CTRL3_SAMPLEAVG_0;
	hal_i2c_write_to_addr(dev,IST8310_CONTROL_REGISTER3, &reg_val,1); //enter standby
	
	return 0;

}

int ist8310_open(struct hal_dev_s *dev, uint16_t oflag)
{
	return 0;
}

int ist8310_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{
	struct mag_report_s *data = (struct mag_report_s *)buffer;
	uint8_t buff[6];
	
	if(size == sizeof(struct mag_report_s)){
		return -1;
	}
	
	hal_i2c_read_from_addr(dev,IST8310_STATUS_REGISTER1,buff,1);
	
	if(buff[0] != STAT1_DRDY){
		return 0;
	}
	
	hal_i2c_read_from_addr(dev, IST8310_OUTPUT_VALUE_X_L,buff,6);
	data->data[0]  = (float)(buff[1]  << 8|buff[0]) * 3.0f;	
	data->data[1]  = (float)(buff[3]  << 8|buff[2]) * 3.0f;	
	data->data[2]  = (float)(buff[5]  << 8|buff[4]) * 3.0f;	
	hal_i2c_read_from_addr(dev, IST8310_OUTPUT_VALUE_T_L,buff,2);
	data->temperature  = (float)(buff[1]  << 8|buff[0]) * 0.1f;	

	buff[0] = CTRL1_MODE_SINGLE;
	hal_i2c_write_to_addr(dev,IST8310_CONTROL_REGISTER1,buff,1); 		

	return size;

}

int ist8310_write(struct hal_dev_s *dev, const void *buffer, int size,int pos)
{
	return size;
}

int ist8310_ioctl(struct hal_dev_s *dev, uint8_t cmd, void *args)
{
	return 0;
}

int ist8310_close(struct hal_dev_s *dev)
{
	/* we can sleep mpu6050 here for saving power */
	return 0;
}

struct hal_i2c_dev_s ist8310;
	
int ist8310_register(void)
{
	ist8310.port	  = 0;
	ist8310.address   = 0x69;
	ist8310.cfg.flags = HAL_I2C_WR | HAL_I2C_RD;
	ist8310.cfg.speed = 1200000;
	ist8310.cfg.width = 8;

	ist8310.dev.init  = ist8310_init;
	ist8310.dev.open  = ist8310_open;
	ist8310.dev.close = ist8310_close;
	ist8310.dev.read  = ist8310_read;
	ist8310.dev.write = ist8310_write;
	ist8310.dev.ioctl = ist8310_ioctl;

	hal_i2c_device_register(&ist8310,"ist8310-0",HAL_O_RDWR | HAL_DEV_STANDALONE);
	return 0;
};

