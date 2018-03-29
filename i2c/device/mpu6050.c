#include "hal_i2c.h"
#include "hal_sensor.h"

#define SENSOR_NAME "mpu6050"
#define SENSOR_I2C_SLAVE_ADDRESS	0x69

#define	MPU6050REG_SMPLRT_DIV		0x19
#define	MPU6050REG_CONFIG			0x1A
#define	MPU6050REG_GYRO_CONFIG		0x1B
#define	MPU6050REG_ACCEL_CONFIG		0x1C
#define	MPU6050REG_INT_PIN_CFG		0x37
#define	MPU6050REG_INT_ENABLE		0x38
#define MPU6050REG_INT_STATUS		0x3A
#define	MPU6050REG_ACCEL_XOUT_H		0x3B
#define	MPU6050REG_ACCEL_XOUT_L		0x3C
#define	MPU6050REG_ACCEL_YOUT_H		0x3D
#define	MPU6050REG_ACCEL_YOUT_L		0x3E
#define	MPU6050REG_ACCEL_ZOUT_H		0x3F
#define	MPU6050REG_ACCEL_ZOUT_L		0x40
#define	MPU6050REG_TEMP_OUT_H		0x41
#define	MPU6050REG_TEMP_OUT_L		0x42
#define	MPU6050REG_GYRO_XOUT_H		0x43
#define	MPU6050REG_GYRO_XOUT_L		0x44	
#define	MPU6050REG_GYRO_YOUT_H		0x45
#define	MPU6050REG_GYRO_YOUT_L		0x46
#define	MPU6050REG_GYRO_ZOUT_H		0x47
#define	MPU6050REG_GYRO_ZOUT_L		0x48
#define	MPU6050REG_PWR_MGMT_1		0x6B
#define	MPU6050REG_WHO_AM_I			0x75

#define MPU6050_WHO_AM_I_VALUE		0x68
#define MPU6050A_WHO_AM_I_VALUE		0x98

#define BITS_DLPF_CFG_256HZ_NOLPF2	0x00
#define BITS_DLPF_CFG_188HZ			0x01
#define BITS_DLPF_CFG_98HZ			0x02
#define BITS_DLPF_CFG_42HZ			0x03
#define BITS_DLPF_CFG_20HZ			0x04
#define BITS_DLPF_CFG_10HZ			0x05
#define BITS_DLPF_CFG_5HZ			0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF	0x07
#define BITS_DLPF_CFG_MASK			0x07
#define MODE_CHANGE_DELAY_MS          50

static void mpu6050_delay_us(uint32_t ms)
{
    volatile int i,j;
    for(i = ms;i > 0;i--){
        for(j = 20;j > 0;j--);
    }
}

struct mpu6050_config_s{
	float acc_range;
	float gyro_range;
	float acc_scale;
	float gyro_scale;
};

int mpu6050_init(struct hal_dev_s *dev)
{
	uint8_t result;
	hal_i2c_read_from_addr(dev,MPU6050REG_WHO_AM_I,&result,1);
	if(result == MPU6050_WHO_AM_I_VALUE){
		printf("mpu6050 found\n");
		return 0;
	}else if(result == MPU6050A_WHO_AM_I_VALUE){
		printf("mpu6050a found\n");
		return 0;
	}else{
		printf("init mpu6050 failed\n");
	}
	return -1;
}

static int mpu6050_init_device(struct hal_dev_s *dev)
{
	uint8_t reg_val = 0;
	int8_t result;

	reg_val = 0x80;
	result = hal_i2c_write_to_addr(dev,MPU6050REG_PWR_MGMT_1, &reg_val,1); //enter standby
	if(result < 0){
		printf("[%s] set MPU6050REG_PWR_MGMT_1 failed\r\n",__func__);
		return -1;
	}
	mpu6050_delay_us(50000);
	reg_val = 0x03;
	result = hal_i2c_write_to_addr(dev,MPU6050REG_PWR_MGMT_1, &reg_val,1); //enter standby
	if(result < 0){
		printf("[%s] set MPU6050REG_PWR_MGMT_1 failed\r\n",__func__);
		return -1;
	}
	reg_val = 0x01;
	result = hal_i2c_write_to_addr(dev,MPU6050REG_SMPLRT_DIV, &reg_val,1); //enter standby
	if(result < 0){
		printf("[%s] set MPU6050REG_SMPLRT_DIV failed\r\n",__func__);
		return -1;
	}
	reg_val = BITS_DLPF_CFG_42HZ;
	result = hal_i2c_write_to_addr(dev,MPU6050REG_CONFIG , &reg_val,1); //enter standby
	if(result < 0){
		printf("[%s] set MPU6050REG_CONFIG failed\r\n",__func__);
		return -1;
	}
	reg_val = 0x18;
	result = hal_i2c_write_to_addr(dev,MPU6050REG_GYRO_CONFIG, &reg_val,1); //enter standby  2000 deg/s
	if(result < 0){
		printf("[%s] set MPU6050REG_GYRO_CONFIG failed\r\n",__func__);
		return -1;
	}
	reg_val = 0x10;
	result = hal_i2c_write_to_addr(dev,MPU6050REG_ACCEL_CONFIG, &reg_val,1); //enter standby  8g
	if(result < 0){
		printf("[%s] set MPU6050REG_ACCEL_CONFIG failed\r\n",__func__);
		return -1;
	}
	return 0;
}
int mpu6050_open(struct hal_dev_s *dev, uint16_t oflag)
{
	return mpu6050_init_device(dev);
}

int mpu6050_close(struct hal_dev_s *dev)
{
	/* we can sleep mpu6050 here for saving power */
	printf("close mpu6050 ok\n");
	return 0;
}

int mpu6050_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{
	int i = 0;
	int ret = 0;
	uint8_t buff[14];
	int16_t acc[3];
	int16_t temp;
	int16_t gyro[3];
	struct imu_report_s *imu = (struct imu_report_s *)buffer;
	struct mpu6050_config_s *config = dev->priv_data;
	int read_num = size / sizeof(struct imu_report_s);

	for(i = 0; i < read_num;i++){
		ret += hal_i2c_read_from_addr(dev, MPU6050REG_ACCEL_XOUT_H,buff,14);
		acc[0]  = buff[0]  << 8|buff[1];
		acc[1]  = buff[2]  << 8|buff[3];
		acc[2]  = buff[4]  << 8|buff[5];
		temp    = buff[6]  << 8|buff[7];
		gyro[0] = buff[8]  << 8|buff[9];
		gyro[1] = buff[10] << 8|buff[11];
		gyro[2] = buff[12] << 8|buff[13];

		imu->acc.data[0] = acc[0] * config->acc_scale;
		imu->acc.data[1] = acc[1] * config->acc_scale;
		imu->acc.data[2] = acc[2] * config->acc_scale;
		
		imu->gyro.data[0] = gyro[0] * config->gyro_scale;
		imu->gyro.data[1] = gyro[1] * config->gyro_scale;
		imu->gyro.data[2] = gyro[2] * config->gyro_scale;

		imu->acc.temperature  = 36.53f + (float)temp / 340.0f;
		imu->gyro.temperature = imu->acc.temperature;
		
		imu++;
	}
	return ret;
}

int mpu6050_write(struct hal_dev_s *dev, const void *buffer, int size,int pos)
{
	printf("write mpu6050: %d\n",size);
	return size;
}

int mpu6050_ioctl(struct hal_dev_s *dev, uint8_t cmd, void *args)
{
	printf("ioctl mpu6050 OK\n");
	return 0;
}

struct hal_i2c_dev_s mpu6050;
struct mpu6050_config_s mpu6050_config;

int mpu6050_register(void)
{
	mpu6050.port	  = 0;
	mpu6050.address   = 0x69;
	mpu6050.cfg.flags = HAL_I2C_WR | HAL_I2C_RD;
	mpu6050.cfg.speed = 1200000;
	mpu6050.cfg.width = 8;

	mpu6050.dev.init  = mpu6050_init;
	mpu6050.dev.open  = mpu6050_open;
	mpu6050.dev.close = mpu6050_close;
	mpu6050.dev.read  = mpu6050_read;
	mpu6050.dev.write = mpu6050_write;
	mpu6050.dev.ioctl = mpu6050_ioctl;

	mpu6050_config.acc_range  = 8.0f;
	mpu6050_config.gyro_range = 2000.0f;
	mpu6050_config.acc_scale  = 0.002387768f;
	mpu6050_config.gyro_scale = 0.001065185f;
	mpu6050.dev.priv_data = &mpu6050_config;

	hal_i2c_device_register(&mpu6050,"mpu6050-0",HAL_O_RDWR | HAL_DEV_STANDALONE);
	return 0;
}



