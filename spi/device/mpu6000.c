#include "hal_spi.h"
#include "hal_sensor.h"
#include "stm32f4xx.h"

#define SENSOR_NAME "mpu6000"

#define MPU6000_CSM_PORT              GPIOA
#define MPU6000_CSM_CLK               RCC_AHB1Periph_GPIOA
#define MPU6000_CSM_PIN               GPIO_Pin_4
#define Set_MPU6000_CSM  MPU6000_CSM_PORT->BSRRL = MPU6000_CSM_PIN //{GPIO_SetBits(MPU6000_CSM_PORT,MPU6000_CSM_PIN);}
#define Clr_MPU6000_CSM  MPU6000_CSM_PORT->BSRRH = MPU6000_CSM_PIN //{GPIO_ResetBits(MPU6000_CSM_PORT,MPU6000_CSM_PIN);}  
//INT主机数据输入
#define MPU6000_INT_PORT              GPIOA
#define MPU6000_INT_CLK               RCC_AHB1Periph_GPIOA 
#define MPU6000_INT_PIN               GPIO_Pin_0
#define Set_MPU6000_INT  MPU6000_INT_PORT->BSRRL = MPU6000_INT_PIN//{GPIO_SetBits(MPU6000_INT_PORT,MPU6000_INT_PIN);}
#define Clr_MPU6000_INT  MPU6000_INT_PORT->BSRRH = MPU6000_INT_PIN{GPIO_ResetBits(MPU6000_INT_PORT,MPU6000_INT_PIN);} 
#define MPU6000_INT  (GPIO_ReadInputDataBit(MPU6000_INT_PORT, MPU6000_INT_PIN))



#define	MPU6000REG_SMPLRT_DIV		0x19
#define	MPU6000REG_CONFIG			0x1A
#define	MPU6000REG_GYRO_CONFIG		0x1B
#define	MPU6000REG_ACCEL_CONFIG		0x1C
#define	MPU6000REG_INT_PIN_CFG		0x37
#define	MPU6000REG_INT_ENABLE		0x38
#define MPU6000REG_INT_STATUS		0x3A
#define	MPU6000REG_ACCEL_XOUT_H		0x3B
#define	MPU6000REG_ACCEL_XOUT_L		0x3C
#define	MPU6000REG_ACCEL_YOUT_H		0x3D
#define	MPU6000REG_ACCEL_YOUT_L		0x3E
#define	MPU6000REG_ACCEL_ZOUT_H		0x3F
#define	MPU6000REG_ACCEL_ZOUT_L		0x40
#define	MPU6000REG_TEMP_OUT_H		0x41
#define	MPU6000REG_TEMP_OUT_L		0x42
#define	MPU6000REG_GYRO_XOUT_H		0x43
#define	MPU6000REG_GYRO_XOUT_L		0x44	
#define	MPU6000REG_GYRO_YOUT_H		0x45
#define	MPU6000REG_GYRO_YOUT_L		0x46
#define	MPU6000REG_GYRO_ZOUT_H		0x47
#define	MPU6000REG_GYRO_ZOUT_L		0x48
#define	MPU6000REG_PWR_MGMT_1		0x6B
#define	MPU6000REG_WHO_AM_I			0x75

#define MPU6000_WHO_AM_I_VALUE		0x68
#define MPU6000A_WHO_AM_I_VALUE		0x98

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


void mpu6000_write_reg(struct hal_dev_s *dev,uint8_t reg,uint8_t dat)
{
	struct hal_spi_msg_s msg;
    uint8_t send_buffer[2];
    uint8_t recv_buffer[2];
	
	msg.recv_buffer = recv_buffer;
	msg.send_buffer = send_buffer;
	msg.length      = 2;
	msg.cs_release  = 1;
	msg.cs_take     = 1;
    
    send_buffer[0] =  reg;
    send_buffer[1] = dat;
	
	hal_spi_transfer(dev,&msg);
}

void mpu6000_read_reg(struct hal_dev_s *dev,uint8_t reg,uint8_t *dat)
{
	struct hal_spi_msg_s msg;
    uint8_t send_buffer[2];
    uint8_t recv_buffer[2];
	
	msg.recv_buffer = recv_buffer;
	msg.send_buffer = send_buffer;
	msg.length      = 2;
	msg.cs_release  = 1;
	msg.cs_take     = 1;
    
    send_buffer[0] = (0x80 | reg);
    send_buffer[1] = 0x00;//(0x80 | reg);
	hal_spi_transfer(dev,&msg);
    
    *dat = recv_buffer[1];
}



static void mpu6000_pin_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(MPU6000_INT_CLK | MPU6000_CSM_CLK, ENABLE);
	 
	GPIO_InitStructure.GPIO_Pin = MPU6000_INT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN  ;   
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(MPU6000_INT_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = MPU6000_CSM_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT  ;   
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(MPU6000_CSM_PORT, &GPIO_InitStructure);
}	

static void mpu6000_cs_take(void)
{
	GPIO_ResetBits(MPU6000_CSM_PORT,MPU6000_CSM_PIN);
}

static void mpu6000_cs_release(void)
{
	GPIO_SetBits(MPU6000_CSM_PORT,MPU6000_CSM_PIN);
}


static void mpu6000_read_multi(struct hal_dev_s *dev,uint8_t reg, uint8_t *buff, uint8_t num)
{
	uint8_t cmd = (0x80 | reg);
	struct hal_spi_msg_s msg;
	
	msg.recv_buffer = NULL;
	msg.send_buffer = &cmd;
	msg.length      = 1;
	msg.cs_take     = 1;
	msg.cs_release  = 0;
    
	hal_spi_transfer(dev,&msg);
    
	msg.recv_buffer = buff;
	msg.send_buffer = NULL;
	msg.length      = num;
	msg.cs_take     = 0;
	msg.cs_release  = 1;
	
	hal_spi_transfer(dev,&msg);
}

static void mpu6000_delay_us(uint32_t ms)
{
    volatile int i,j;
    for(i = ms;i > 0;i--){
        for(j = 20;j > 0;j--);
    }
}

struct mpu6000_config_s{
	float acc_range;
	float gyro_range;
	float acc_scale;
	float gyro_scale;
};

int mpu6000_init(struct hal_dev_s *dev)
{
	uint8_t reg_val;
	
	mpu6000_pin_init();
	mpu6000_read_reg(dev,MPU6000REG_WHO_AM_I,&reg_val);
	
	if(reg_val != MPU6000_WHO_AM_I_VALUE){
		return -1;
	}

	mpu6000_write_reg(dev,MPU6000REG_PWR_MGMT_1,0x80);
	mpu6000_delay_us(50000);
	mpu6000_write_reg(dev,MPU6000REG_PWR_MGMT_1,0x03);
	mpu6000_write_reg(dev,MPU6000REG_SMPLRT_DIV,0x01);
	mpu6000_write_reg(dev,MPU6000REG_CONFIG,BITS_DLPF_CFG_42HZ);
	mpu6000_write_reg(dev,MPU6000REG_GYRO_CONFIG,0x18);           //gyro range 2000 deg/s
	mpu6000_write_reg(dev,MPU6000REG_ACCEL_CONFIG,0x10);          //acc range  8g


	return 0;
}
int mpu6000_open(struct hal_dev_s *dev, uint16_t oflag)
{
	return mpu6000_init(dev);
}

int mpu6000_close(struct hal_dev_s *dev)
{
	/* we can sleep mpu6000 here for saving power */
	printf("close mpu6000 ok\n");
	return 0;
}

int mpu6000_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{
	int i = 0;
	uint8_t buff[14];
	int16_t acc[3];
	int16_t temp;
	int16_t gyro[3];
	struct imu_report_s *imu = (struct imu_report_s *)buffer;
	struct mpu6000_config_s *config = dev->priv_data;
	int read_num = size / sizeof(struct imu_report_s);

	for(i = 0; i < read_num;i++){
		mpu6000_read_multi(dev, MPU6000REG_ACCEL_XOUT_H,buff,14);
		acc[0]  = buff[0]  << 8|buff[1];
		acc[1]  = buff[2]  << 8|buff[3];
		acc[2]  = buff[4]  << 8|buff[5];
		temp    = buff[6]  << 8|buff[7];
		gyro[0] = buff[8]  << 8|buff[9];
		gyro[1] = buff[10] << 8|buff[11];
		gyro[2] = buff[12] << 8|buff[13];

		imu->acc.accel[0] = acc[0] * config->acc_scale;
		imu->acc.accel[1] = acc[1] * config->acc_scale;
		imu->acc.accel[2] = acc[2] * config->acc_scale;
		
		imu->gyro.gyro[0] = gyro[0] * config->gyro_scale;
		imu->gyro.gyro[1] = gyro[1] * config->gyro_scale;
		imu->gyro.gyro[2] = gyro[2] * config->gyro_scale;

		imu->acc.temperature  = 36.53f + (float)temp / 340.0f;
		imu->gyro.temperature = imu->acc.temperature;
		
		imu++;
	}
	return size;
}

int mpu6000_write(struct hal_dev_s *dev, const void *buffer, int size,int pos)
{
	printf("write mpu6000: %d\n",size);
	return size;
}

int mpu6000_ioctl(struct hal_dev_s *dev, uint8_t cmd, void *args)
{
	printf("ioctl mpu6000 OK\n");
	return 0;
}

struct hal_spi_dev_s mpu6000_spi;
struct mpu6000_config_s mpu6000_config;

int mpu6000_register(void)
{
	mpu6000_spi.port       = 0;
	mpu6000_spi.cfg.speed  = 10000000;
	mpu6000_spi.cfg.mode   = HAL_SPI_MODE_3 | HAL_SPI_MSB;
	mpu6000_spi.cfg.width  = 8;
	mpu6000_spi.cs_take    = mpu6000_cs_take;
	mpu6000_spi.cs_release = mpu6000_cs_release;
	

	mpu6000_spi.dev.init  = mpu6000_init;
	mpu6000_spi.dev.open  = mpu6000_open;
	mpu6000_spi.dev.close = mpu6000_close;
	mpu6000_spi.dev.read  = mpu6000_read;
	mpu6000_spi.dev.write = mpu6000_write;
	mpu6000_spi.dev.ioctl = mpu6000_ioctl;

	mpu6000_config.acc_range  = 8.0f;
	mpu6000_config.gyro_range = 2000.0f;
	mpu6000_config.acc_scale  = 0.002387768f;
	mpu6000_config.gyro_scale = 0.001065185f;
	mpu6000_spi.dev.priv_data = &mpu6000_config;

	hal_spi_device_register(&mpu6000_spi,"mpu6000",HAL_O_RDWR | HAL_DEV_STANDALONE);
	return 0;
}



