#include "hal_spi.h"
#include "hal_sensor.h"
#include "stm32f4xx.h"

#define SENSOR_NAME "dps280"

#define DPS280_CSM_PORT              GPIOA
#define DPS280_CSM_CLK               RCC_AHB1Periph_GPIOA
#define DPS280_CSM_PIN               GPIO_Pin_4
#define Set_DPS280_CSM  DPS280_CSM_PORT->BSRRL = DPS280_CSM_PIN  
#define Clr_DPS280_CSM  DPS280_CSM_PORT->BSRRH = DPS280_CSM_PIN  


void dps280_write_reg(struct hal_dev_s *dev,uint8_t reg,uint8_t dat)
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

void dps280_read_reg(struct hal_dev_s *dev,uint8_t reg,uint8_t *dat)
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



static void dps280_pin_init(void)
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

static void dps280_cs_take(void)
{
	GPIO_ResetBits(MPU6000_CSM_PORT,MPU6000_CSM_PIN);
}

static void dps280_cs_release(void)
{
	GPIO_SetBits(MPU6000_CSM_PORT,MPU6000_CSM_PIN);
}


static void dps280_read_multi(struct hal_dev_s *dev,uint8_t reg, uint8_t *buff, uint8_t num)
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

static void dps280_delay_us(uint32_t ms)
{
    volatile int i,j;
    for(i = ms;i > 0;i--){
        for(j = 20;j > 0;j--);
    }
}

struct dps280_config_s{
	float acc_range;
	float gyro_range;
	float acc_scale;
	float gyro_scale;
};

int dps280_init(struct hal_dev_s *dev)
{
	uint8_t reg_val;
	struct hal_spi_dev_s *spidev = (struct hal_spi_dev_s *)dev;
	
	dps280_pin_init();
	
	spidev->cfg.speed = 1000000;
    hal_spi_configure(dev,&spidev->cfg);
	
	dps280_read_reg(dev,MPU_RA_WHO_AM_I,&reg_val);
	
	if(reg_val != MPU6000_WHO_AM_I_VALUE){
		return -1;
	}
	// Device Reset
	dps280_write_reg(dev,MPU_RA_PWR_MGMT_1,BIT_H_RESET);
	dps280_delay_us(15000);
	dps280_write_reg(dev,MPU_RA_SIGNAL_PATH_RESET,BIT_GYRO | BIT_ACC | BIT_TEMP);
	dps280_delay_us(15000);
	// Clock Source PPL with Z axis gyro reference
	dps280_write_reg(dev,MPU_RA_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
	dps280_delay_us(15);
	// Disable Primary I2C Interface
    dps280_write_reg(dev, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    dps280_delay_us(15);
	dps280_write_reg(dev, MPU_RA_PWR_MGMT_2, 0x00);
    dps280_delay_us(15);
	dps280_write_reg(dev, MPU_RA_SMPLRT_DIV, 0x00);
    dps280_delay_us(15);
	// Gyro +/- 1000 DPS Full Scale
    dps280_write_reg(dev, MPU_RA_GYRO_CONFIG, BITS_FS_2000DPS);
    dps280_delay_us(15);
    // Accel +/- 8 G Full Scale
    dps280_write_reg(dev, MPU_RA_ACCEL_CONFIG, BITS_FS_8G);
    dps280_delay_us(15);
	

	spidev->cfg.speed = 10000000;
    hal_spi_configure(dev,&spidev->cfg);

	return 0;
}
int dps280_open(struct hal_dev_s *dev, uint16_t oflag)
{
	return dps280_init(dev);
}

int dps280_close(struct hal_dev_s *dev)
{
	/* we can sleep dps280 here for saving power */
	printf("close dps280 ok\n");
	return 0;
}

int dps280_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{ 
	int i = 0;
	uint8_t buff[14];
	int16_t acc[3];
	int16_t temp;
	int16_t gyro[3];
	struct imu_report_s *imu = (struct imu_report_s *)buffer;
	struct dps280_config_s *config = dev->priv_data;
	int read_num = size / sizeof(struct imu_report_s);

	for(i = 0; i < read_num;i++){
		dps280_read_multi(dev, MPU_RA_ACCEL_XOUT_H,buff,14);
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
	return size;
}

int dps280_write(struct hal_dev_s *dev, const void *buffer, int size,int pos)
{
	printf("write dps280: %d\n",size);
	return size;
}

int dps280_ioctl(struct hal_dev_s *dev, uint8_t cmd, void *args)
{
	printf("ioctl dps280 OK\n");
	return 0;
}

struct hal_spi_dev_s dps280_spi;
struct dps280_config_s dps280_config;

int dps280_register(void)
{
	dps280_spi.port       = 0;
	dps280_spi.cfg.speed  = 10000000;
	dps280_spi.cfg.mode   = HAL_SPI_MODE_3 | HAL_SPI_MSB;
	dps280_spi.cfg.width  = 8;
	dps280_spi.cs_take    = dps280_cs_take;
	dps280_spi.cs_release = dps280_cs_release;
	

	dps280_spi.dev.init  = dps280_init;
	dps280_spi.dev.open  = dps280_open;
	dps280_spi.dev.close = dps280_close;
	dps280_spi.dev.read  = dps280_read;
	dps280_spi.dev.write = dps280_write;
	dps280_spi.dev.ioctl = dps280_ioctl;

	dps280_config.acc_range  = 8.0f;     /* G */
	dps280_config.gyro_range = 2000.0f;  /* d/s */
	dps280_config.acc_scale  = dps280_config.acc_range *  9.8f / 32767.0f;
	dps280_config.gyro_scale = dps280_config.gyro_range  / 57.3f / 32767.0f;
	dps280_spi.dev.priv_data = &dps280_config;

	hal_spi_device_register(&dps280_spi,"dps280",HAL_O_RDWR | HAL_DEV_STANDALONE);
	return 0;
}



