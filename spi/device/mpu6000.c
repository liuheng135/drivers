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



// RA = Register Address

#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define MPU6000_WHO_AM_I_VALUE		0x68
#define MPU6000A_WHO_AM_I_VALUE		0x98



// Bits
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1


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
	struct hal_spi_dev_s *spidev = (struct hal_spi_dev_s *)dev;
	
	mpu6000_pin_init();
	
	spidev->cfg.speed = 1000000;
    hal_spi_configure(dev,&spidev->cfg);
	
	mpu6000_read_reg(dev,MPU_RA_WHO_AM_I,&reg_val);
	
	if(reg_val != MPU6000_WHO_AM_I_VALUE){
		return -1;
	}
	// Device Reset
	mpu6000_write_reg(dev,MPU_RA_PWR_MGMT_1,BIT_H_RESET);
	mpu6000_delay_us(15000);
	mpu6000_write_reg(dev,MPU_RA_SIGNAL_PATH_RESET,BIT_GYRO | BIT_ACC | BIT_TEMP);
	mpu6000_delay_us(15000);
	// Clock Source PPL with Z axis gyro reference
	mpu6000_write_reg(dev,MPU_RA_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
	mpu6000_delay_us(15);
	// Disable Primary I2C Interface
    mpu6000_write_reg(dev, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    mpu6000_delay_us(15);
	mpu6000_write_reg(dev, MPU_RA_PWR_MGMT_2, 0x00);
    mpu6000_delay_us(15);
	mpu6000_write_reg(dev, MPU_RA_SMPLRT_DIV, 0x00);
    mpu6000_delay_us(15);
	// Gyro +/- 1000 DPS Full Scale
    mpu6000_write_reg(dev, MPU_RA_GYRO_CONFIG, BITS_FS_2000DPS);
    mpu6000_delay_us(15);
    // Accel +/- 8 G Full Scale
    mpu6000_write_reg(dev, MPU_RA_ACCEL_CONFIG, BITS_FS_8G);
    mpu6000_delay_us(15);
	

	spidev->cfg.speed = 10000000;
    hal_spi_configure(dev,&spidev->cfg);

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
		mpu6000_read_multi(dev, MPU_RA_ACCEL_XOUT_H,buff,14);
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

	mpu6000_config.acc_range  = 8.0f;     /* G */
	mpu6000_config.gyro_range = 2000.0f;  /* d/s */
	mpu6000_config.acc_scale  = mpu6000_config.acc_range *  9.8f / 32767.0f;
	mpu6000_config.gyro_scale = mpu6000_config.gyro_range  / 57.3f / 32767.0f;
	mpu6000_spi.dev.priv_data = &mpu6000_config;

	hal_spi_device_register(&mpu6000_spi,"mpu6000",HAL_O_RDWR | HAL_DEV_STANDALONE);
	return 0;
}



