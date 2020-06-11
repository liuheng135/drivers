#include "hal_spi.h"
#include "hal_sensor.h"
#include "stm32f4xx.h"

// RA = Register Address

#define ICM_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define ICM_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define ICM_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD

#define ICM_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define ICM_XG_OFFS_USRL     0x14
#define ICM_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define ICM_YG_OFFS_USRL     0x16
#define ICM_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define ICM_ZG_OFFS_USRL     0x18
#define ICM_SMPLRT_DIV       0x19
#define ICM_CONFIG           0x1A
#define ICM_GYRO_CONFIG      0x1B
#define ICM_ACCEL_CONFIG     0x1C
#define ICM_ACCEL_CONFIG2    0x1D
#define ICM_LP_MODE_CTL      0x1E
#define ICM_ACCEL_WOM_X_THR  0x20
#define ICM_ACCEL_WOM_Y_THR  0x21
#define ICM_ACCEL_WOM_Z_THR  0x22
#define ICM_FIFO_EN          0x23
#define ICM_INT_PIN_CFG      0x37
#define ICM_INT_ENABLE       0x38
#define ICM_DMP_INT_STATUS   0x39
#define ICM_INT_STATUS       0x3A
#define ICM_ACCEL_XOUT_H     0x3B
#define ICM_ACCEL_XOUT_L     0x3C
#define ICM_ACCEL_YOUT_H     0x3D
#define ICM_ACCEL_YOUT_L     0x3E
#define ICM_ACCEL_ZOUT_H     0x3F
#define ICM_ACCEL_ZOUT_L     0x40
#define ICM_TEMP_OUT_H       0x41
#define ICM_TEMP_OUT_L       0x42
#define ICM_GYRO_XOUT_H      0x43
#define ICM_GYRO_XOUT_L      0x44
#define ICM_GYRO_YOUT_H      0x45
#define ICM_GYRO_YOUT_L      0x46
#define ICM_GYRO_ZOUT_H      0x47
#define ICM_GYRO_ZOUT_L      0x48
#define ICM_SIGNAL_PATH_RESET    0x68
#define ICM_ACCEL_INTEL_CTRL     0x69
#define ICM_USER_CTRL        0x6A
#define ICM_PWR_MGMT_1       0x6B
#define ICM_PWR_MGMT_2       0x6C
#define ICM_FIFO_COUNTH      0x72
#define ICM_FIFO_COUNTL      0x73
#define ICM_FIFO_R_W         0x74
#define ICM_WHO_AM_I         0x75
#define ICM_XA_OFFS_H        0x77
#define ICM_XA_OFFS_L        0x78
#define ICM_YA_OFFS_H        0x7A
#define ICM_YA_OFFS_L        0x7B
#define ICM_ZA_OFFS_H        0x7D
#define ICM_ZA_OFFS_L        0x7E

#define icm20789_WHO_AM_I_VALUE		    0x03

// Bits
#define BITS_SLEEP                  0x40
#define BITS_H_RESET                0x80
#define BITS_CLKSEL                 0x07
#define ICM_CLK_SEL_PLLGYROX        0x01
#define ICM_CLK_SEL_PLLGYROZ        0x03
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
#define BIT_GYRO                    0x04
#define BIT_ACC                     0x02
#define BIT_TEMP                    0x01


void icm20789_write_reg(struct hal_dev_s *dev,uint8_t reg,uint8_t dat)
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

void icm20789_read_reg(struct hal_dev_s *dev,uint8_t reg,uint8_t *dat)
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


static void icm20789_read_multi(struct hal_dev_s *dev,uint8_t reg, uint8_t *buff, uint8_t num)
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

static void icm20789_delay_us(uint32_t ms)
{
    volatile int i,j;
    for(i = ms;i > 0;i--){
        for(j = 20;j > 0;j--);
    }
}

struct icm20789_config_s{
	float acc_range;
	float gyro_range;
	float acc_scale;
	float gyro_scale;
};

int icm20789_init(struct hal_dev_s *dev)
{
	uint8_t reg_val = 0;
	struct hal_spi_dev_s *spidev = (struct hal_spi_dev_s *)dev;
	
	spidev->cfg.speed = 8000000;
    hal_spi_configure(dev,&spidev->cfg);
	
	icm20789_delay_us(15);
	icm20789_read_reg(dev,ICM_WHO_AM_I,&reg_val);
	icm20789_delay_us(15);
	icm20789_read_reg(dev,ICM_WHO_AM_I,&reg_val);
	icm20789_delay_us(15);
	icm20789_read_reg(dev,ICM_WHO_AM_I,&reg_val);
	if(reg_val != icm20789_WHO_AM_I_VALUE){
		return -1;
	}
	
	// Device Reset
	icm20789_write_reg(dev,ICM_PWR_MGMT_1,BITS_H_RESET);
	icm20789_delay_us(15000);
	icm20789_write_reg(dev,ICM_SIGNAL_PATH_RESET,BIT_GYRO | BIT_ACC | BIT_TEMP);
	icm20789_delay_us(15000);
	// Clock Source PPL with Z axis gyro reference
	icm20789_write_reg(dev,ICM_PWR_MGMT_1,ICM_CLK_SEL_PLLGYROZ);
	icm20789_delay_us(15);
	// Disable Primary I2C Interface
    icm20789_write_reg(dev, ICM_USER_CTRL, BIT_I2C_IF_DIS);
    icm20789_delay_us(15);
	icm20789_write_reg(dev, ICM_PWR_MGMT_2, 0x00);
    icm20789_delay_us(15);
	icm20789_write_reg(dev, ICM_SMPLRT_DIV, 0x00);
    icm20789_delay_us(15);
	// Gyro +/- 1000 DPS Full Scale
    icm20789_write_reg(dev, ICM_GYRO_CONFIG, BITS_FS_2000DPS);
    icm20789_delay_us(15);
    // Accel +/- 8 G Full Scale
    icm20789_write_reg(dev, ICM_ACCEL_CONFIG, BITS_FS_8G);
    icm20789_delay_us(15);
	
	spidev->cfg.speed = 8000000;
    hal_spi_configure(dev,&spidev->cfg);

	return 0;
}
int icm20789_open(struct hal_dev_s *dev, uint16_t oflag)
{
	return 0;
}

int icm20789_close(struct hal_dev_s *dev)
{
	/* we can sleep icm20789 here for saving power */
	return 0;
}

int icm20789_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{ 
	int i = 0;
	uint8_t buff[14];
	int16_t acc[3];
	int16_t temp;
	int16_t gyro[3];
	struct imu_report_s *imu = (struct imu_report_s *)buffer;
	struct icm20789_config_s *config = dev->priv_data;
	int read_num = size / sizeof(struct imu_report_s);

	for(i = 0; i < read_num;i++){
		icm20789_read_multi(dev, ICM_ACCEL_XOUT_H,buff,14);
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

int icm20789_write(struct hal_dev_s *dev, const void *buffer, int size,int pos)
{
	return size;
}

int icm20789_ioctl(struct hal_dev_s *dev, uint8_t cmd, void *args)
{
	return 0;
}

struct hal_spi_dev_s icm20789_spi;
struct icm20789_config_s icm20789_config;

int icm20789_register(void)
{
	icm20789_spi.port       = 0;
	icm20789_spi.cfg.speed  = 8000000;
	icm20789_spi.cfg.mode   = HAL_SPI_MODE_3 | HAL_SPI_MSB;
	icm20789_spi.cfg.width  = 8;
	icm20789_spi.cs_id      = 1;

	icm20789_spi.dev.init  = icm20789_init;
	icm20789_spi.dev.open  = icm20789_open;
	icm20789_spi.dev.close = icm20789_close;
	icm20789_spi.dev.read  = icm20789_read;
	icm20789_spi.dev.write = icm20789_write;
	icm20789_spi.dev.ioctl = icm20789_ioctl;

	icm20789_config.acc_range  = 8.0f;     /* G */
	icm20789_config.gyro_range = 2000.0f;  /* d/s */
	icm20789_config.acc_scale  = icm20789_config.acc_range *  9.8f / 32767.0f;
	icm20789_config.gyro_scale = icm20789_config.gyro_range  / 57.3f / 32767.0f;
	icm20789_spi.dev.priv_data = &icm20789_config;

	hal_spi_device_register(&icm20789_spi,"icm20789",HAL_O_RDWR | HAL_DEV_STANDALONE);
	return 0;
}



