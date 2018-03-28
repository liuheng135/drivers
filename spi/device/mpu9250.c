#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "hal_spi.h"
#include "hal_sensor.h"

#define    MPU9250_GET_TIME_MS()           hrt_absolute_ms()


#define MPU9250_CSM_PORT              GPIOA
#define MPU9250_CSM_CLK               RCC_AHB1Periph_GPIOA
#define MPU9250_CSM_PIN               GPIO_Pin_4
#define Set_MPU9250_CSM  MPU9250_CSM_PORT->BSRRL = MPU9250_CSM_PIN //{GPIO_SetBits(MPU9250_CSM_PORT,MPU9250_CSM_PIN);}
#define Clr_MPU9250_CSM  MPU9250_CSM_PORT->BSRRH = MPU9250_CSM_PIN //{GPIO_ResetBits(MPU9250_CSM_PORT,MPU9250_CSM_PIN);}  
//INT主机数据输入
#define MPU9250_INT_PORT              GPIOA
#define MPU9250_INT_CLK               RCC_AHB1Periph_GPIOA 
#define MPU9250_INT_PIN               GPIO_Pin_0
#define Set_MPU9250_INT  MPU9250_INT_PORT->BSRRL = MPU9250_INT_PIN//{GPIO_SetBits(MPU9250_INT_PORT,MPU9250_INT_PIN);}
#define Clr_MPU9250_INT  MPU9250_INT_PORT->BSRRH = MPU9250_INT_PIN{GPIO_ResetBits(MPU9250_INT_PORT,MPU9250_INT_PIN);} 
#define MPU9250_INT  (GPIO_ReadInputDataBit(MPU9250_INT_PORT, MPU9250_INT_PIN))



#define MPU6500_DEG_PER_LSB_250  0.0076295109483482f  //((2 * 250.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_500  0.0152587890625f     //((2 * 500.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_1000 0.030517578125f      //((2 * 1000.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_2000 0.06103515625f       //((2 * 2000.0) / 65536.0)

#define MPU6500_G_PER_LSB_2      0.00006103515625f    //((2 * 2) / 65536.0)
#define MPU6500_G_PER_LSB_4      0.0001220703125f     //((2 * 4) / 65536.0)
#define MPU6500_G_PER_LSB_8      0.000244140625f     //((2 * 8) / 65536.0)
#define MPU6500_G_PER_LSB_16     0.00048828125f       //((2 * 16) / 65536.0)

#define AK8963_uT_PER_LSB_4900   0.15f     //16bit

#define MPU6500_ST_GYRO_LOW      10.0   // deg/s
#define MPU6500_ST_GYRO_HIGH     105.0  // deg/s
#define MPU6500_ST_ACCEL_LOW     0.300  // G
#define MPU6500_ST_ACCEL_HIGH    0.950  // G


#define MPU6500_I2C_ADDR            ((uint8_t)0xD0)
#define MPU6500_Device_ID           ((uint8_t)0x71) // In MPU9250

#define MPU6500_SELF_TEST_XG        ((uint8_t)0x00)
#define MPU6500_SELF_TEST_YG        ((uint8_t)0x01)
#define MPU6500_SELF_TEST_ZG        ((uint8_t)0x02)
#define MPU6500_SELF_TEST_XA        ((uint8_t)0x0D)
#define MPU6500_SELF_TEST_YA        ((uint8_t)0x0E)
#define MPU6500_SELF_TEST_ZA        ((uint8_t)0x0F)
#define MPU6500_XG_OFFSET_H         ((uint8_t)0x13)
#define MPU6500_XG_OFFSET_L         ((uint8_t)0x14)
#define MPU6500_YG_OFFSET_H         ((uint8_t)0x15)
#define MPU6500_YG_OFFSET_L         ((uint8_t)0x16)
#define MPU6500_ZG_OFFSET_H         ((uint8_t)0x17)
#define MPU6500_ZG_OFFSET_L         ((uint8_t)0x18)
#define MPU6500_SMPLRT_DIV          ((uint8_t)0x19)
#define MPU6500_CONFIG              ((uint8_t)0x1A)
#define MPU6500_GYRO_CONFIG         ((uint8_t)0x1B)
#define MPU6500_ACCEL_CONFIG        ((uint8_t)0x1C)
#define MPU6500_ACCEL_CONFIG_2      ((uint8_t)0x1D)
#define MPU6500_LP_ACCEL_ODR        ((uint8_t)0x1E)
#define MPU6500_MOT_THR             ((uint8_t)0x1F)
#define MPU6500_FIFO_EN             ((uint8_t)0x23)
#define MPU6500_I2C_MST_CTRL        ((uint8_t)0x24)
#define MPU6500_I2C_SLV0_ADDR       ((uint8_t)0x25)
#define MPU6500_I2C_SLV0_REG        ((uint8_t)0x26)
#define MPU6500_I2C_SLV0_CTRL       ((uint8_t)0x27)
#define MPU6500_I2C_SLV1_ADDR       ((uint8_t)0x28)
#define MPU6500_I2C_SLV1_REG        ((uint8_t)0x29)
#define MPU6500_I2C_SLV1_CTRL       ((uint8_t)0x2A)
#define MPU6500_I2C_SLV2_ADDR       ((uint8_t)0x2B)
#define MPU6500_I2C_SLV2_REG        ((uint8_t)0x2C)
#define MPU6500_I2C_SLV2_CTRL       ((uint8_t)0x2D)
#define MPU6500_I2C_SLV3_ADDR       ((uint8_t)0x2E)
#define MPU6500_I2C_SLV3_REG        ((uint8_t)0x2F)
#define MPU6500_I2C_SLV3_CTRL       ((uint8_t)0x30)
#define MPU6500_I2C_SLV4_ADDR       ((uint8_t)0x31)
#define MPU6500_I2C_SLV4_REG        ((uint8_t)0x32)
#define MPU6500_I2C_SLV4_DO         ((uint8_t)0x33)
#define MPU6500_I2C_SLV4_CTRL       ((uint8_t)0x34)
#define MPU6500_I2C_SLV4_DI         ((uint8_t)0x35)
#define MPU6500_I2C_MST_STATUS      ((uint8_t)0x36)
#define MPU6500_INT_PIN_CFG         ((uint8_t)0x37)
#define MPU6500_INT_ENABLE          ((uint8_t)0x38)
#define MPU6500_INT_STATUS          ((uint8_t)0x3A)
#define MPU6500_ACCEL_XOUT_H        ((uint8_t)0x3B)
#define MPU6500_ACCEL_XOUT_L        ((uint8_t)0x3C)
#define MPU6500_ACCEL_YOUT_H        ((uint8_t)0x3D)
#define MPU6500_ACCEL_YOUT_L        ((uint8_t)0x3E)
#define MPU6500_ACCEL_ZOUT_H        ((uint8_t)0x3F)
#define MPU6500_ACCEL_ZOUT_L        ((uint8_t)0x40)
#define MPU6500_TEMP_OUT_H          ((uint8_t)0x41)
#define MPU6500_TEMP_OUT_L          ((uint8_t)0x42)
#define MPU6500_GYRO_XOUT_H         ((uint8_t)0x43)
#define MPU6500_GYRO_XOUT_L         ((uint8_t)0x44)
#define MPU6500_GYRO_YOUT_H         ((uint8_t)0x45)
#define MPU6500_GYRO_YOUT_L         ((uint8_t)0x46)
#define MPU6500_GYRO_ZOUT_H         ((uint8_t)0x47)
#define MPU6500_GYRO_ZOUT_L         ((uint8_t)0x48)
#define MPU6500_EXT_SENS_DATA_00    ((uint8_t)0x49)
#define MPU6500_EXT_SENS_DATA_01    ((uint8_t)0x4A)
#define MPU6500_EXT_SENS_DATA_02    ((uint8_t)0x4B)
#define MPU6500_EXT_SENS_DATA_03    ((uint8_t)0x4C)
#define MPU6500_EXT_SENS_DATA_04    ((uint8_t)0x4D)
#define MPU6500_EXT_SENS_DATA_05    ((uint8_t)0x4E)
#define MPU6500_EXT_SENS_DATA_06    ((uint8_t)0x4F)
#define MPU6500_EXT_SENS_DATA_07    ((uint8_t)0x50)
#define MPU6500_EXT_SENS_DATA_08    ((uint8_t)0x51)
#define MPU6500_EXT_SENS_DATA_09    ((uint8_t)0x52)
#define MPU6500_EXT_SENS_DATA_10    ((uint8_t)0x53)
#define MPU6500_EXT_SENS_DATA_11    ((uint8_t)0x54)
#define MPU6500_EXT_SENS_DATA_12    ((uint8_t)0x55)
#define MPU6500_EXT_SENS_DATA_13    ((uint8_t)0x56)
#define MPU6500_EXT_SENS_DATA_14    ((uint8_t)0x57)
#define MPU6500_EXT_SENS_DATA_15    ((uint8_t)0x58)
#define MPU6500_EXT_SENS_DATA_16    ((uint8_t)0x59)
#define MPU6500_EXT_SENS_DATA_17    ((uint8_t)0x5A)
#define MPU6500_EXT_SENS_DATA_18    ((uint8_t)0x5B)
#define MPU6500_EXT_SENS_DATA_19    ((uint8_t)0x5C)
#define MPU6500_EXT_SENS_DATA_20    ((uint8_t)0x5D)
#define MPU6500_EXT_SENS_DATA_21    ((uint8_t)0x5E)
#define MPU6500_EXT_SENS_DATA_22    ((uint8_t)0x5F)
#define MPU6500_EXT_SENS_DATA_23    ((uint8_t)0x60)
#define MPU6500_I2C_SLV0_DO         ((uint8_t)0x63)
#define MPU6500_I2C_SLV1_DO         ((uint8_t)0x64)
#define MPU6500_I2C_SLV2_DO         ((uint8_t)0x65)
#define MPU6500_I2C_SLV3_DO         ((uint8_t)0x66)
#define MPU6500_I2C_MST_DELAY_CTRL  ((uint8_t)0x67)
#define MPU6500_SIGNAL_PATH_RESET   ((uint8_t)0x68)
#define MPU6500_MOT_DETECT_CTRL     ((uint8_t)0x69)
#define MPU6500_USER_CTRL           ((uint8_t)0x6A)
#define MPU6500_PWR_MGMT_1          ((uint8_t)0x6B)
#define MPU6500_PWR_MGMT_2          ((uint8_t)0x6C)
#define MPU6500_FIFO_COUNTH         ((uint8_t)0x72)
#define MPU6500_FIFO_COUNTL         ((uint8_t)0x73)
#define MPU6500_FIFO_R_W            ((uint8_t)0x74)
#define MPU6500_WHO_AM_I            ((uint8_t)0x75) // ID = 0x71 In MPU9250
#define MPU6500_XA_OFFSET_H         ((uint8_t)0x77)
#define MPU6500_XA_OFFSET_L         ((uint8_t)0x78)
#define MPU6500_YA_OFFSET_H         ((uint8_t)0x7A)
#define MPU6500_YA_OFFSET_L         ((uint8_t)0x7B)
#define MPU6500_ZA_OFFSET_H         ((uint8_t)0x7D)
#define MPU6500_ZA_OFFSET_L         ((uint8_t)0x7E)

#define MPU6500_I2C_SLVx_EN         ((uint8_t)0x80)
#define MPU6500_I2C_SLV4_DONE       ((uint8_t)0x40)
#define MPU6500_I2C_SLV4_NACK       ((uint8_t)0x10)

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

#define AK8963_I2C_ADDR             ((uint8_t)0x0C)
#define AK8963_Device_ID            ((uint8_t)0x48)

// Read-only Reg
#define AK8963_WIA                  ((uint8_t)0x00)
#define AK8963_INFO                 ((uint8_t)0x01)
#define AK8963_ST1                  ((uint8_t)0x02)
#define AK8963_HXL                  ((uint8_t)0x03)
#define AK8963_HXH                  ((uint8_t)0x04)
#define AK8963_HYL                  ((uint8_t)0x05)
#define AK8963_HYH                  ((uint8_t)0x06)
#define AK8963_HZL                  ((uint8_t)0x07)
#define AK8963_HZH                  ((uint8_t)0x08)
#define AK8963_ST2                  ((uint8_t)0x09)
// Write/Read Reg
#define AK8963_CNTL1                ((uint8_t)0x0A)
#define AK8963_CNTL2                ((uint8_t)0x0B)
#define AK8963_ASTC                 ((uint8_t)0x0C)
#define AK8963_TS1                  ((uint8_t)0x0D)
#define AK8963_TS2                  ((uint8_t)0x0E)
#define AK8963_I2CDIS               ((uint8_t)0x0F)
// Read-only Reg ( ROM )
#define AK8963_ASAX                 ((uint8_t)0x10)
#define AK8963_ASAY                 ((uint8_t)0x11)
#define AK8963_ASAZ                 ((uint8_t)0x12)
// Status
#define AK8963_STATUS_DRDY          ((uint8_t)0x01)
#define AK8963_STATUS_DOR           ((uint8_t)0x02)
#define AK8963_STATUS_HOFL          ((uint8_t)0x08)
/*=====================================================================================================*/

#define MPU9250_CMD_SET_ACCEL_RANGE 0
#define MPU9250_CMD_SET_GYRO_RANGE  1
#define MPU9250_CMD_SET_ACCEL_DLPF  2
#define MPU9250_CMD_SET_GYRO_DLPF   3
#define MPU9250_CMD_GET_ACCEL_SCALE 4
#define MPU9250_CMD_GET_GYRO_SCALE  5
#define MPU9250_CMD_GET_MAG_SCALE   6


struct mpu9250_report{
    int16_t acc[3];
    int16_t gyro[3];
    int16_t mag[3];
	int16_t temp;
};

struct mpu6050_config_s{
	float acc_range;
	float gyro_range;
	float acc_scale;
	float gyro_scale;
	float mag_range;
	float mag_scale;
};

void mpu9250_write_reg(struct hal_dev_s *dev,uint8_t reg,uint8_t dat)
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

void mpu9250_read_reg(struct hal_dev_s *dev,uint8_t reg,uint8_t *dat)
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

void mpu9250_measure(struct hal_dev_s *dev,struct mpu9250_report *dat);

#define DEBUG_PRINT(...)

static void mpu9250_delay_ms(uint32_t ms)
{
    volatile int i,j;
    for(i = ms;i > 0;i--){
        for(j = 10000;j > 0;j--);
    }
}

static void mpu9250_int_pin_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(MPU9250_INT_CLK | MPU9250_CSM_CLK, ENABLE);
	 
	GPIO_InitStructure.GPIO_Pin = MPU9250_INT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN  ;   
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(MPU9250_INT_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = MPU9250_CSM_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT  ;   
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(MPU9250_CSM_PORT, &GPIO_InitStructure);
}	

static void mpu9250_cs_take(void)
{
	GPIO_ResetBits(MPU9250_CSM_PORT,MPU9250_CSM_PIN);
}

static void mpu9250_cs_release(void)
{
	GPIO_SetBits(MPU9250_CSM_PORT,MPU9250_CSM_PIN);
}


static void mpu9250_read_muhali(struct hal_dev_s *dev,uint8_t reg, uint8_t *buff, uint8_t num)
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

static float mpu9250_get_full_scale_gyro_DPL(struct hal_dev_s *dev)
{
	uint8_t data;
	float range;
	
	mpu9250_read_reg(dev,27, &data);
	data >>= 3;
	switch(data)
	{
		case 0:
			range = MPU6500_DEG_PER_LSB_250;
			break;
		case 1:
			range = MPU6500_DEG_PER_LSB_500;
			break;
		case 2:
			range = MPU6500_DEG_PER_LSB_1000;
			break;
		case 3:
			range = MPU6500_DEG_PER_LSB_2000;
			break;
		default:
			range = MPU6500_DEG_PER_LSB_1000;
			break;
	}
	
	return range;
}

static float mpu9250_get_full_scale_mag(void)
{
	return AK8963_uT_PER_LSB_4900;
}

static float mpu9250_get_full_scale_accel_GPL(struct hal_dev_s *dev)
{
	uint8_t data;
	float range;
	
	mpu9250_read_reg(dev,28, &data);
	data >>= 3;
	switch(data)
	{
		case 0:
			range = MPU6500_G_PER_LSB_2;
			break;
		case 1:
			range = MPU6500_G_PER_LSB_4;
			break;
		case 2:
			range = MPU6500_G_PER_LSB_8;
			break;
		case 3:
			range = MPU6500_G_PER_LSB_16;
			break;
		default:
			range = MPU6500_G_PER_LSB_8;
			break;
	}
	
	return range;
}

//static void mpu9250_ak8963_write_reg(dev,uint8_t reg, uint8_t wrdata)
//{
//	mpu9250_write_reg(dev,37, 0X0C);//AK8975 Address WRITE
//	mpu9250_write_reg(dev,38, reg);//AK8975 Reg Address
//	mpu9250_write_reg(dev,39, 0x81);//enable
//	mpu9250_write_reg(dev,99, wrdata);//data
//	mpu9250_delay_ms(1);
//	mpu9250_write_reg(dev,39, 0x00);//disable
//	
//}
static void mpu9250_ak8963_write_reg(struct hal_dev_s *dev,uint8_t reg, uint8_t wrdata)
{
    uint8_t  status = 0;
    uint32_t timeout = 10;
    
	mpu9250_write_reg(dev,MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR);//AK8975 Address WRITE
	mpu9250_write_reg(dev,MPU6500_I2C_SLV4_REG, reg);//AK8975 Reg Address
	mpu9250_write_reg(dev,MPU6500_I2C_SLV4_DO, wrdata);//data
	mpu9250_write_reg(dev,MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);//enable
    mpu9250_delay_ms(1);
	
    do {
        mpu9250_read_reg(dev,MPU6500_I2C_MST_STATUS,&status);
        mpu9250_delay_ms(1);
    } while(((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));
    mpu9250_write_reg(dev,39, 0x00);//disable
}

void mpu9250_ak8963_write_regs(struct hal_dev_s *dev,uint8_t writeAddr, uint8_t *writeData, uint8_t lens )
{
  uint8_t  status = 0;
  uint32_t timeout = 10;

  mpu9250_write_reg(dev,MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
  mpu9250_delay_ms(1);
  for(uint8_t i = 0; i < lens; i++) {
    mpu9250_write_reg(dev,MPU6500_I2C_SLV4_REG, writeAddr + i);
    mpu9250_delay_ms(1);
    mpu9250_write_reg(dev,MPU6500_I2C_SLV4_DO, writeData[i]);
    mpu9250_delay_ms(1);
    mpu9250_write_reg(dev,MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
    mpu9250_delay_ms(1);
      
    timeout = 10;

    do {
      mpu9250_read_reg(dev,MPU6500_I2C_MST_STATUS,&status);
    } while(((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));
  }
}

static void mpu9250_ak8963_read_reg(struct hal_dev_s *dev,uint8_t reg, uint8_t *rddata)
{
    uint8_t status = 0;
    //uint8_t readData = 0;
    uint32_t timeout = 10;
    
	mpu9250_write_reg(dev,MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80);//AK8975 Address WRITE
	mpu9250_write_reg(dev,MPU6500_I2C_SLV4_REG, reg);//AK8975 Reg Address
	mpu9250_write_reg(dev,MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);//enable
	mpu9250_delay_ms(1);
    
    do {
        mpu9250_read_reg(dev,MPU6500_I2C_MST_STATUS,&status);
        mpu9250_delay_ms(1);
    } while(((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));

    mpu9250_read_reg(dev,MPU6500_I2C_SLV4_DI,rddata);
     
}

void mpu9250_ak8963_read_regs( struct hal_dev_s *dev,uint8_t readAddr, uint8_t *readData, uint8_t lens )
{
  uint8_t status = 0;
  uint32_t timeout = 10;

  mpu9250_write_reg(dev,MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80);
  mpu9250_delay_ms(1);
  for(uint8_t i = 0; i< lens; i++) {
    mpu9250_write_reg(dev,MPU6500_I2C_SLV4_REG, readAddr + i);
    mpu9250_delay_ms(1);
    mpu9250_write_reg(dev,MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
    mpu9250_delay_ms(1);
    timeout = 10;
    do {
      mpu9250_read_reg(dev,MPU6500_I2C_MST_STATUS,&status);
    } while(((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));

    mpu9250_read_reg(dev,MPU6500_I2C_SLV4_DI,&readData[i]);
    mpu9250_delay_ms(1);
  }
}



void mpu9250_ak8963_get_adjustment(struct hal_dev_s *dev,int8_t *x, int8_t *y, int8_t *z)
{
	int8_t buff[3];
	
	mpu9250_ak8963_write_reg(dev,0X0A, 0X0F);//Fuse ROM access mode 
	
	mpu9250_write_reg(dev,37, 0X8C);//AK8975 Address READ
	mpu9250_write_reg(dev,38, 0X10);//AK8975 ASAX
	mpu9250_write_reg(dev,39, 0x83);//enable
	
	mpu9250_delay_ms(10);
	mpu9250_write_reg(dev,39, 0x00);//disable
	
	mpu9250_read_muhali(dev,73, (uint8_t*)buff, 3);
	*x = buff[0];
	*y = buff[1];
	*z = buff[2];
	
	mpu9250_ak8963_write_reg(dev,0X0A, 0X00);//Power-down mode
}
void mpu9250_ak8963_read_data(struct hal_dev_s *dev,int16_t *mx, int16_t *my, int16_t *mz)
{
	uint8_t buff[6];
	
	mpu9250_write_reg(dev,37, 0X8C);//AK8975 Address READ
	mpu9250_write_reg(dev,38, 0X03);//AK8975 HXL
	mpu9250_write_reg(dev,39, 0x86);//enable
	
	mpu9250_delay_ms(10);
	mpu9250_write_reg(dev,39, 0x00);//disable
	
	mpu9250_read_muhali(dev,73, buff, 6);
    *mx = (((int16_t)buff[1]) << 8) | buff[0];
    *my = (((int16_t)buff[3]) << 8) | buff[2];
    *mz = (((int16_t)buff[5]) << 8) | buff[4];
}

//检测AK8963是否存在
//返回值:0，成功;1，失败	
int mpu9250_ak8963_check(struct hal_dev_s *dev)
{
	uint8_t aid=0;
	
	mpu9250_ak8963_read_reg(dev,0x00,&aid);
    if(aid == 0x48)
        return 0;
    else 
        return -1;
}

//返回值:0，成功;1，失败	
uint8_t mpu9250_ak8963_selftest(struct hal_dev_s *dev)
{
	int16_t smx, smy, smz;
	
	if(mpu9250_ak8963_check(dev))
	{
		return 1;
	}
	mpu9250_ak8963_write_reg(dev,0X0A, 0X00);//Power-down mode
	mpu9250_ak8963_write_reg(dev,0X0C, 0X40);//SELF TEST
	mpu9250_ak8963_write_reg(dev,0X0A, 0X18);//16BIT Self-test Mode
	
	mpu9250_delay_ms(100);
	mpu9250_ak8963_read_data(dev,&smx, &smy, &smz);
	
	mpu9250_ak8963_write_reg(dev,0X0C, 0X00);
	mpu9250_ak8963_write_reg(dev,0X0A, 0X00);//Power-down mode
	
	if(smx < -200 || smx > 200)
		return 1;
	if(smy < -200 || smy > 200)
		return 1;
	if(smz < -3200 || smz > -800)
		return 1;
	
	return 0;
}

//MPU9250设置电子罗盘连续测量自动读取
//void mpu9250_mag_measure_continous(void)
//{
//	//SLV0
//	mpu9250_write_reg(dev,37, 0X8C);//AK8975 Address READ
//	mpu9250_write_reg(dev,38, 0X03);//AK8975 HXL
//	mpu9250_write_reg(dev,39, 0x86);//enable		

//	//SLV1
//	mpu9250_write_reg(dev,40, 0X0C);//AK8975 Address READ
//	mpu9250_write_reg(dev,41, 0X0A);//AK8975 CNTL1
//	mpu9250_write_reg(dev,42, 0x81);//enable	
//	mpu9250_write_reg(dev,100, 0X11);//16BIT Single measurement mode 
//	
//	mpu9250_write_reg(dev,52, 0x04);//I2C_MST_DLY = 4
//	mpu9250_write_reg(dev,103, 0x03);//I2C_SLV0_DLY_EN 
//}


//检测MPU9250是否存在
//返回值:0，成功;1，失败	
uint8_t mpu9250_check(struct hal_dev_s *dev)
{
	uint8_t mid = 0;
	
	mpu9250_read_reg(dev,117, &mid);
	if(mid == 0x71)
		return 0;
	else
		return 1;
}

static bool mpu9250_evaluate_selftest(float low, float high, float value, char* string)
{
	if (value < low || value > high)
	{
		return false;
	}
	return true;
}

//返回值:0，成功;1，失败	
int mpu9250_selftest(struct hal_dev_s *dev)
{
	uint8_t asel,gsel;
	
	struct mpu9250_report dat;
	float axf, ayf, azf;
	float gxf, gyf, gzf;
	float axfTst, ayfTst, azfTst;
	float gxfTst, gyfTst, gzfTst;
	float axfDiff, ayfDiff, azfDiff;
	float gxfDiff, gyfDiff, gzfDiff;
	float gRange, aRange;
	uint32_t scrap;
	
	aRange = mpu9250_get_full_scale_accel_GPL(dev);
	gRange = mpu9250_get_full_scale_gyro_DPL(dev);

	// First values after startup can be read as zero. Scrap a couple to be sure.
	for (scrap = 0; scrap < 20; scrap++)
	{
		mpu9250_measure(dev,&dat);
		mpu9250_delay_ms(2);
	}
	// First measurement
	gxf = (float)dat.gyro[0] * gRange;
	gyf = (float)dat.gyro[1] * gRange;
	gzf = (float)dat.gyro[2] * gRange;
	axf = (float)dat.acc[0]  * aRange;
	ayf = (float)dat.acc[1]  * aRange;
	azf = (float)dat.acc[2]  * aRange;

	// Enable self test
	mpu9250_read_reg(dev,27, &gsel);
	mpu9250_read_reg(dev,28, &asel);
	mpu9250_write_reg(dev,27, 0xE0|gsel);
	mpu9250_write_reg(dev,28, 0xE0|asel);


	// Wait for self test to take effect
	mpu9250_delay_ms(10);
	// Take second measurement
	mpu9250_measure(dev,&dat);
	gxfTst = (float)dat.gyro[0] * gRange;
	gyfTst = (float)dat.gyro[1] * gRange;
	gzfTst = (float)dat.gyro[2] * gRange;
	axfTst = (float)dat.acc[0]  * aRange;
	ayfTst = (float)dat.acc[1]  * aRange;
	azfTst = (float)dat.acc[2]  * aRange;

	// Disable self test
	mpu9250_write_reg(dev,27, gsel);
	mpu9250_write_reg(dev,28, asel);

	// Calculate difference
	gxfDiff = gxfTst - gxf;
	gyfDiff = gyfTst - gyf;
	gzfDiff = gzfTst - gzf;
	axfDiff = axfTst - axf;
	ayfDiff = ayfTst - ayf;
	azfDiff = azfTst - azf;

	// Check resuhal
	if (mpu9250_evaluate_selftest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gxfDiff, "gyro X") &&
	mpu9250_evaluate_selftest(-MPU6500_ST_GYRO_HIGH, -MPU6500_ST_GYRO_LOW, gyfDiff, "gyro Y") &&
	mpu9250_evaluate_selftest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gzfDiff, "gyro Z") &&
	mpu9250_evaluate_selftest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, axfDiff, "acc X") &&
	mpu9250_evaluate_selftest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, ayfDiff, "acc Y") &&
	mpu9250_evaluate_selftest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, azfDiff, "acc Z"))
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

void mpu9250_set_gyro_dlpf_freq(struct hal_dev_s *dev,unsigned int freq)
{
    uint8_t tmp = 0;
    mpu9250_read_reg(dev,26,&tmp);
    tmp &= 0xf8;
    if(freq > 250)tmp |= 0x07;
    else if (freq <=250 && freq >184)tmp |= 0x00;
    else if (freq <=184 && freq >92)tmp |= 0x01;
    else if (freq <=92 && freq >41)tmp |= 0x02;
    else if (freq <=41 && freq >20)tmp |= 0x03;
    else if (freq <=20 && freq >10)tmp |= 0x04;
    else if (freq <=10 && freq >5)tmp |= 0x05;
    else if (freq <=5 )tmp |= 0x06;
    mpu9250_write_reg(dev,26,tmp);
}

void mpu9250_set_accel_dlpf_freq(struct hal_dev_s *dev,unsigned int freq)
{
    uint8_t tmp = 0;
    if(freq > 460)tmp |= 0x08;
    else if (freq <=460 && freq >184)tmp |= 0x00;
    else if (freq <=184 && freq >92)tmp |= 0x01;
    else if (freq <=92 && freq >41)tmp |= 0x02;
    else if (freq <=41 && freq >20)tmp |= 0x03;
    else if (freq <=20 && freq >10)tmp |= 0x04;
    else if (freq <=10 && freq >5)tmp |= 0x05;
    else if (freq <=5 )tmp |= 0x06;
    mpu9250_write_reg(dev,29,tmp);
}

void mpu9250_set_gyro_range(struct hal_dev_s *dev,unsigned int range)
{
    uint8_t tmp = 0;
    mpu9250_read_reg(dev,27,&tmp);
    tmp &= 0xe7;
    if(range == 2000)      tmp |= 0x18;
    else if (range == 1000)tmp = 0x10;
    else if (range == 500) tmp = 0x08;
    else if (range == 250) tmp = 0x00;
    else return;
    mpu9250_write_reg(dev,27,tmp);
}

void mpu9250_set_accel_range(struct hal_dev_s *dev,unsigned int range)
{
    uint8_t tmp = 0;
    if(range == 16)     tmp = 0x18;
    else if (range == 8)tmp = 0x10;
    else if (range == 4)tmp = 0x08;
    else if (range == 2)tmp = 0x00;
    else return;
    mpu9250_write_reg(dev,28,tmp);
}

void mpu9250_measure(struct hal_dev_s *dev,struct mpu9250_report *dat)
{
	uint8_t buffer[21];
	
	mpu9250_read_muhali(dev,59, buffer, 21);
	dat->acc[0] = (((int16_t) buffer[0]) << 8) | buffer[1];
	dat->acc[1] = (((int16_t) buffer[2]) << 8) | buffer[3];
	dat->acc[2] = (((int16_t) buffer[4]) << 8) | buffer[5];
	
	dat->temp   = (((int16_t) buffer[6]) << 8) | buffer[7];
	
	dat->gyro[0] = (((int16_t) buffer[8]) << 8) | buffer[9];
	dat->gyro[1] = (((int16_t) buffer[10]) << 8) | buffer[11];
    dat->gyro[2] = (((int16_t) buffer[12]) << 8) | buffer[13];
	
	dat->mag[0] = (((int16_t) buffer[16]) << 8) | buffer[15];
	dat->mag[1] = (((int16_t) buffer[18]) << 8) | buffer[17];
	dat->mag[2] = (((int16_t) buffer[20]) << 8) | buffer[19];
}

int mpu9250_init(struct hal_dev_s *dev)
{
    uint8_t buff[3];
	struct hal_spi_cfg_s cfg;
    uint8_t id = 0;
	mpu9250_int_pin_init();
    
	//mpu9250_int_pin_init();
	cfg.width = 8;
    cfg.mode = HAL_SPI_MODE_3 | HAL_SPI_MSB;
	
    cfg.speed = 1000000;
	
    hal_spi_configure(dev,&cfg);
    
	mpu9250_write_reg(dev,107, 0x80);//Reset
	mpu9250_delay_ms(100);
	mpu9250_write_reg(dev,107, 0x01);//Clock Source 
	mpu9250_write_reg(dev,108, 0x00);//Enable Acc & Gyro
	
	mpu9250_write_reg(dev,56, 0x01);//enabled RAW_RDY_EN Interrupt
	mpu9250_write_reg(dev,55, 0xB0);//disabled BYPASS  LOW INT
	
	mpu9250_write_reg(dev,106, 0x30);//I2C_MST_EN
	mpu9250_write_reg(dev,36, 0x4D);//I2C Speed 400 kHz
	
	mpu9250_write_reg(dev,25, 0x01);//SMPLRT_DIV
	mpu9250_write_reg(dev,26, 0x01);//Bandwidth = 184Hz, FS=1KHz
	mpu9250_write_reg(dev,27, 0x18);//2000 dps 
	mpu9250_write_reg(dev,28, 0x10);//8g 
	mpu9250_write_reg(dev,29, 0x00);//Bandwidth = 460Hz, FS=1KHz
	
    if(mpu9250_check(dev)){
        return -1;
    }
    if(mpu9250_ak8963_check(dev) != 0){
        //return -1;
    }
    
	mpu9250_ak8963_write_reg(dev,AK8963_CNTL2, 0x01);//Reset AK8963
    mpu9250_delay_ms(1);
    mpu9250_ak8963_write_reg(dev,AK8963_CNTL1,0x10);
    mpu9250_delay_ms(1);
	mpu9250_ak8963_write_reg(dev,AK8963_CNTL1,0x1f);
    mpu9250_ak8963_read_reg(dev,AK8963_CNTL1,&id);
    mpu9250_delay_ms(1);
    mpu9250_ak8963_read_regs(dev,AK8963_ASAX,buff,3);
    
    mpu9250_ak8963_write_reg(dev,AK8963_CNTL1, 0x16);       // Continuous measurement mode 2
    
    mpu9250_write_reg(dev,MPU6500_I2C_MST_CTRL, 0x5D);    
    mpu9250_delay_ms(1);
    mpu9250_write_reg(dev,MPU6500_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
    mpu9250_delay_ms(1);
    mpu9250_write_reg(dev,MPU6500_I2C_SLV0_REG, AK8963_ST1);
    mpu9250_delay_ms(1);
    mpu9250_write_reg(dev,MPU6500_I2C_SLV0_CTRL, MPU6500_I2C_SLVx_EN | 8);
    mpu9250_delay_ms(1);

    
    mpu9250_delay_ms(1);

	mpu9250_write_reg(dev,MPU6500_I2C_SLV4_CTRL, 0x09);
	mpu9250_delay_ms(1);
	mpu9250_write_reg(dev,MPU6500_I2C_MST_DELAY_CTRL, 0x81);
	mpu9250_delay_ms(1);
	
    cfg.speed = 10000000;
	
    hal_spi_configure(dev,&cfg);
	return 0;
}



static int mpu9250_open(struct hal_dev_s *dev, uint16_t oflag)
{
	return 0;
}

static int mpu9250_close(struct hal_dev_s *dev)
{
	return 0;
}

static int mpu9250_read(struct hal_dev_s *dev,  void* buffer, int size,int pos)
{
    uint8_t num = size / sizeof(struct imu_report_s);
    struct mpu9250_report rept;

	struct imu_report_s *imu = (struct imu_report_s*)buffer;
	struct mpu6050_config_s *config = dev->priv_data;
	
    while(num--){
        mpu9250_measure(dev,&rept);
		
		imu->acc.accel[0] = rept.acc[0] * config->acc_scale;
		imu->acc.accel[1] = rept.acc[1] * config->acc_scale;
		imu->acc.accel[2] = rept.acc[2] * config->acc_scale;
		
		imu->gyro.gyro[0] = rept.gyro[0] * config->gyro_scale;
		imu->gyro.gyro[1] = rept.gyro[1] * config->gyro_scale;
		imu->gyro.gyro[2] = rept.gyro[2] * config->gyro_scale;

		imu->acc.temperature  = 36.53f + (float)rept.temp / 340.0f;
		imu->gyro.temperature = imu->acc.temperature;
		
        imu++;
    }
	return size;
}

static int mpu9250_write (struct hal_dev_s *dev, const void* buffer, int size,int pos)
{
    return 0;
}

static int mpu9250_control(struct hal_dev_s *dev, uint8_t cmd, void *args)
{
    uint32_t *dat = args;
    float    *f   = args;
    
    switch(cmd){
        case MPU9250_CMD_SET_ACCEL_RANGE: 
            mpu9250_set_accel_range(dev,*dat);
            break;
        case MPU9250_CMD_SET_GYRO_RANGE: 
            mpu9250_set_gyro_range(dev,*dat);
            break;
        case MPU9250_CMD_SET_ACCEL_DLPF:
            mpu9250_set_accel_dlpf_freq(dev,*dat);
            break;
        case MPU9250_CMD_SET_GYRO_DLPF:
            mpu9250_set_gyro_dlpf_freq(dev,*dat);
            break;
        case MPU9250_CMD_GET_ACCEL_SCALE:
            *f = mpu9250_get_full_scale_accel_GPL(dev);
            break;
        case MPU9250_CMD_GET_GYRO_SCALE:
            *f = mpu9250_get_full_scale_gyro_DPL(dev);
            break;
        case MPU9250_CMD_GET_MAG_SCALE:
            *f = mpu9250_get_full_scale_mag();
            break;
        default:break;
    }
    return 0;
}

static struct hal_spi_dev_s       mpu9250_spi;
static struct mpu6050_config_s   mpu6050_config;

int mpu9250_device_init(void)
{
	mpu9250_spi.port       = 0;
	mpu9250_spi.cfg.speed  = 10000000;
	mpu9250_spi.cfg.mode   = HAL_SPI_MODE_3 | HAL_SPI_MSB;
	mpu9250_spi.cfg.width  = 8;
	mpu9250_spi.cs_take    = mpu9250_cs_take;
	mpu9250_spi.cs_release = mpu9250_cs_release;
	
    mpu9250_spi.dev.init  = mpu9250_init;
    mpu9250_spi.dev.open  = mpu9250_open;
    mpu9250_spi.dev.close = mpu9250_close;
    mpu9250_spi.dev.write = mpu9250_write;
    mpu9250_spi.dev.read  = mpu9250_read;
    mpu9250_spi.dev.ioctl = mpu9250_control;
	mpu9250_spi.dev.priv_data = &mpu6050_config;
	
	mpu6050_config.acc_range  = 8.0f;
	mpu6050_config.gyro_range = 2000.0f;
	mpu6050_config.acc_scale  = 0.002387768f;
	mpu6050_config.gyro_scale = 0.001065185f;
    
    return hal_spi_device_register(&mpu9250_spi,"mpu9250",
			HAL_O_RDWR | HAL_DEV_REMOVABLE | HAL_DEV_STANDALONE);
}
