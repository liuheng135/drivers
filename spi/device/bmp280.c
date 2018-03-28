#include "hal_spi.h"
#include "hal_sensor.h"
#include "stm32f4xx.h"

#define BMP280_CS_PORT              GPIOB
#define BMP280_CS_CLK               RCC_AHB1Periph_GPIOB
#define BMP280_CS_PIN               GPIO_Pin_3

#define BMP280_I2C_ADDR                      (0x76)
#define BMP280_DEFAULT_CHIP_ID               (0x58)

#define BMP280_CHIP_ID_REG                   (0xD0)  /* Chip ID Register */
#define BMP280_RST_REG                       (0xE0)  /* Softreset Register */
#define BMP280_STAT_REG                      (0xF3)  /* Status Register */
#define BMP280_CTRL_MEAS_REG                 (0xF4)  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG                    (0xF5)  /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG              (0xF7)  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG              (0xF8)  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG             (0xF9)  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG           (0xFA)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG           (0xFB)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG          (0xFC)  /* Temperature XLSB Reg */
#define BMP280_FORCED_MODE                   (0x01)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define BMP280_DATA_FRAME_SIZE               (6)

#define BMP280_OVERSAMP_SKIPPED          (0x00)
#define BMP280_OVERSAMP_1X               (0x01)
#define BMP280_OVERSAMP_2X               (0x02)
#define BMP280_OVERSAMP_4X               (0x03)
#define BMP280_OVERSAMP_8X               (0x04)
#define BMP280_OVERSAMP_16X              (0x05)

// configure pressure and temperature oversampling, forced sampling mode
#define BMP280_PRESSURE_OSR              (BMP280_OVERSAMP_8X)
#define BMP280_TEMPERATURE_OSR           (BMP280_OVERSAMP_1X)
#define BMP280_MODE                      (BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_FORCED_MODE)

#define T_INIT_MAX                       (20)
// 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX           (37)
// 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX             (10)
// 10/16 = 0.625 ms



struct bmp280_calib_param_s {
    uint16_t dig_T1; /* calibration T1 data */
    int16_t dig_T2; /* calibration T2 data */
    int16_t dig_T3; /* calibration T3 data */
    uint16_t dig_P1; /* calibration P1 data */
    int16_t dig_P2; /* calibration P2 data */
    int16_t dig_P3; /* calibration P3 data */
    int16_t dig_P4; /* calibration P4 data */
    int16_t dig_P5; /* calibration P5 data */
    int16_t dig_P6; /* calibration P6 data */
    int16_t dig_P7; /* calibration P7 data */
    int16_t dig_P8; /* calibration P8 data */
    int16_t dig_P9; /* calibration P9 data */
    int32_t t_fine; /* calibration t_fine data */
};

struct bmp280_config_s{
	struct bmp280_calib_param_s calib;
};

static void bmp280_pin_init(struct hal_dev_s *dev)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(BMP280_CS_CLK , ENABLE);
	 
	
	GPIO_InitStructure.GPIO_Pin = BMP280_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT  ;   
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(BMP280_CS_PORT, &GPIO_InitStructure);
}	

static void bmp280_cs_take(void)
{
	GPIO_ResetBits(BMP280_CS_PORT,BMP280_CS_PIN);
}

static void bmp280_cs_release(void)
{
	GPIO_SetBits(BMP280_CS_PORT,BMP280_CS_PIN);
}

void bmp280_write_reg(struct hal_dev_s *dev,uint8_t reg,uint8_t dat)
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

void bmp280_read_reg(struct hal_dev_s *dev,uint8_t reg,uint8_t *dat)
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

void bmp280_read_multi(struct hal_dev_s *dev,uint8_t reg, uint8_t *buff, uint8_t num)
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

int bmp280_init(struct hal_dev_s *dev)
{
	uint8_t bmp280_chip_id;
	struct bmp280_config_s *config = (struct bmp280_config_s *)dev->priv_data;
	
	bmp280_pin_init(dev);
	
	bmp280_read_reg(dev, BMP280_CHIP_ID_REG, &bmp280_chip_id);  /* read Chip Id */
	if(bmp280_chip_id != BMP280_DEFAULT_CHIP_ID){
		HAL_DEBUG("[BMP280] Can not find bmp280,chip id is wrong [0x%2x]\r\n",bmp280_chip_id);
		return -1;
	}
	HAL_DEBUG("[BMP280] bmp280 chip found\r\n");
	
	bmp280_read_multi(dev, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, (uint8_t *)&config->calib, 24);
	
	/* set oversampling + power mode (forced), and start sampling */
    bmp280_write_reg(dev, BMP280_CTRL_MEAS_REG, BMP280_MODE);
	
	return 0;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "51.23" equals 51.23 DegC
// t_fine carries fine temperature as global value
static float bmp280_compensate_T(struct bmp280_calib_param_s *calib,int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)calib->dig_T1 << 1))) * ((int32_t)calib->dig_T2)) >> 11;
    var2  = (((((adc_T >> 4) - ((int32_t)calib->dig_T1)) * ((adc_T >> 4) - ((int32_t)calib->dig_T1))) >> 12) * ((int32_t)calib->dig_T3)) >> 14;
    calib->t_fine = var1 + var2;
    T = (calib->t_fine * 5 + 128) >> 8;

    return (float)T/100.0f;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa

static float bmp280_compensate_P(struct bmp280_calib_param_s *calib,int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)calib->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib->dig_P6;
    var2 = var2 + ((var1*(int64_t)calib->dig_P5) << 17);
    var2 = var2 + (((int64_t)calib->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib->dig_P3) >> 8) + ((var1 * (int64_t)calib->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib->dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib->dig_P7) << 4);
    return (float)p/100.0f;
}

int bmp280_open(struct hal_dev_s *dev, uint16_t oflag)
{
	return 0;
}

int bmp280_close(struct hal_dev_s *dev)
{
	return 0;
}

int bmp280_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{
	struct baro_report_s *report = (struct baro_report_s *)buffer;
	uint8_t num = size / sizeof(struct imu_report_s);
	struct bmp280_config_s *config = (struct bmp280_config_s *)dev->priv_data;
	uint8_t data[BMP280_DATA_FRAME_SIZE];
	int32_t press;
	int32_t temp;
	
	while(num--){
		// read data from sensor
		bmp280_read_multi(dev, BMP280_PRESSURE_MSB_REG, data, BMP280_DATA_FRAME_SIZE);
		press = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
		temp = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));

		report->pressure = bmp280_compensate_P(&config->calib,press);
		report->temperature = bmp280_compensate_T(&config->calib,temp);
		report++;
	}
	
	return size;
}

int bmp280_write(struct hal_dev_s *dev, const void *buffer, int size,int pos)
{
	return 0;
}

int bmp280_ioctl(struct hal_dev_s *dev, uint8_t cmd, void *args)
{
	return 0;
}

static struct hal_spi_dev_s     bmp280_spi;
static struct bmp280_config_s   bmp280_config;

int dmp280_device_init(void)
{
	bmp280_spi.port       = 2;
	bmp280_spi.cfg.speed  = 10000000;
	bmp280_spi.cfg.mode   = HAL_SPI_MODE_3 | HAL_SPI_MSB;
	bmp280_spi.cfg.width  = 8;
	bmp280_spi.cs_take    = bmp280_cs_take;
	bmp280_spi.cs_release = bmp280_cs_release;
	
    bmp280_spi.dev.init   = bmp280_init;
    bmp280_spi.dev.open   = bmp280_open;
    bmp280_spi.dev.close  = bmp280_close;
    bmp280_spi.dev.write  = bmp280_write;
    bmp280_spi.dev.read   = bmp280_read;
    bmp280_spi.dev.ioctl  = bmp280_ioctl;
	bmp280_spi.dev.priv_data = &bmp280_config;
	
    
    return hal_spi_device_register(&bmp280_spi,"bmp280",
			HAL_O_RDWR | HAL_DEV_REMOVABLE | HAL_DEV_STANDALONE);
}
