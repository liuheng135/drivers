#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "hal_i2c.h"

struct stm32f4xx_i2c_gpio
{
	uint16_t      pin;
	GPIO_TypeDef* port;
	uint16_t      GPIO_PinSource;
	uint8_t       GPIO_AF;
	uint32_t      RCC_APB2Periph;
};

struct stm32f4xx_i2c_adapter{
	I2C_TypeDef*             	I2Cx;
	uint32_t 					rcc;
	void 					  (*rcc_cmd)(uint32_t rcc, FunctionalState NewState);
	struct stm32f4xx_i2c_gpio 	sda;
	struct stm32f4xx_i2c_gpio 	scl;
};


static void  stm32f4_i2c_gpio_init(struct stm32f4xx_i2c_adapter *dev)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd((dev->sda.RCC_APB2Periph) |
						   (dev->scl.RCC_APB2Periph) , ENABLE);

	GPIO_PinAFConfig(dev->sda.port, dev->sda.GPIO_PinSource,dev->sda.GPIO_AF);
	GPIO_PinAFConfig(dev->scl.port, dev->scl.GPIO_PinSource,dev->scl.GPIO_AF);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //for emi
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; //NO PULL, pull up will let sdcard unsteady

	/* Configure SPI1 pins */
	GPIO_InitStructure.GPIO_Pin = dev->scl.pin;
	GPIO_Init(dev->scl.port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = dev->sda.pin;
	GPIO_Init(dev->sda.port, &GPIO_InitStructure);
}

static int  stm32f4_i2c_init(struct hal_i2c_adapter_s *dev)
{
	struct stm32f4xx_i2c_adapter *adpt = dev->user_data;
	
	I2C_InitTypeDef I2C_InitStructure;

	adpt->rcc_cmd(adpt->rcc, ENABLE);
	
	stm32f4_i2c_gpio_init(adpt);

    I2C_StructInit(&I2C_InitStructure);
	
	I2C_InitStructure.I2C_Mode  = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle  = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1  = 0x00;
	I2C_InitStructure.I2C_Ack  = I2C_Ack_Enable;
	I2C_InitStructure.I2C_ClockSpeed  = 400 * 1000;
    I2C_InitStructure.I2C_AcknowledgedAddress  = I2C_AcknowledgedAddress_7bit;

    /* init SPI */
    I2C_DeInit(adpt->I2Cx);
    I2C_Init(adpt->I2Cx, &I2C_InitStructure);
    /* Enable SPI_MASTER */
    I2C_Cmd(adpt->I2Cx, ENABLE);
	
	return 0;
}


static int stm32f4_i2c_configure(struct hal_i2c_adapter_s *dev, struct hal_i2c_cfg_s *cfg)
{
    return 0;
}

static int stm32f4_i2c_transfer(struct hal_i2c_adapter_s *adpt,struct hal_i2c_msg_s *msg,int num)
{
    return 0;
}

/* end of stm32 SPI3 GPIO define */
struct stm32f4xx_i2c_adapter stm32f4_i2c1; 
struct hal_i2c_adapter_s		 i2c1_bus; 


int stm32_i2c_init(void)
{
	i2c1_bus.init      = stm32f4_i2c_init;
	i2c1_bus.configure = stm32f4_i2c_configure;
	i2c1_bus.transfer  = stm32f4_i2c_transfer;
	i2c1_bus.user_data = &stm32f4_i2c1;
	
	hal_i2c_adapter_register(&i2c1_bus,0);
	
	return 0;
}

