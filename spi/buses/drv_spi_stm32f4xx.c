#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "hal_spi.h"

struct stm32f4xx_spi_gpio
{
	uint16_t      pin;
	GPIO_TypeDef* port;
	uint16_t      GPIO_PinSource;
	uint8_t       GPIO_AF;
	uint32_t      RCC_APB2Periph;
};

struct stm32f4xx_spi_adapter{
	SPI_TypeDef*             	SPIx;
	uint32_t 					rcc;
	void 					  (*rcc_cmd)(uint32_t rcc, FunctionalState NewState);
	struct stm32f4xx_spi_gpio 	mosi;
	struct stm32f4xx_spi_gpio 	miso;
	struct stm32f4xx_spi_gpio 	sck;
};

static void  stm32f4_spi_gpio_init(struct stm32f4xx_spi_adapter *dev)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd((dev->mosi.RCC_APB2Periph) |
						   (dev->miso.RCC_APB2Periph) |
						   (dev->sck.RCC_APB2Periph)  , ENABLE);

	GPIO_PinAFConfig(dev->mosi.port, dev->mosi.GPIO_PinSource,dev->mosi.GPIO_AF);
	GPIO_PinAFConfig(dev->miso.port, dev->miso.GPIO_PinSource,dev->miso.GPIO_AF);
	GPIO_PinAFConfig(dev->sck.port , dev->sck.GPIO_PinSource ,dev->sck.GPIO_AF);
	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //for emi
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; //NO PULL, pull up will let sdcard unsteady

	/* Configure SPI1 pins */
	GPIO_InitStructure.GPIO_Pin = dev->mosi.pin;
	GPIO_Init(dev->mosi.port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = dev->miso.pin;
	GPIO_Init(dev->miso.port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = dev->sck.pin;
	GPIO_Init(dev->sck.port, &GPIO_InitStructure);
}

static int  stm32f4_spi_init(struct hal_spi_adapter_s *dev)
{
	
	struct stm32f4xx_spi_adapter *adpt = dev->user_data;
	
	SPI_InitTypeDef SPI_InitStructure;

	adpt->rcc_cmd(adpt->rcc, ENABLE);
	
	stm32f4_spi_gpio_init(adpt);

    SPI_StructInit(&SPI_InitStructure);
	
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;

    /* init SPI */
    SPI_I2S_DeInit(adpt->SPIx);
    SPI_Init(adpt->SPIx, &SPI_InitStructure);
    /* Enable SPI_MASTER */
    SPI_Cmd(adpt->SPIx, ENABLE);
    SPI_CalculateCRC(adpt->SPIx, DISABLE);
	return 0;
}


static void stm32f4_spi_configure(struct hal_spi_adapter_s *dev,
                          struct hal_spi_cfg_s *cfg)
{
    struct stm32f4xx_spi_adapter *adpt = dev->user_data;
	
    SPI_InitTypeDef SPI_InitStructure;

    SPI_StructInit(&SPI_InitStructure);

    /* data_width */
    if(cfg->width <= 8)
    {
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    }
    else if(cfg->width <= 16)
    {
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    }
    else
    {
        return;
    }

	uint32_t stm32_spi_max_clock = 37500000;
	uint32_t stm32_apb2_clock  = SystemCoreClock / 4; 
	uint32_t speed = cfg->speed; 

	if(speed > stm32_spi_max_clock)
	{
		speed = stm32_spi_max_clock;
	}


	if(speed >= stm32_apb2_clock/2 && stm32_apb2_clock/2 <= 30000000)
	{
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	}
	else if(speed >= stm32_apb2_clock/4)
	{
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	}
	else if(speed >= stm32_apb2_clock/8)
	{
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	}
	else if(speed >= stm32_apb2_clock/16)
	{
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	}
	else if(speed >= stm32_apb2_clock/32)
	{
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	}
	else if(speed >= stm32_apb2_clock/64)
	{
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	}
	else if(speed >= stm32_apb2_clock/128)
	{
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	}
	else
	{
		/*  min prescaler 256 */
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	}

    /* CPOL */
    if(cfg->mode & HAL_SPI_CPOL)
    {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    }
    else
    {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    }
    /* CPHA */
    if(cfg->mode & HAL_SPI_CPHA)
    {
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    }
    else
    {
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    }
    /* MSB or LSB */
    if(cfg->mode & HAL_SPI_MSB)
    {
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    }
    else
    {
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
    }
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;

    /* init SPI */
    SPI_I2S_DeInit(adpt->SPIx);
    SPI_Init(adpt->SPIx, &SPI_InitStructure);
    /* Enable SPI_MASTER */
    SPI_Cmd(adpt->SPIx, ENABLE);
    SPI_CalculateCRC(adpt->SPIx, DISABLE);
}

static int stm32f4_spi_transfer(struct hal_spi_adapter_s *dev, struct hal_spi_msg_s* message)
{
   struct stm32f4xx_spi_adapter *adpt = dev->user_data;

    SPI_TypeDef * SPI = adpt->SPIx;
    uint32_t size = message->length;
	
    {
        if(dev->owner->cfg.width <= 8)
        {
            const uint8_t * send_ptr = message->send_buffer;
            uint8_t * recv_ptr = message->recv_buffer;

            while(size--)
            {
                uint8_t data = 0xFF;

                if(send_ptr != NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
                // Send the byte
                SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
                // Get the received data
                data = SPI_I2S_ReceiveData(SPI);

                if(recv_ptr != NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
        else if(dev->owner->cfg.width <= 16)
        {
            const uint16_t * send_ptr = (const uint16_t*)message->send_buffer;
            uint16_t * recv_ptr = (uint16_t*)message->recv_buffer;

            while(size--)
            {
                uint16_t data = 0xFF;

                if(send_ptr != NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
                // Send the byte
                SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
                // Get the received data
                data = SPI_I2S_ReceiveData(SPI);

                if(recv_ptr != NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
    }

    return message->length;
};


/* stm32 SPI1 GPIO define */
struct stm32f4xx_spi_gpio spi1_mosi_pina = {
	.pin 			= GPIO_Pin_5,
	.port 			= GPIOA,
	.GPIO_PinSource = GPIO_PinSource5,
	.GPIO_AF		= GPIO_AF_SPI1,
	.RCC_APB2Periph = RCC_AHB1Periph_GPIOA,
};

struct stm32f4xx_spi_gpio spi1_miso_pina = {
	.pin 			= GPIO_Pin_6,
	.port 			= GPIOA,
	.GPIO_PinSource = GPIO_PinSource6,
	.GPIO_AF		= GPIO_AF_SPI1,
	.RCC_APB2Periph = RCC_AHB1Periph_GPIOA,
};

struct stm32f4xx_spi_gpio spi1_sck_pina = {
	.pin 			= GPIO_Pin_7,
	.port 			= GPIOA,
	.GPIO_PinSource = GPIO_PinSource7,
	.GPIO_AF		= GPIO_AF_SPI1,
	.RCC_APB2Periph = RCC_AHB1Periph_GPIOA,
};
/* end of stm32 SPI1 GPIO define */

struct stm32f4xx_spi_adapter stm32f4_spi1; 
struct hal_spi_adapter_s		 spi1_bus; 


/* stm32 SPI2 GPIO define */
struct stm32f4xx_spi_gpio spi2_mosi_pina = {
	.pin 			= GPIO_Pin_15,
	.port 			= GPIOB,
	.GPIO_PinSource = GPIO_PinSource15,
	.GPIO_AF		= GPIO_AF_SPI2,
	.RCC_APB2Periph = RCC_AHB1Periph_GPIOB,
};

struct stm32f4xx_spi_gpio spi2_miso_pina = {
	.pin 			= GPIO_Pin_14,
	.port 			= GPIOB,
	.GPIO_PinSource = GPIO_PinSource14,
	.GPIO_AF		= GPIO_AF_SPI2,
	.RCC_APB2Periph = RCC_AHB1Periph_GPIOB,
};

struct stm32f4xx_spi_gpio spi2_sck_pina = {
	.pin 			= GPIO_Pin_13,
	.port 			= GPIOB,
	.GPIO_PinSource = GPIO_PinSource13,
	.GPIO_AF		= GPIO_AF_SPI2,
	.RCC_APB2Periph = RCC_AHB1Periph_GPIOB,
};
/* end of stm32 SPI2 GPIO define */
struct stm32f4xx_spi_adapter stm32f4_spi2; 
struct hal_spi_adapter_s		 spi2_bus; 


/* stm32 SPI3 GPIO define */
struct stm32f4xx_spi_gpio spi3_mosi_pina = {
	.pin 			= GPIO_Pin_12,
	.port 			= GPIOC,
	.GPIO_PinSource = GPIO_PinSource12,
	.GPIO_AF		= GPIO_AF_SPI3,
	.RCC_APB2Periph = RCC_AHB1Periph_GPIOC,
};

struct stm32f4xx_spi_gpio spi3_miso_pina = {
	.pin 			= GPIO_Pin_11,
	.port 			= GPIOC,
	.GPIO_PinSource = GPIO_PinSource11,
	.GPIO_AF		= GPIO_AF_SPI3,
	.RCC_APB2Periph = RCC_AHB1Periph_GPIOC,
};

struct stm32f4xx_spi_gpio spi3_sck_pina = {
	.pin 			= GPIO_Pin_10,
	.port 			= GPIOC,
	.GPIO_PinSource = GPIO_PinSource10,
	.GPIO_AF		= GPIO_AF_SPI3,
	.RCC_APB2Periph = RCC_AHB1Periph_GPIOC,
};
/* end of stm32 SPI3 GPIO define */
struct stm32f4xx_spi_adapter stm32f4_spi3; 
struct hal_spi_adapter_s		 spi3_bus; 


int stm32_spi_init(void)
{
	stm32f4_spi1.SPIx 			= SPI1;
	stm32f4_spi1.rcc        	= RCC_APB2Periph_SPI1;
	stm32f4_spi1.rcc_cmd        = RCC_APB2PeriphClockCmd;
	stm32f4_spi1.mosi			= spi1_mosi_pina;
	stm32f4_spi1.miso			= spi1_miso_pina;
    stm32f4_spi1.sck			= spi1_sck_pina;
	
	spi1_bus.init 		= stm32f4_spi_init;
	spi1_bus.configure 	= stm32f4_spi_configure;
	spi1_bus.transfer  	= stm32f4_spi_transfer;
	spi1_bus.owner      = NULL;
	spi1_bus.user_data  = &stm32f4_spi1;
	
	hal_spi_adapter_register(&spi1_bus,0);
	
	stm32f4_spi2.SPIx 			= SPI2;
	stm32f4_spi2.rcc         	= RCC_APB1Periph_SPI2;
	stm32f4_spi2.rcc_cmd        = RCC_APB1PeriphClockCmd;
	stm32f4_spi2.mosi			= spi2_mosi_pina;
	stm32f4_spi2.miso			= spi2_miso_pina;
    stm32f4_spi2.sck			= spi2_sck_pina;
	
	spi2_bus.init 		= stm32f4_spi_init;
	spi2_bus.configure 	= stm32f4_spi_configure;
	spi2_bus.transfer  	= stm32f4_spi_transfer;
	spi2_bus.owner      = NULL;
	spi2_bus.user_data  = &stm32f4_spi2;
	
	hal_spi_adapter_register(&spi2_bus,1);
	
	stm32f4_spi3.SPIx 			= SPI3;
	stm32f4_spi3.rcc         	= RCC_APB1Periph_SPI3;
	stm32f4_spi3.rcc_cmd        = RCC_APB1PeriphClockCmd;
	stm32f4_spi3.mosi			= spi3_mosi_pina;
	stm32f4_spi3.miso			= spi3_miso_pina;
    stm32f4_spi3.sck			= spi3_sck_pina;
	
	spi3_bus.init 		= stm32f4_spi_init;
	spi3_bus.configure 	= stm32f4_spi_configure;
	spi3_bus.transfer  	= stm32f4_spi_transfer;
	spi3_bus.owner      = NULL;
	spi3_bus.user_data  = &stm32f4_spi3;
	
	hal_spi_adapter_register(&spi3_bus,2);
	return 0;
};


