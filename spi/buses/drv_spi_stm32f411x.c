#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"
#include "hal_spi.h"

struct stm32f411x_gpio_s
{
	uint32_t      pin;
	GPIO_TypeDef* port;
	uint32_t      alternate;
	uint32_t      clock;
};

struct stm32f4xx_spi_adapter{
	SPI_TypeDef*             	SPIx;
    uint32_t                    clock;
	void                      (*enable_clock)(uint32_t Periphs);
	struct stm32f411x_gpio_s 	mosi;
	struct stm32f411x_gpio_s 	miso;
	struct stm32f411x_gpio_s 	sck;
};

static void  stm32f4_spi_gpio_init(struct stm32f4xx_spi_adapter *dev)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    LL_AHB1_GRP1_EnableClock(dev->miso.clock);
    LL_AHB1_GRP1_EnableClock(dev->mosi.clock);
    LL_AHB1_GRP1_EnableClock(dev->sck.clock);

    GPIO_InitStruct.Pin = dev->sck.pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = dev->sck.alternate;
    LL_GPIO_Init(dev->sck.port, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = dev->miso.pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = dev->miso.alternate;
    LL_GPIO_Init(dev->miso.port, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = dev->mosi.pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = dev->mosi.alternate;
    LL_GPIO_Init(dev->mosi.port, &GPIO_InitStruct);
	
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
}

static void stm32f4_spi_cs_take(int id)
{
	if(id == 1){
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0);
	}
}

static void stm32f4_spi_cs_release(int id)
{
	if(id == 1){
		LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0);
	}
}	

static int  stm32f4_spi_init(struct hal_spi_adapter_s *dev)
{
	struct stm32f4xx_spi_adapter *adpt = dev->user_data;
    LL_SPI_InitTypeDef SPI_InitStruct = {0};
	
	if(adpt->enable_clock != NULL){
		adpt->enable_clock(adpt->clock);
	}else{
		return -1;
	}
        
	stm32f4_spi_gpio_init(adpt);

    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 10;
    LL_SPI_Init(adpt->SPIx, &SPI_InitStruct);
    LL_SPI_SetStandard(adpt->SPIx, LL_SPI_PROTOCOL_MOTOROLA);

	return 0;
}


static void stm32f4_spi_configure(struct hal_spi_adapter_s *dev,
                          struct hal_spi_cfg_s *cfg)
{
    struct stm32f4xx_spi_adapter *adpt = dev->user_data;
	uint32_t stm32_apb_clock;
    LL_SPI_InitTypeDef SPI_InitStruct = {0};

    /* data_width */
    if(cfg->width <= 8){
        SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    }else if(cfg->width <= 16){
        SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
    }else{
        return;
    }

	if(adpt->SPIx == SPI1){
		stm32_apb_clock = 84000000; 
	}else{
		stm32_apb_clock = 42000000;
	}		
	
	uint32_t speed = cfg->speed; 

	if(speed > dev->max_speed){
		speed = dev->max_speed;
	}

	if(speed >= stm32_apb_clock/2){
		SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
	}else if(speed >= stm32_apb_clock/4){
		SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
	}else if(speed >= stm32_apb_clock/8){
		SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
	}else if(speed >= stm32_apb_clock/16){
		SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
	}else if(speed >= stm32_apb_clock/32){
		SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
	}else if(speed >= stm32_apb_clock/64){
		SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
	}else if(speed >= stm32_apb_clock/128){
		SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV128;
	}else{
		SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
	}

    /* CPOL */
    if(cfg->mode & HAL_SPI_CPOL){
        SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
    }else{
        SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    }
    /* CPHA */
    if(cfg->mode & HAL_SPI_CPHA){
        SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
    }else{
        SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    }
    /* MSB or LSB */
    if(cfg->mode & HAL_SPI_MSB){
        SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    }else{
        SPI_InitStruct.BitOrder = LL_SPI_LSB_FIRST;
    }
    
    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;

    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 10;
    LL_SPI_Init(adpt->SPIx, &SPI_InitStruct);
    LL_SPI_SetStandard(adpt->SPIx, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_Enable(adpt->SPIx);
}

static int stm32f4_spi_transfer(struct hal_spi_adapter_s *dev, struct hal_spi_msg_s* message)
{
   struct stm32f4xx_spi_adapter *adpt = dev->user_data;

    SPI_TypeDef *SPIx = adpt->SPIx;
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
                
                while((SPIx->SR & LL_SPI_SR_TXE) == 0); 
                SPIx->DR = data; 
                
                while((SPIx->SR & LL_SPI_SR_RXNE) == 0);
                data = SPIx->DR;
                

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

                while((SPI1->SR & LL_SPI_SR_TXE) == 0); 
                SPI1->DR = data; 
                
                while((SPI1->SR & LL_SPI_SR_RXNE) == 0);
                data = SPI1->DR;

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
struct stm32f411x_gpio_s spi1_mosi_pin = {
	.pin 			= LL_GPIO_PIN_5,
	.port 			= GPIOA,
	.alternate      = LL_GPIO_AF_5,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOA,
};

struct stm32f411x_gpio_s spi1_miso_pin = {
	.pin 			= LL_GPIO_PIN_6,
	.port 			= GPIOA,
	.alternate      = LL_GPIO_AF_5,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOA,
};

struct stm32f411x_gpio_s spi1_sck_pin = {
	.pin 			= LL_GPIO_PIN_7,
	.port 			= GPIOA,
	.alternate      = LL_GPIO_AF_5,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOA,
};
/* end of stm32 SPI1 GPIO define */


/* stm32 SPI2 GPIO define */
struct stm32f411x_gpio_s spi2_mosi_pin = {
	.pin 			= LL_GPIO_PIN_13,
	.port 			= GPIOB,
	.alternate      = LL_GPIO_AF_5,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOB,
};

struct stm32f411x_gpio_s spi2_miso_pin = {
	.pin 			= LL_GPIO_PIN_14,
	.port 			= GPIOB,
	.alternate      = LL_GPIO_AF_5,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOB,
};

struct stm32f411x_gpio_s spi2_sck_pin = {
	.pin 			= LL_GPIO_PIN_15,
	.port 			= GPIOB,
	.alternate      = LL_GPIO_AF_5,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOB,
};
/* end of stm32 SPI2 GPIO define */

struct stm32f4xx_spi_adapter stm32f4_spi1;
struct hal_spi_adapter_s		 spi1_bus; 

struct stm32f4xx_spi_adapter stm32f4_spi2;
struct hal_spi_adapter_s		 spi2_bus;

int stm32_spi_init(void)
{
	stm32f4_spi1.SPIx   = SPI1;
    stm32f4_spi1.mosi   = spi1_mosi_pin;
    stm32f4_spi1.miso   = spi1_miso_pin;
    stm32f4_spi1.sck    = spi1_sck_pin;
    stm32f4_spi1.clock  = LL_APB2_GRP1_PERIPH_SPI1;
	stm32f4_spi1.enable_clock = LL_APB2_GRP1_EnableClock;
    
	spi1_bus.init 		= stm32f4_spi_init;
	spi1_bus.configure 	= stm32f4_spi_configure;
	spi1_bus.transfer  	= stm32f4_spi_transfer;
	spi1_bus.cs_take    = stm32f4_spi_cs_take;
	spi1_bus.cs_release = stm32f4_spi_cs_release;
	spi1_bus.owner      = NULL;
	spi1_bus.user_data  = &stm32f4_spi1;
	spi1_bus.max_speed  = 42000000;
	
	hal_spi_adapter_register(&spi1_bus,0);
	
	stm32f4_spi2.SPIx   = SPI2;
    stm32f4_spi2.mosi   = spi2_mosi_pin;
    stm32f4_spi2.miso   = spi2_miso_pin;
    stm32f4_spi2.sck    = spi2_sck_pin;
    stm32f4_spi2.clock  = LL_APB1_GRP1_PERIPH_SPI2;
	stm32f4_spi2.enable_clock = LL_APB1_GRP1_EnableClock;
    
	spi2_bus.init 		= stm32f4_spi_init;
	spi2_bus.configure 	= stm32f4_spi_configure;
	spi2_bus.transfer  	= stm32f4_spi_transfer;
	spi2_bus.cs_take    = stm32f4_spi_cs_take;
	spi2_bus.cs_release = stm32f4_spi_cs_release;
	spi2_bus.owner      = NULL;
	spi2_bus.user_data  = &stm32f4_spi2;
	spi2_bus.max_speed  = 21000000;
	
	//hal_spi_adapter_register(&spi2_bus,1);
	return 0;
};


