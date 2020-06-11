#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"
#include "hal_device.h"
#include "hal_serial.h"

#define STM32F4XX_UART_BUFFER_LENGTH   64

struct stm32f411x_gpio_s{
	uint32_t      pin;
	GPIO_TypeDef* port;
	uint32_t      alternate;
	uint32_t      clock;
};

struct stm32f411x_uart_config{
    USART_TypeDef               *USARTx;
    uint32_t                     clock;
    void                       (*enable_clock)(uint32_t Periphs);
    struct stm32f411x_gpio_s 	*txd;
	struct stm32f411x_gpio_s 	*rxd;
    uint8_t                     irq_pre_pri;
	uint8_t                     irq_sub_pri;   
    uint32_t                    new_data_from_int;
    IRQn_Type                   irq;	
};

void stm32f411x_uart_gpio_init(struct stm32f411x_uart_config *port)
{
    LL_GPIO_InitTypeDef   GPIO_InitStruct  = {0};
    
	LL_AHB1_GRP1_EnableClock(port->txd->clock);
    LL_AHB1_GRP1_EnableClock(port->rxd->clock);
    
    GPIO_InitStruct.Pin   = port->txd->pin;
    GPIO_InitStruct.Mode  = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = port->txd->alternate;
    LL_GPIO_Init(port->txd->port, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin   = port->rxd->pin;
    GPIO_InitStruct.Alternate  = port->rxd->alternate;
    LL_GPIO_Init(port->rxd->port, &GPIO_InitStruct);
}	
	

int stm32f411x_uart_config(struct stm32f411x_uart_config *port,struct hal_serial_cfg_s *cfg)
{
    LL_USART_InitTypeDef  USART_InitStruct = {0};
	
	if(port->enable_clock != NULL){
		port->enable_clock(port->clock);
	}else{
		return -1;
	}
    
    USART_InitStruct.BaudRate   = cfg->buad_rate;

    if (cfg->data_bits == DATA_BITS_8)
        USART_InitStruct.DataWidth  = LL_USART_DATAWIDTH_8B;

    if (cfg->stop_bits == STOP_BITS_1)
        USART_InitStruct.StopBits   = LL_USART_STOPBITS_1;
    else if (cfg->stop_bits == STOP_BITS_2)
        USART_InitStruct.StopBits   = LL_USART_STOPBITS_2;
    
    if(cfg->parity == PARITY_EVEN)
        USART_InitStruct.Parity     = LL_USART_PARITY_EVEN;
    else if(cfg->parity == PARITY_ODD)
        USART_InitStruct.Parity     = LL_USART_PARITY_ODD;
    else 
        USART_InitStruct.Parity     = LL_USART_PARITY_NONE;
    
    USART_InitStruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling        = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(port->USARTx, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(port->USARTx);
    
	return 0;
}
static void stm32f411x_irq_config(struct hal_serial_s *serial)
{   
    struct stm32f411x_uart_config *port = (struct stm32f411x_uart_config *)serial->priv;

    NVIC_SetPriority(port->irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),port->irq_pre_pri, port->irq_sub_pri));
    NVIC_EnableIRQ(port->irq);
	
	LL_USART_DisableIT_TXE(port->USARTx);
    LL_USART_DisableIT_TC(port->USARTx);
    LL_USART_DisableIT_RXNE(port->USARTx);
	/* clear interrupt */
    LL_USART_ClearFlag_RXNE(port->USARTx);
	/* clear interrupt */
    LL_USART_ClearFlag_TC(port->USARTx);
}

int stm32f4_uart_init(struct hal_serial_s *serial)
{
	struct stm32f411x_uart_config *port = (struct stm32f411x_uart_config *)serial->priv;
	
	stm32f411x_uart_gpio_init(port);
	stm32f411x_uart_config(port,&serial_defauhal_cfg);
	stm32f411x_irq_config(serial);
	LL_USART_Enable(port->USARTx);

	return 0;
}

int stm32f4_putc(struct hal_serial_s *serial,char ch)
{
    struct stm32f411x_uart_config *port = (struct stm32f411x_uart_config *)serial->priv;
	
	while(!LL_USART_IsActiveFlag_TXE(port->USARTx));
    LL_USART_TransmitData8(port->USARTx,ch);
	return 1;
}

int stm32f4_getc(struct hal_serial_s *serial,char *ch)
{
    struct stm32f411x_uart_config *port = (struct stm32f411x_uart_config *)serial->priv;
	
    if(LL_USART_IsActiveFlag_RXNE(port->USARTx) != RESET)
    {
        *ch = LL_USART_ReceiveData8(port->USARTx);
        port->new_data_from_int = 0;
		return 1;
    }
    
	if(port->new_data_from_int & 0x10000000){
		*ch = port->new_data_from_int & 0x00000ff;
		port->new_data_from_int = 0;
		return 1;
	}
	return 0;
}

void stm32f4_enable_irq(struct hal_serial_s *serial,int irq)
{
	struct stm32f411x_uart_config *port = (struct stm32f411x_uart_config *)serial->priv;
	if(irq == SERIAL_INT_TXDONE){
        LL_USART_EnableIT_TC(port->USARTx);
	}else if(irq == SERIAL_INT_RXDONE){
		LL_USART_EnableIT_RXNE(port->USARTx);
	}else if(irq == SERIAL_INT_DMATXDONE){
		/*  not support for now */
	}
}

void stm32f4_disable_irq(struct hal_serial_s *serial,int irq)
{
	struct stm32f411x_uart_config *port = (struct stm32f411x_uart_config *)serial->priv;
	if(irq == SERIAL_INT_TXDONE){
		LL_USART_DisableIT_TC(port->USARTx);
	}else if(irq == SERIAL_INT_RXDONE){
		LL_USART_DisableIT_RXNE(port->USARTx);
	}else if(irq == SERIAL_INT_DMATXDONE){
		/*  not support for now */
	}
}

struct stm32f411x_gpio_s uart1_txd_pin = {
	.pin 			= LL_GPIO_PIN_9,
	.port 			= GPIOA,
	.alternate      = LL_GPIO_AF_7,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOA,
};

struct stm32f411x_gpio_s uart1_rxd_pin = {
	.pin 			= LL_GPIO_PIN_10,
	.port 			= GPIOA,
	.alternate      = LL_GPIO_AF_7,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOA,
};

struct stm32f411x_uart_config uart1 = {
	.USARTx          = USART1,
	.txd             = &uart1_txd_pin,
	.rxd             = &uart1_rxd_pin,
	.clock           = LL_APB2_GRP1_PERIPH_USART1, 
	.enable_clock    = LL_APB2_GRP1_EnableClock,
	.irq_pre_pri     = 5,
	.irq_sub_pri     = 0,
	.irq             = USART1_IRQn,
};

struct hal_serial_s stm32_serial1 = {
	.putc = stm32f4_putc,
	.getc = stm32f4_getc,
	.enable_irq  = stm32f4_enable_irq,
	.disable_irq = stm32f4_disable_irq, 
	.init = stm32f4_uart_init,
	.priv = &uart1,
};


struct stm32f411x_gpio_s uart2_txd_pin = {
	.pin 			= LL_GPIO_PIN_5,
	.port 			= GPIOD,
	.alternate      = LL_GPIO_AF_7,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOD,
};

struct stm32f411x_gpio_s uart2_rxd_pin = {
	.pin 			= LL_GPIO_PIN_6,
	.port 			= GPIOD,
	.alternate      = LL_GPIO_AF_7,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOD,
};

struct stm32f411x_uart_config uart2 = {
	.USARTx          = USART2,
	.txd             = &uart2_txd_pin,
	.rxd             = &uart2_rxd_pin,
	.clock           = LL_APB1_GRP1_PERIPH_USART2, 
	.enable_clock    = LL_APB1_GRP1_EnableClock,
	.irq_pre_pri     = 6,
	.irq_sub_pri     = 0,
	.irq             = USART2_IRQn,
};

struct hal_serial_s stm32_serial2 = {
	.putc = stm32f4_putc,
	.getc = stm32f4_getc,
	.enable_irq  = stm32f4_enable_irq,
	.disable_irq = stm32f4_disable_irq, 
	.init = stm32f4_uart_init,
	.priv = &uart2,
};


struct stm32f411x_gpio_s uart3_txd_pin = {
	.pin 			= LL_GPIO_PIN_8,
	.port 			= GPIOD,
	.alternate      = LL_GPIO_AF_7,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOD,
};

struct stm32f411x_gpio_s uart3_rxd_pin = {
	.pin 			= LL_GPIO_PIN_9,
	.port 			= GPIOD,
	.alternate      = LL_GPIO_AF_7,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOD,
};


struct stm32f411x_gpio_s uart3_txd_pin2 = {
	.pin 			= LL_GPIO_PIN_10,
	.port 			= GPIOB,
	.alternate      = LL_GPIO_AF_7,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOB,
};

struct stm32f411x_gpio_s uart3_rxd_pin2 = {
	.pin 			= LL_GPIO_PIN_11,
	.port 			= GPIOB,
	.alternate      = LL_GPIO_AF_7,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOB,
};

struct stm32f411x_uart_config uart3 = {
	.USARTx          = USART3,
	.txd             = &uart3_txd_pin2,
	.rxd             = &uart3_rxd_pin2,
	.clock           = LL_APB1_GRP1_PERIPH_USART3, 
	.enable_clock    = LL_APB1_GRP1_EnableClock,
	.irq_pre_pri     = 4,
	.irq_sub_pri     = 0,
	.irq             = USART3_IRQn,
};

struct hal_serial_s stm32_serial3 = {
	.putc = stm32f4_putc,
	.getc = stm32f4_getc,
	.enable_irq  = stm32f4_enable_irq,
	.disable_irq = stm32f4_disable_irq, 
	.init = stm32f4_uart_init,
	.priv = &uart3,
};


struct stm32f411x_gpio_s uart4_txd_pin = {
	.pin 			= LL_GPIO_PIN_0,
	.port 			= GPIOA,
	.alternate      = LL_GPIO_AF_8,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOA,
};

struct stm32f411x_gpio_s uart4_rxd_pin = {
	.pin 			= LL_GPIO_PIN_1,
	.port 			= GPIOA,
	.alternate      = LL_GPIO_AF_8,
	.clock          = LL_AHB1_GRP1_PERIPH_GPIOA,
};

struct stm32f411x_uart_config uart4 = {
	.USARTx          = UART4,
	.txd             = &uart4_txd_pin,
	.rxd             = &uart4_rxd_pin,
	.clock           = LL_APB1_GRP1_PERIPH_UART4, 
	.enable_clock    = LL_APB1_GRP1_EnableClock,
	.irq_pre_pri     = 7,
	.irq_sub_pri     = 0,
	.irq             = UART4_IRQn,
};

struct hal_serial_s stm32_serial4 = {
	.putc = stm32f4_putc,
	.getc = stm32f4_getc,
	.enable_irq  = stm32f4_enable_irq,
	.disable_irq = stm32f4_disable_irq, 
	.init = stm32f4_uart_init,
	.priv = &uart4,
};

void USART1_IRQHandler(void)
{
	struct stm32f411x_uart_config *port = (struct stm32f411x_uart_config *)stm32_serial1.priv;
    
	/* enter interrupt */
    //hal_enter_interrupt();
	
    if(LL_USART_IsActiveFlag_RXNE(port->USARTx) != RESET){
        port->new_data_from_int  = 0x10000000;
		port->new_data_from_int |= (LL_USART_ReceiveData8(port->USARTx) & 0x0000ffff);
        serial_isr(&stm32_serial1, SERIAL_INT_RXDONE);
        LL_USART_ClearFlag_RXNE(port->USARTx);
    }
    
    if(LL_USART_IsActiveFlag_TC(port->USARTx) != RESET){
        serial_isr(&stm32_serial1, SERIAL_INT_TXDONE);
        LL_USART_ClearFlag_TC(port->USARTx);
    }

    /* exit interrupt */
    //hal_exit_interrupt();
}


void USART2_IRQHandler(void)
{
	struct stm32f411x_uart_config *port = (struct stm32f411x_uart_config *)stm32_serial2.priv;
    
	/* enter interrupt */
    //hal_enter_interrupt();
	
    if(LL_USART_IsActiveFlag_RXNE(port->USARTx) != RESET){
        port->new_data_from_int  = 0x10000000;
		port->new_data_from_int |= (LL_USART_ReceiveData8(port->USARTx) & 0x0000ffff);
        serial_isr(&stm32_serial2, SERIAL_INT_RXDONE);
        LL_USART_ClearFlag_RXNE(port->USARTx);
    }
    
    if(LL_USART_IsActiveFlag_TC(port->USARTx) != RESET){
        serial_isr(&stm32_serial2, SERIAL_INT_TXDONE);
        LL_USART_ClearFlag_TC(port->USARTx);
    }

    /* exit interrupt */
    //hal_exit_interrupt();
}


void USART3_IRQHandler(void)
{
	struct stm32f411x_uart_config *port = (struct stm32f411x_uart_config *)stm32_serial3.priv;
    
	/* enter interrupt */
    //hal_enter_interrupt();
	
    if(LL_USART_IsActiveFlag_RXNE(port->USARTx) != RESET){
        port->new_data_from_int  = 0x10000000;
		port->new_data_from_int |= (LL_USART_ReceiveData8(port->USARTx) & 0x0000ffff);
        serial_isr(&stm32_serial3, SERIAL_INT_RXDONE);
        LL_USART_ClearFlag_RXNE(port->USARTx);
    }
    
    if(LL_USART_IsActiveFlag_TC(port->USARTx) != RESET){
        serial_isr(&stm32_serial3, SERIAL_INT_TXDONE);
        LL_USART_ClearFlag_TC(port->USARTx);
    }

    /* exit interrupt */
    //hal_exit_interrupt();
}

void UART4_IRQHandler(void)
{
	struct stm32f411x_uart_config *port = (struct stm32f411x_uart_config *)stm32_serial4.priv;
    
	/* enter interrupt */
    //hal_enter_interrupt();
	
    if(LL_USART_IsActiveFlag_RXNE(port->USARTx) != RESET){
        port->new_data_from_int  = 0x10000000;
		port->new_data_from_int |= (LL_USART_ReceiveData8(port->USARTx) & 0x0000ffff);
        serial_isr(&stm32_serial4, SERIAL_INT_RXDONE);
        LL_USART_ClearFlag_RXNE(port->USARTx);
    }
    
    if(LL_USART_IsActiveFlag_TC(port->USARTx) != RESET){
        serial_isr(&stm32_serial4, SERIAL_INT_TXDONE);
        LL_USART_ClearFlag_TC(port->USARTx);
    }

    /* exit interrupt */
    //hal_exit_interrupt();
}


void stm32f4_serial_init(void)
{
	//serial_device_register(&stm32_serial1,"ttyS1",HAL_DEV_INT_RX | HAL_DEV_INT_TX | HAL_DEV_RDWR);
	//serial_device_register(&stm32_serial2,"ttyS2",HAL_DEV_INT_RX | HAL_DEV_INT_TX | HAL_DEV_RDWR);
	serial_device_register(&stm32_serial3,"ttyS3",HAL_DEV_INT_RX | HAL_DEV_INT_TX | HAL_DEV_RDWR);
	serial_device_register(&stm32_serial4,"ttyS4",HAL_DEV_INT_RX | HAL_DEV_INT_TX | HAL_DEV_RDWR);
}





