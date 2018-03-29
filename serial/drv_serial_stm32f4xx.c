#include "hal_serial.h"
#include "stm32f4xx.h"

#define STM32F4XX_UART_BUFFER_LENGTH   64

struct stm32f4xx_uart_gpio
{
	uint16_t      pin;
	GPIO_TypeDef* port;
	uint16_t      GPIO_PinSource;
	uint8_t       GPIO_AF;
	uint32_t      GPIO_Clock;
};

struct stm32f4xx_uart_port{
	USART_TypeDef*             	UARTx;
	uint32_t 					rcc_peri;
	void 						(*rcc_cmd)(uint32_t rcc_peri, FunctionalState NewState);
	struct stm32f4xx_uart_gpio 	*txd;
	struct stm32f4xx_uart_gpio 	*rxd;
	uint8_t                     irq_pre_pri;
	uint8_t                     irq_sub_pri;   
	IRQn_Type                   irq;	
	DMA_Stream_TypeDef         *tx_dma_channel;	
	IRQn_Type                   DMA_irq;
	uint32_t                    new_data_from_int;
};

void stm32f4_uart_gpio_init(struct stm32f4xx_uart_port *port)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd((port->txd->GPIO_Clock) |(port->rxd->GPIO_Clock) , ENABLE);

	GPIO_PinAFConfig(port->txd->port, port->txd->GPIO_PinSource,port->txd->GPIO_AF);
	GPIO_PinAFConfig(port->rxd->port, port->rxd->GPIO_PinSource,port->rxd->GPIO_AF);
	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //for emi
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; //NO PULL, pull up will let sdcard unsteady

	/* Configure SPI1 pins */
	GPIO_InitStructure.GPIO_Pin = port->txd->pin;
	GPIO_Init(port->txd->port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = port->rxd->pin;
	GPIO_Init(port->rxd->port, &GPIO_InitStructure);
}	
	

int stm32f4_uart_config(struct hal_serial_s *serial,struct hal_serial_cfg_s *cfg)
{
	struct stm32f4xx_uart_port *port = (struct stm32f4xx_uart_port *)serial->priv;
	USART_InitTypeDef USART_InitStructure;


    USART_InitStructure.USART_BaudRate = cfg->buad_rate;

    if (cfg->data_bits == DATA_BITS_8)
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;

    if (cfg->stop_bits == STOP_BITS_1)
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    else if (cfg->stop_bits == STOP_BITS_2)
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
    
    if(cfg->parity == PARITY_EVEN)
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    else if(cfg->parity == PARITY_ODD)
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
    else 
        USART_InitStructure.USART_Parity = USART_Parity_No;
    
    
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(port->UARTx, &USART_InitStructure);
	
	return 0;
}
static void stm32f4_irq_config(struct hal_serial_s *serial)
{
	struct stm32f4xx_uart_port *port = (struct stm32f4xx_uart_port *)serial->priv;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = port->irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = port->irq_pre_pri;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = port->irq_sub_pri;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	
	USART_ITConfig(port->UARTx, USART_IT_TC, DISABLE);
	USART_ITConfig(port->UARTx, USART_IT_TXE, DISABLE);
	USART_ITConfig(port->UARTx, USART_IT_RXNE, ENABLE);
	
	/* clear interrupt */
    USART_ClearITPendingBit(port->UARTx, USART_IT_RXNE);
	/* clear interrupt */
    USART_ClearITPendingBit(port->UARTx, USART_IT_TC);
	
	NVIC_EnableIRQ(port->irq);
	/*
    NVIC_InitStructure.NVIC_IRQChannel = port->DMA_irq;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	*/
}

int stm32f4_uart_init(struct hal_serial_s *serial)
{
	struct stm32f4xx_uart_port *port = (struct stm32f4xx_uart_port *)serial->priv;
	
	stm32f4_uart_gpio_init(port);
	port->rcc_cmd(port->rcc_peri,ENABLE);
	
	stm32f4_uart_config(serial,&serial_defauhal_cfg);
	stm32f4_irq_config(serial);
	
	USART_Cmd(port->UARTx,ENABLE);

	return 0;
}

int stm32f4_putc(struct hal_serial_s *serial,char ch)
{
	struct stm32f4xx_uart_port *port = (struct stm32f4xx_uart_port *)serial->priv;
	
	while((USART_GetFlagStatus(port->UARTx, USART_FLAG_TXE) == RESET));
	USART_SendData(port->UARTx,ch);
	return 1;
}

int stm32f4_getc(struct hal_serial_s *serial,char *ch)
{
	struct stm32f4xx_uart_port *port = (struct stm32f4xx_uart_port *)serial->priv;
	
	if(USART_GetFlagStatus(port->UARTx, USART_FLAG_RXNE) != RESET){
		*ch = (char)USART_ReceiveData(port->UARTx);
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
	struct stm32f4xx_uart_port *port = (struct stm32f4xx_uart_port *)serial->priv;
	if(irq == SERIAL_INT_TXDONE){
		USART_ITConfig(port->UARTx, USART_IT_TC, ENABLE);
	}else if(irq == SERIAL_INT_RXDONE){
		USART_ITConfig(port->UARTx, USART_IT_RXNE, ENABLE);
	}else if(irq == SERIAL_INT_DMATXDONE){
		/*  not support for now */
	}
}

void stm32f4_disable_irq(struct hal_serial_s *serial,int irq)
{
	struct stm32f4xx_uart_port *port = (struct stm32f4xx_uart_port *)serial->priv;
	if(irq == SERIAL_INT_TXDONE){
		USART_ITConfig(port->UARTx, USART_IT_TC, DISABLE);
	}else if(irq == SERIAL_INT_RXDONE){
		USART_ITConfig(port->UARTx, USART_IT_RXNE, DISABLE);
	}else if(irq == SERIAL_INT_DMATXDONE){
		/*  not support for now */
	}
}



struct stm32f4xx_uart_gpio uart1_txd_pin_1 = {
	.pin              = GPIO_Pin_9,
	.port             = GPIOA,
	.GPIO_PinSource   = GPIO_PinSource9,
	.GPIO_AF          = GPIO_AF_USART1,
	.GPIO_Clock       = RCC_AHB1Periph_GPIOA,
};

struct stm32f4xx_uart_gpio uart1_rxd_pin_1 = {
	.pin              = GPIO_Pin_10,
	.port             = GPIOA,
	.GPIO_PinSource   = GPIO_PinSource10,
	.GPIO_AF          = GPIO_AF_USART1,
	.GPIO_Clock       = RCC_AHB1Periph_GPIOA,
};

struct stm32f4xx_uart_gpio uart1_txd_pin_2 = {
	.pin              = GPIO_Pin_6,
	.port             = GPIOB,
	.GPIO_PinSource   = GPIO_PinSource6,
	.GPIO_AF          = GPIO_AF_USART1,
	.GPIO_Clock       = RCC_AHB1Periph_GPIOB,
};

struct stm32f4xx_uart_gpio uart1_rxd_pin_2 = {
	.pin              = GPIO_Pin_7,
	.port             = GPIOB,
	.GPIO_PinSource   = GPIO_PinSource7,
	.GPIO_AF          = GPIO_AF_USART1,
	.GPIO_Clock       = RCC_AHB1Periph_GPIOB,
};

struct stm32f4xx_uart_port uart1 = {
	.UARTx           = USART1,
	.txd             = &uart1_txd_pin_1,
	.rxd             = &uart1_rxd_pin_1,
	.rcc_peri        = RCC_APB2Periph_USART1, 
	.rcc_cmd         = RCC_APB2PeriphClockCmd,
	.irq_pre_pri     = 14,
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

void USART1_IRQHandler(void)
{
	struct stm32f4xx_uart_port *port = (struct stm32f4xx_uart_port *)stm32_serial1.priv;
    
	/* enter interrupt */
    //hal_enter_interrupt();
	
    if (USART_GetITStatus(port->UARTx, USART_IT_RXNE) != RESET)
    {
		port->new_data_from_int  = 0x10000000;
		port->new_data_from_int |= (USART_ReceiveData(port->UARTx) & 0x0000ffff);
        serial_isr(&stm32_serial1, SERIAL_INT_RXDONE);
        /* clear interrupt */
        USART_ClearITPendingBit(port->UARTx, USART_IT_RXNE);
    }
	
    if (USART_GetITStatus(port->UARTx, USART_IT_TC) != RESET)
    {
		serial_isr(&stm32_serial1, SERIAL_INT_TXDONE);
        /* clear interrupt */
        USART_ClearITPendingBit(port->UARTx, USART_IT_TC);
    }

    /* exit interrupt */
    //hal_exit_interrupt();
}

void stm32f4_serial_init(void)
{
	serial_device_register(&stm32_serial1,"ttyS1",HAL_DEV_INT_RX | HAL_DEV_INT_TX | HAL_DEV_RDWR);
}





