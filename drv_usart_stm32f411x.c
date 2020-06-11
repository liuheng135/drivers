#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"
#include "hal_device.h"

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
};



int uart_init(struct hal_dev_s *dev)
{
    struct stm32f411x_uart_config *cfg = (struct stm32f411x_uart_config *)dev->priv_data;
    
    LL_USART_InitTypeDef  USART_InitStruct = {0};
    LL_GPIO_InitTypeDef   GPIO_InitStruct  = {0};
    
    cfg->enable_clock(cfg->clock);
    LL_AHB1_GRP1_EnableClock(cfg->txd->clock);
    LL_AHB1_GRP1_EnableClock(cfg->rxd->clock);
    
    GPIO_InitStruct.Pin   = cfg->txd->pin;
    GPIO_InitStruct.Mode  = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = cfg->txd->alternate;
    LL_GPIO_Init(cfg->txd->port, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin   = cfg->rxd->pin;
    GPIO_InitStruct.Alternate  = cfg->rxd->alternate;
    LL_GPIO_Init(cfg->rxd->port, &GPIO_InitStruct);
    
    USART_InitStruct.BaudRate   = 115200;
    USART_InitStruct.DataWidth  = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits   = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity     = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling        = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(cfg->USARTx, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(cfg->USARTx);
    LL_USART_Enable(cfg->USARTx);
    
    return 0;
}

int uart_open(struct hal_dev_s *dev, uint16_t oflag)
{
    return 0;
}

int uartd_write(struct hal_dev_s *dev, const void *buffer, int size,int pos)
{
    int i;
    char *str;
    struct stm32f411x_uart_config *cfg = (struct stm32f411x_uart_config *)dev->priv_data;
    
    str = (char *)buffer;
    for(i = 0;i < size;i++){
        while(!LL_USART_IsActiveFlag_TXE(cfg->USARTx));
        LL_USART_TransmitData8(cfg->USARTx,str[i]);
    }
    return size;
}

struct hal_dev_s stm32f411x_uart1;
struct stm32f411x_uart_config uart1_config;
/* stm32 UART1 GPIO define */
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

struct hal_dev_s stm32f411x_uart2;
struct stm32f411x_uart_config uart2_config;
/* stm32 UART1 GPIO define */
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


int uart_register(void)
{
    uart1_config.USARTx       = USART1;
    uart1_config.enable_clock = LL_APB2_GRP1_EnableClock;
    uart1_config.clock        = LL_APB2_GRP1_PERIPH_USART1;
    uart1_config.txd          = &uart1_txd_pin;
    uart1_config.rxd          = &uart1_rxd_pin;
    
	stm32f411x_uart1.init  = uart_init;
	stm32f411x_uart1.open  = uart_open;
	stm32f411x_uart1.write = uartd_write;
    stm32f411x_uart1.priv_data = &uart1_config;

	//hal_dev_register(&stm32f411x_uart1,"uart1",HAL_O_RDWR | HAL_DEV_STANDALONE);
	
	uart2_config.USARTx       = USART2;
    uart2_config.enable_clock = LL_APB1_GRP1_EnableClock;
    uart2_config.clock        = LL_APB1_GRP1_PERIPH_USART2;
    uart2_config.txd          = &uart2_txd_pin;
    uart2_config.rxd          = &uart2_rxd_pin;
    
	stm32f411x_uart2.init  = uart_init;
	stm32f411x_uart2.open  = uart_open;
	stm32f411x_uart2.write = uartd_write;
    stm32f411x_uart2.priv_data = &uart2_config;

	hal_dev_register(&stm32f411x_uart2,"uart2",HAL_O_RDWR | HAL_DEV_STANDALONE);
	return 0;
}

