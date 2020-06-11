#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"
#include "hal_i2c.h"

struct stm32f411x_gpio_s
{
	uint32_t      pin;
	GPIO_TypeDef* port;
	uint32_t      alternate;
	uint32_t      clock;
};


struct stm32f4xx_i2c_adapter{
	struct stm32f411x_gpio_s 	*sda;
	struct stm32f411x_gpio_s 	*scl;
	uint32_t delay_num;
};

static void siic_delay_us(volatile uint32_t us)
{
	while(us)
		us--;
}

void stm32f4xx_set_gpio(struct stm32f411x_gpio_s *gpio)
{
	LL_GPIO_SetOutputPin(gpio->port,gpio->pin);
}

void stm32f4xx_reset_gpio(struct stm32f411x_gpio_s *gpio)
{
	LL_GPIO_ResetOutputPin(gpio->port,gpio->pin);
}

void stm32f4xx_set_gpio_out(struct stm32f411x_gpio_s *gpio)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.Pin  = gpio->pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(gpio->port, &GPIO_InitStruct);
}

void stm32f4xx_set_gpio_in(struct stm32f411x_gpio_s *gpio)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.Pin  = gpio->pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(gpio->port, &GPIO_InitStruct);
}

void IIC_Start(struct stm32f4xx_i2c_adapter *adpt)
{
	stm32f4xx_set_gpio_out(adpt->sda);     //sda?????
	stm32f4xx_set_gpio(adpt->sda);	  
	stm32f4xx_set_gpio(adpt->scl);
	siic_delay_us(adpt->delay_num);
 	stm32f4xx_reset_gpio(adpt->sda);   //START:when CLK is high,DATA change form high to low 
	siic_delay_us(adpt->delay_num);
	stm32f4xx_reset_gpio(adpt->scl);   
}

void IIC_Stop(struct stm32f4xx_i2c_adapter *adpt)
{
	stm32f4xx_set_gpio_out(adpt->sda);
	stm32f4xx_reset_gpio(adpt->scl);
	stm32f4xx_reset_gpio(adpt->sda);
 	siic_delay_us(adpt->delay_num);
	stm32f4xx_set_gpio(adpt->scl);
    siic_delay_us(adpt->delay_num);
	stm32f4xx_set_gpio(adpt->sda);
	siic_delay_us(adpt->delay_num);							   	
}


uint8_t IIC_Wait_Ack(struct stm32f4xx_i2c_adapter *adpt)
{
	uint16_t ucErrTime = 0;
	stm32f4xx_set_gpio(adpt->sda);
    siic_delay_us(adpt->delay_num);
	stm32f4xx_set_gpio_in(adpt->sda);
    siic_delay_us(adpt->delay_num);    
	stm32f4xx_set_gpio(adpt->scl);
    siic_delay_us(adpt->delay_num); 	
	while(LL_GPIO_IsInputPinSet(adpt->sda->port,adpt->sda->pin))
	{
		ucErrTime++;
		if(ucErrTime > 400)
		{
			IIC_Stop(adpt);
			return 1;     // no ack
		}
	}
	stm32f4xx_reset_gpio(adpt->scl);
	return 0;  
}

void IIC_Ack(struct stm32f4xx_i2c_adapter *adpt)
{
	stm32f4xx_reset_gpio(adpt->scl);
	stm32f4xx_set_gpio_out(adpt->sda);
	stm32f4xx_reset_gpio(adpt->sda);
	siic_delay_us(adpt->delay_num);
	stm32f4xx_set_gpio(adpt->scl);
	siic_delay_us(adpt->delay_num);
	stm32f4xx_reset_gpio(adpt->scl);
}
		    
void IIC_NAck(struct stm32f4xx_i2c_adapter *adpt)
{
    stm32f4xx_set_gpio_out(adpt->sda);
	stm32f4xx_reset_gpio(adpt->scl);
    siic_delay_us(adpt->delay_num);
	stm32f4xx_set_gpio(adpt->sda);
	siic_delay_us(adpt->delay_num);
	stm32f4xx_set_gpio(adpt->scl);
	siic_delay_us(adpt->delay_num);
	stm32f4xx_reset_gpio(adpt->scl);
}					 				     
	  
void IIC_Send_Byte(struct stm32f4xx_i2c_adapter *adpt,uint8_t txd)
{                        
	uint8_t t;
	stm32f4xx_set_gpio_out(adpt->sda);
    stm32f4xx_reset_gpio(adpt->scl); 
    siic_delay_us(1);
    for(t=0;t<8;t++)
    {              
		if ((txd&0x80)){
            stm32f4xx_set_gpio(adpt->sda);
        }else{
            stm32f4xx_reset_gpio(adpt->sda);
        }
        txd<<=1; 	  
		siic_delay_us(adpt->delay_num);  
		stm32f4xx_set_gpio(adpt->scl);
		siic_delay_us(adpt->delay_num); 
		stm32f4xx_reset_gpio(adpt->scl);	
    }	 
} 	    
//??1??????ack=1???????ACK??ack=0??????nACK   
uint8_t IIC_Read_Byte(struct stm32f4xx_i2c_adapter *adpt,unsigned char ack)
{
	unsigned char i,receive=0;
	stm32f4xx_set_gpio_in(adpt->sda);//SDA?????????
    for(i=0;i<8;i++ )
	{
        stm32f4xx_reset_gpio(adpt->scl); 
        siic_delay_us(adpt->delay_num);
		stm32f4xx_set_gpio(adpt->scl);
        receive<<=1;
        siic_delay_us(adpt->delay_num);
        if(LL_GPIO_IsInputPinSet(adpt->sda->port,adpt->sda->pin)){
			receive|=0x01;
		}			
    }					 
    if (!ack)
        IIC_NAck(adpt); 
    else
        IIC_Ack(adpt);  
    return receive;
}


int stm32f4xx_soft_i2c_adapter_init(struct hal_i2c_adapter_s *hal_adpt)
{
	struct stm32f4xx_i2c_adapter *adpt = (struct stm32f4xx_i2c_adapter *)hal_adpt->user_data;
	
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	LL_AHB1_GRP1_EnableClock(adpt->scl->clock | adpt->sda->clock);
	
	GPIO_InitStruct.Pin        = adpt->scl->pin;
	GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
	LL_GPIO_Init(adpt->scl->port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin        = adpt->sda->pin;
	LL_GPIO_Init(adpt->sda->port, &GPIO_InitStruct);
		
	stm32f4xx_set_gpio(adpt->scl);
	stm32f4xx_set_gpio(adpt->sda);
    return 0;
}

int stm32f4xx_soft_i2c_adapter_configure(struct hal_i2c_adapter_s *adpt,struct hal_i2c_cfg_s *cfg)
{
	struct stm32f4xx_i2c_adapter  *stm_adpt =(struct stm32f4xx_i2c_adapter *)adpt->user_data;
	
	if(cfg->speed >= 400000 && cfg->speed < 1000000){
		stm_adpt->delay_num = 7;
	}
	
	if(cfg->speed >= 1000000){
		stm_adpt->delay_num = 2;
	}
	
    return 0;
}

int stm32f4xx_soft_i2c_adapter_transfer(struct hal_i2c_adapter_s *hal_adpt,struct hal_i2c_msg_s *msg,int num)
{
    int i,j;
    struct stm32f4xx_i2c_adapter *adpt = (struct stm32f4xx_i2c_adapter *)hal_adpt->user_data;
	
    for(i = 0;i < num;i++){
        if(msg[i].flags & HAL_I2C_RD){
            IIC_Start(adpt);
            IIC_Send_Byte(adpt,msg[i].addr << 1 | 0x01);
            if(IIC_Wait_Ack(adpt))
				return -1;

            for(j = 0;j < msg[i].length;j++){
                if(j == (msg[i].length - 1)){
                    msg[i].buff[j] = IIC_Read_Byte(adpt,0);
                }else{
                    msg[i].buff[j] = IIC_Read_Byte(adpt,1);
                }
            }
        }else{  
            IIC_Start(adpt);
            IIC_Send_Byte(adpt,msg[i].addr << 1);
            if(IIC_Wait_Ack(adpt))
				return -1;
            for(j = 0;j < msg[i].length;j++){
                IIC_Send_Byte(adpt,msg[i].buff[j]);
                if(IIC_Wait_Ack(adpt))
                    return -1;
            }
        }
        
    }
    IIC_Stop(adpt);
    return num;
}

struct stm32f411x_gpio_s i2c1_scl = {
	.pin = LL_GPIO_PIN_8,
	.port = GPIOB,
	.clock = LL_AHB1_GRP1_PERIPH_GPIOB,
};

struct stm32f411x_gpio_s i2c1_sda = {
	.pin = LL_GPIO_PIN_9,
	.port = GPIOB,
	.clock = LL_AHB1_GRP1_PERIPH_GPIOB,
};

/* end of stm32 SPI3 GPIO define */
struct stm32f4xx_i2c_adapter stm32f4_i2c1; 
struct hal_i2c_adapter_s		 i2c1_bus; 

int stm32f4xx_i2c_init(void)
{
	stm32f4_i2c1.scl = &i2c1_scl;
	stm32f4_i2c1.sda = &i2c1_sda;
	stm32f4_i2c1.delay_num = 1;
	
	i2c1_bus.init      = stm32f4xx_soft_i2c_adapter_init;
	i2c1_bus.configure = stm32f4xx_soft_i2c_adapter_configure;
	i2c1_bus.transfer  = stm32f4xx_soft_i2c_adapter_transfer;
	i2c1_bus.user_data = &stm32f4_i2c1;
	
	hal_i2c_adapter_register(&i2c1_bus,0);
	
	return 0;
}

