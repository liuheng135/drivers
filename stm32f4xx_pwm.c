#include "hal_device.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"

static int stm32f4xx_pwm_init(struct hal_dev_s *dev)
{
	LL_TIM_InitTypeDef TIM_InitStruct = {0};
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
	LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

	TIM_InitStruct.Prescaler = 167;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 2500;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	TIM_InitStruct.RepetitionCounter = 0;
	LL_TIM_Init(TIM1, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM1);
	
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
	
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 1000;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_LOW;
	TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
	TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_HIGH;
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
	
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
	
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
	
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
	
	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM1);
	TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
	TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
	TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
	TIM_BDTRInitStruct.DeadTime = 0;
	TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
	TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
	TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
	LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
	
	/**TIM1 GPIO Configuration  
	PE14   ------> TIM1_CH4
	PE13   ------> TIM1_CH3
	PE11   ------> TIM1_CH2
	PE9    ------> TIM1_CH1 
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_14|LL_GPIO_PIN_13|LL_GPIO_PIN_11|LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
	return 0;
}

static int stm32f4xx_pwm_open(struct hal_dev_s *dev,uint16_t flag)
{
	LL_TIM_EnableCounter(TIM1);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableAllOutputs(TIM1);
	return 0;
}

static int stm32f4xx_pwm_write(struct hal_dev_s *dev, const void *buffer, int buflen,int pos)
{
	return 0;
}


struct hal_dev_s stm32f4xx_pwm_dev;

int stm32f4xx_pwm_register(void)
{
	stm32f4xx_pwm_dev.init  = stm32f4xx_pwm_init;
	stm32f4xx_pwm_dev.open  = stm32f4xx_pwm_open;
	stm32f4xx_pwm_dev.write = stm32f4xx_pwm_write;
	
	hal_dev_register(&stm32f4xx_pwm_dev,"pwm",HAL_O_WRONLY);
	return 0;
}
