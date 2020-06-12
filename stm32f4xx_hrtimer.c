#include "hal_device.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_bus.h"
#include "hrtimer.h"

unsigned long long hrt_global_time;

void hrtimer_init(void)
{	
	LL_TIM_InitTypeDef TIM_InitStruct = {0};

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

	NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(TIM4_IRQn);

	TIM_InitStruct.Prescaler = 83;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 49999;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM4, &TIM_InitStruct);
	LL_TIM_EnableARRPreload(TIM4);
	LL_TIM_EnableIT_UPDATE(TIM4);
	LL_TIM_DisableMasterSlaveMode(TIM4);
	LL_TIM_EnableCounter(TIM4);
}

float hrtimer_get_time(void)
{
    return 1e-6f * (float)(hrt_global_time + TIM4->CNT);      
}

float hrtimer_elapse_time(float start)
{
    return 1e-6f * (float)(hrt_global_time + TIM4->CNT) - start;    
}

void mdelay(uint32_t ms)
{
    volatile int i,j;
    for(i = 0;i < ms;i++){
        for(j = 0;j < 5000;j++);
    }
}

void TIM4_IRQHandler(void)
{
	
	if(LL_TIM_IsActiveFlag_UPDATE(TIM4)){
        LL_TIM_ClearFlag_UPDATE(TIM4); 
		hrt_global_time += 49999;
    } 
}
