#include <string.h>
#include "STM32f10x.h"
#include "STM32f10x_gpio.h"
#include "STM32f10x_rcc.h"
#include "STM32f10x_dma.h"
#include "STM32f10x_tim.h"
#include "dma.h"
#include "gpio.h"

TIM_OCInitTypeDef  TIM_OCInitStructure;
//uint16_t SRC_Buffer[3] = {0, 0, 0};
uint16_t TimerPeriod = 0, TimerPeriod1;
//uint8_t vb[514] = "01021103434E5311074C494D49544544";

void InitDma()
{

	TIM_TimeBaseInitTypeDef config_timer;

	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);



	  /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
	  TimerPeriod = 179;
	  /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
	  uint16_t  Channel1Pulse = (uint16_t) (((uint32_t) 50 * (TimerPeriod - 1)) / 100);


	config_timer.TIM_Prescaler = 1;
	config_timer.TIM_CounterMode = TIM_CounterMode_Up;
	config_timer.TIM_Period = TimerPeriod;
	config_timer.TIM_ClockDivision = TIM_CKD_DIV1;
	config_timer.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3,&config_timer);



	TimerPeriod1 = 47999;
	config_timer.TIM_Prescaler = 2;
	config_timer.TIM_Period = TimerPeriod1;
	config_timer.TIM_ClockDivision = TIM_CKD_DIV1;
	config_timer.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4,&config_timer);



	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	//TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;


	TIM_OC3Init(TIM3, &TIM_OCInitStructure);


	Channel1Pulse = (uint16_t) (((uint32_t) 40 * (TimerPeriod1 - 1)) / 100);


	//TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	//TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);

	/* 7 (Enable TIM for DMA events) */
	TIM_DMACmd(TIM3, TIM_DMA_CC3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM4, ENABLE);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	NVIC_SetPriority(TIM4_IRQn, 12);

	NVIC_EnableIRQ(TIM4_IRQn);

	TIM_Cmd(TIM4, ENABLE);
}


void DMA_config(uint32_t mem_address)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel2);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPIOA->ODR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)mem_address;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 257;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel6, &DMA_InitStructure);
	DMA_ClearITPendingBit(DMA1_IT_TC2);
	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
	  /* DMA1 Channel2 enable */
	DMA_Cmd(DMA1_Channel2, ENABLE);

	NVIC_SetPriority(DMA1_Channel2_IRQn, 13);

	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	TIM_Cmd(TIM3, ENABLE);
}




void DMA1_Channel2_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA1_IT_TC2);
	//DMA_config((uint32_t)vb);
	TIM_Cmd(TIM3, DISABLE);
	DMA_Cmd(DMA1_Channel2, DISABLE);
}


void TIM4_IRQHandler()
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		extern void DisplayScanner();
		DisplayScanner();

		extern uint32_t GetBaseAddress();
		DMA_config((uint32_t)GetBaseAddress());
	}
}
