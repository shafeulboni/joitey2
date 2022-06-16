#include "STM32f10x.h"
#include "STM32f10x_gpio.h"
#include "STM32f10x_rcc.h"
#include "STM32f10x_bkp.h"
#include "STM32f10x_pwr.h"
#include "STM32f10x_rtc.h"
#include "gpio.h"
#include "STM32f10x_usart.h"
#include "STM32f10x_iwdg.h"
#include "STM32f10x_dbgmcu.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "core_cm3.h"
//#include "STM32f10x_adc.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "usart.h"
#include "w26.h"

PinHandle pin;
UsartHandle port;

//void SysTick_Handler(void)
//{
//	extern void DisplayScanner();
//	DisplayScanner();
////	//DMA_config(GetBaseAddress());
//}

#define LED_ACTIVE_HIGH_TIME 	2
#define LED_ACTIVE_LOW_TIME 	2

#define LED_INACTIVE_HIGH_TIME 	2
#define LED_INACTIVE_LOW_TIME 	12

#define BUFFER_SIZE 100

void NVIC_SystemReset(void) {
	__DSB();
	SCB->AIRCR =
			((0x5FA << SCB_AIRCR_VECTKEY_Pos)
					| (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk)
					| SCB_AIRCR_SYSRESETREQ_Msk);
	__DSB();
	while (1)
		;
}


typedef struct
{
    xQueueHandle system_queue;
    DebugChannel dbg;
    uint32_t card_data;
} System;

typedef enum
{
    SYSTEM_COMMAND_READER,
    SYSTEM_COMMAND_wifi,
} SystemCommandType;

typedef struct
{
    struct
    {
        SystemCommandType cmd : 3;
        uint8_t sub_cmd : 5;
    };
    uint8_t data[26];

} SystemCommand;

System sstem;

void Watchdog_Config(uint16_t timeout_ms) {
#ifdef LSI_TIM_MEASURE
	/* Enable the LSI OSC */
	RCC_LSICmd(ENABLE);
	/* Wait till LSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{}
	/* TIM Configuration -------------------------------------------------------*/
	TIM5_ConfigForLSI();
	/* Wait until the TIM5 get 2 LSI edges */
	while(CaptureNumber != 2)
	{}
	/* Disable TIM5 CC4 Interrupt Request */
	TIM_ITConfig(TIM5, TIM_IT_CC4, DISABLE);
#endif
	/* IWDG timeout equal to timeout ms (the timeout may varies due to LSI frequency dispersion) */
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/32 */
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* Set counter reload value to obtain 250ms IWDG TimeOut.
	 Counter Reload Value e.g. = 250ms/IWDG counter clock period
	 = 250ms / (LSI/32)
	 = 0.25s / (LsiFreq/32)
	 = LsiFreq/(32 * 4)
	 = LsiFreq/128
	 So set timeout_ms*LsiFreq/(128*250)=1.25*timeout_ms=timeout_ms*5/4
	 As LsiFreq=40000 Hz
	 */
	IWDG_SetReload(timeout_ms * 5 / 4);

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}

uint8_t Watchdog_Reset_Detect(void) {
	/* Check if the system has resumed from IWDG reset */
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) {
		/* IWDGRST flag set */
		/* Clear reset flags */
		RCC_ClearFlag();
		return 1;
	} else {
		/* IWDGRST flag is not set */
		/* Turn off LED1 */
		return 0;
	}
}

volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr;
volatile uint32_t pc;
volatile uint32_t psr;

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress) {
	/* These are volatile to try and prevent the compiler/linker optimising them
	 away as the variables never actually get used.  If the debugger won't show the
	 values of the variables, make them global my moving their declaration outside
	 of this function. */

	r0 = pulFaultStackAddress[0];
	r1 = pulFaultStackAddress[1];
	r2 = pulFaultStackAddress[2];
	r3 = pulFaultStackAddress[3];

	r12 = pulFaultStackAddress[4];
	lr = pulFaultStackAddress[5];
	pc = pulFaultStackAddress[6];
	psr = pulFaultStackAddress[7];

	/* When the following line is hit, the variables contain the register values. */
	for (;;)
		;
}

uint8_t UintToString(uint16_t in, char * buffer) {
	in = in % 10000;
	uint8_t ret = 0;
	if (in / 1000 != 0) {
		buffer[3] = (uint8_t)(in / 1000) + 48;
		in = in % 1000;
		ret += 1;
	}

	if (in / 100 != 0 || ret != 0) {
		buffer[2] = (uint8_t)(in / 100) + 48;
		in = in % 100;
		ret += 1;
	}

	if (in / 10 != 0 || ret != 0) {
		buffer[1] = (uint8_t)(in / 10) + 48;
		in = in % 10;
		ret += 1;
	}

	buffer[0] = in + 48;
	ret += 1;
	return ret;
}

void HardFault_Handler() {
	__asm volatile
	(
			" tst lr, #4                                                \n"
			" ite eq                                                    \n"
			" mrseq r0, msp                                             \n"
			" mrsne r0, psp                                             \n"
			" ldr r1, [r0, #24]                                         \n"
			" ldr r2, handler2_address_const                            \n"
			" bx r2                                                     \n"
			" handler2_address_const: .word prvGetRegistersFromStack    \n"
	);
}

char CRC8(const char *data, int length) {
	char crc = 0x00;
	char extract;
	char sum;
	for (int i = 0; i < length; i++) {
		extract = *data;
		for (char tempI = 8; tempI; tempI--) {
			sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if (sum)
				crc ^= 0x8C;
			extract >>= 1;
		}
		data++;
	}
	return crc;
}

uint8_t SplitString(char *str, char token, char **string_list) {
	char *head = str;
	char *iterator = str;
	uint8_t count = 0;
	uint8_t field_count = 1;
	for (; *iterator != 0;) {
		if (*iterator == token) {
			*iterator = 0;
			string_list[count++] = head;
			head = (iterator + 1);
			field_count++;
		}
		iterator++;
	}
	string_list[count++] = head;
	return field_count++;
}

void CardHandlerMain(int id, bool is_connected, uint32_t card_number)
{
    DebugPrintf(sstem.dbg, "Card Handler: %d\n", card_number);
    SystemCommand cmd;
    cmd.cmd = SYSTEM_COMMAND_READER;
    cmd.data[0] = id;
    cmd.sub_cmd = is_connected;
    memcpy(&cmd.data[1], &card_number, 4);
    xQueueSend(sstem.system_queue, &cmd, 1000);

}


void GpsTask(void * param) {

	while (1) {
		TogglePinState(pin);
		vTaskDelay(500);
		UsartSendString(port, "HellooooooX\n",12);
	}
}
int main(void) {

	SystemCoreClockUpdate();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC  | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

	//LedSystemInit();
	//dbg_led = LedCreate(LED_SIGNAL_NORMAL, PORTB, PIN8, NULL);
	////usart_lock = xSemaphoreCreateBinary();
	//xSemaphoreGive(usart_lock);
	//gps_port = InitUsart(COM2, 9600, 0, 700);
	//ADC Pin config
	//gpio_config.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	//gpio_config.GPIO_Pin = GPIO_Pin_7;
	//GPIO_Init(GPIOA, &gpio_config);
	//gpio_config.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	//GPIO_Init(GPIOB, &gpio_config);

	port = InitUsart(COM1, 115200, 0, 512);
	//gps_port = InitUsart(COM2, 9600, 0, 512);
	pin = InitPin(PORTA, PIN8, OUTPUT);

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	//ADC_InitTypeDef adc_config;
	//adc_config.ADC_Mode = ADC_Mode_Independent;
	//adc_config.ADC_ContinuousConvMode = DISABLE;
	//adc_config.ADC_NbrOfChannel = 1;
	//adc_config.ADC_DataAlign = ADC_DataAlign_Right;
	//adc_config.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	//adc_config.ADC_ScanConvMode = DISABLE;
	//ADC_Init(ADC1, &adc_config);
	///ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
//	NVIC_EnableIRQ(ADC1_2_IRQn);
	//NVIC_SetPriority(ADC1_2_IRQn, 12);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_239Cycles5);
	//ADC_Cmd(ADC1, ENABLE);
	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	//UsartSendString(port, "HellooooooX\n",12);
	W26Init();
	W26Create(0, PORTC, PORTC, EXTI_ID_13, EXTI_ID_14, CardHandlerMain);
	sstem.dbg = DebugRegister("SYS", SysDbgHandler);
	sstem.system_queue = xQueueCreate(5, sizeof(SystemCommand));

	//xTaskCreate(MainTask, "", 1024, NULL, 1, NULL);
	xTaskCreate(GpsTask, "", 1024, NULL, 2, NULL);
	vTaskStartScheduler();

	while (1) {
		UsartSendString(port, "Code shouldnot reach here\n",26);
		NVIC_SystemReset();
	}

}

