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
//#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "usart.h"
#include "w26.h"
#include "ctype.h"
#include "math.h"

#include "lock.h"
//#include "sensor.h"
#include "source/rms/wifi.h"


PinHandle pin;
UsartHandle port;

//void SysTick_Handler(void)
//{
//	extern void DisplayScanner();
//	DisplayScanner();
////	//DMA_config(GetBaseAddress());
//}


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
    //DebugChannel dbg;
    uint32_t card_data;
    Lock Lock;
    xTimerHandle motortimer;
    xTimerHandle slidertimer;
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
    uint8_t data[10];

} SystemCommand;

System sstem;


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
    SystemCommand cmd;
    cmd.cmd = SYSTEM_COMMAND_READER;
    cmd.data[0] = id;
    cmd.sub_cmd = is_connected;
    memcpy(&cmd.data[1], &card_number, 4);
    xQueueSend(sstem.system_queue, &cmd, 1000);
}

void card_operation()
{
	char buf[10] = {0,};
	itoa(sstem.card_data,buf,10);
	UsartSendString(port, "Card Received:", 14);
	UsartSendString(port, buf, strlen(buf));
	UsartSendString(port, "\n", 1);
	Sendforaccess(sstem.card_data);
	sstem.card_data=0;
}

void SysDbgHandler(char *reply, const char **lst, uint16_t len)
{

}
void MainTask(void * param) {
	UsartSendString(port, "Walton Sanitery Dispenser\n",26);
	while (1) {
		SystemCommand cmd;
		TogglePinState(pin);
		//vTaskDelay(100);
		//UsartSendString(port, "Main Task Checking\n",19);
		if (xQueueReceive(sstem.system_queue, &cmd, 1000) == pdTRUE)
		        {
			 switch (cmd.cmd)
			            {
			 	 	 	case SYSTEM_COMMAND_READER:
			 	 	 		{
			 	 	 		uint32_t temp_card = 0;
			 	 	 		memcpy(&temp_card, &cmd.data[1], 4);
							sstem.card_data = temp_card;
							card_operation();
			 	 	 		}
			 	 	 		break;
//			 	 	 	case SYSTEM_COMMAND_wifi:
//							{
//
//							}
//							break;
			 	 	 	default:
			 	 	 	    break;
			            }
		        }

	}
}

void WifiHandler(char *msg, int status, int length)
{
	Disp_integer(port, length);
	UsartSendString(port, msg,length);
	UsartSendString(port, " ",1);
	Disp_integer(port, status);
	UsartSendString(port, "\n",1);

	if(status==1)
	{
		//Motor_1_Enable(sstem.Lock);
		//Motor_2_Enable(sstem.Lock);
		SliderEnablePositive(sstem.Lock);
		xTimerChangePeriod(sstem.slidertimer,10 * 1000, 10);
		//xTimerChangePeriod(sstem.motortimer,10 * 1000, 10);
	}
}

void MotorAlarmHandler(LockAlarmId id)
{

}

void MotorTimerHandler1()
{
	MotorDisable(sstem.Lock);
}

void SliderTimerHandler1()
{
	SliderDisable(sstem.Lock);
}

void Sensorcallbackhandler(bool uplimit, bool downlimit, bool dispatch_complete)
{
	if(dispatch_complete==true){
		MotorDisable(sstem.Lock);
		UsartSendString(port, "Slider Interrupt Called\n",24);
	}
	else if(uplimit==true)
		{
		SliderDisable(sstem.Lock);
		UsartSendString(port, "Up Interrupt Called\n",24);
		Motor_1_Enable(sstem.Lock);
		Motor_2_Enable(sstem.Lock);
		}
	else if(downlimit==true)
			{
			SliderDisable(sstem.Lock);
			UsartSendString(port, "Down Interrupt Called\n",24);
			MotorDisable(sstem.Lock);
			}

	UsartSendString(port, "\n",1);
}


int main(void) {

	SystemCoreClockUpdate();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC  | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

	////usart_lock = xSemaphoreCreateBinary();
	//xSemaphoreGive(usart_lock);
	//wifi_port = InitUsart(COM2, 9600, 0, 700);
	SensorCreate(Sensorcallbackhandler,PORTA,EXTI_ID_0,PORTA,EXTI_ID_1,PORTB,EXTI_ID_3 );
	port = InitUsart(COM1, 115200, 0, 512);
	//gps_port = InitUsart(COM2, 9600, 0, 512);
	pin = InitPin(PORTA, PIN8, OUTPUT);
	//wifi_port = InitUsart(COM2, 9600, 0, 700);
	W26Init();
	W26Create(0, PORTC, PORTC, EXTI_ID_13, EXTI_ID_14, CardHandlerMain);
	WifiInit(WifiHandler);
	sstem.system_queue = xQueueCreate(3, sizeof(SystemCommand));
	xTaskCreate(MainTask, "", 512, NULL, 2, NULL);
	LockInit();
	sstem.Lock=LockCreate(PORTA, PIN4, PORTA, PIN5,PORTA, PIN6, PORTA, PIN7, MotorAlarmHandler);
	sstem.motortimer = xTimerCreate("", 10000, pdFALSE, NULL, MotorTimerHandler1);
	sstem.slidertimer = xTimerCreate("", 10000, pdFALSE, NULL, SliderTimerHandler1);
	//SensorInit();

	vTaskStartScheduler();

	while (1) {
		UsartSendString(port, "Code should not reach here\n",26);
		NVIC_SystemReset();
	}

}

