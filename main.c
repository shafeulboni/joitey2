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
    bool SliderPositive;
    bool SliderNegaitive;
    bool Motor1;
    bool Motor2;
    bool Op_start;

} System;

typedef enum
{
    SYSTEM_COMMAND_READER,
    SYSTEM_COMMAND_wifi,
    SYSTEM_COMMAND_INTERRUPT,
} SystemCommandType;

typedef struct
{
    struct
    {
        SystemCommandType cmd : 3;
       // uint8_t sub_cmd : 3;
    };
    uint8_t data[5];

} SystemCommand;

System sstem;


void CardHandlerMain(int id, bool is_connected, uint32_t card_number)
{
    SystemCommand cmd;
    cmd.cmd = SYSTEM_COMMAND_READER;
    cmd.data[0] = id;
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

	SliderEnableNegative(sstem.Lock);///ok
	xTimerChangePeriod(sstem.slidertimer,10 * 1000, 10);
	sstem.Op_start=true;
}

void start_motor()
{
	sstem.Motor1=true;
	Motor_1_Enable(sstem.Lock);
	xTimerChangePeriod(sstem.motortimer,10 * 1000, 10);
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
			 	 	 	case SYSTEM_COMMAND_INTERRUPT:
							{
								uint8_t	temp=0;

								memcpy(&temp, &cmd.data[1], 1);
								//Disp_integer(port, cmd.data[0]);
								//UsartSendString(port, "\n",1);
								//Disp_integer(port, cmd.data[1]);
								//UsartSendString(port, "\n",1);
								if(temp==1)
									{
									UsartSendString(port, "Motor Interrupt Called\n",24);
									MotorDisable(sstem.Lock);
									sstem.Motor1=false;
									sstem.Motor2=false;
									if(sstem.Op_start==true){
									SliderEnablePositive(sstem.Lock);///ok
									xTimerChangePeriod(sstem.slidertimer,10 * 1000, 10);
									}
									}
								else if(temp==2)
									{
									UsartSendString(port, "Up Interrupt Called\n",24);
									SliderDisable(sstem.Lock);
									if(sstem.Op_start==true){start_motor();}
									}
								else if(temp==3)
									{
									UsartSendString(port, "Down Interrupt Called\n",24);
									SliderDisable(sstem.Lock);
									sstem.Op_start=false;
									}
							}
							break;
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
		SliderEnableNegative(sstem.Lock);//ok
		xTimerChangePeriod(sstem.slidertimer,10 * 1000, 10);
		sstem.Op_start=true;
		//xTimerChangePeriod(sstem.motortimer,10 * 1000, 10);
	}
}

void MotorAlarmHandler(LockAlarmId id)
{

}

void MotorTimerHandler1()
{
	//MotorDisable(sstem.Lock);
	if(sstem.Motor1==true)
	{
		MotorDisable(sstem.Lock);
		sstem.Motor1=false;
		sstem.Motor2=true;
		Motor_2_Enable(sstem.Lock);
		xTimerChangePeriod(sstem.motortimer,10 * 1000, 10);
	}
	else if(sstem.Motor2==true){
		MotorDisable(sstem.Lock);
		sstem.Motor2=false;
		SliderEnablePositive(sstem.Lock);//ok
		xTimerChangePeriod(sstem.slidertimer,10 * 1000, 10);
	}
}

void SliderTimerHandler1()
{
	SliderDisable(sstem.Lock);
}

void Sensorcallbackhandler(int id, bool uplimit, bool downlimit, bool dispatch_complete)
{

	SystemCommand cmd;
	cmd.cmd = SYSTEM_COMMAND_INTERRUPT;
	cmd.data[0] = id;
	int val;
		if(dispatch_complete==true){
			val=1;
			memcpy(&cmd.data[1], &val, 1);
		}
		else if(uplimit==true)
			{
			val=2;
			memcpy(&cmd.data[1], &val, 1);
			}
		else if(downlimit==true)
				{
			val=3;
			memcpy(&cmd.data[1], &val, 1);
				}
	xQueueSend(sstem.system_queue, &cmd, 1000);
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

