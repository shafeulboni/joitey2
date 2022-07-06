#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "STM32f10x_exti.h"
#include "STM32f10x_tim.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
//#include "led.h"
#include "timers.h"
#include "gpio.h"
#include "adc.h"
#include "usart.h"
#include "wifi.h"

typedef struct
{
    UsartHandle port;
    uint8_t index;
    char reply[15];
    WifiCallBack callback;
   // xSemaphoreHandle wifi_sem;
} WifiManager;

WifiManager wifi_man;


void Sendforaccess(uint32_t cardnumber)
{
	char buf[10] = {0,};
	itoa(cardnumber,buf,10);
	UsartSendString(wifi_man.port, "#", 1);
	UsartSendString(wifi_man.port, buf, strlen(buf));
	UsartSendString(wifi_man.port, "k\n", 2);

}



int Stringcheck(char *param)
{

	if(strcmp(param, "Success>")==0)
	{
		//UsartSendString(wifi_man.port, "Alhamdullillah\n", 15);
		//wifi_man.callback(1);
		return 1;
	}
	else
	{
		//UsartSendString(wifi_man.port, "Try Next time\n", 14);
		//wifi_man.callback(2);
		return 2;
	}
}


void WifiTask(void *param)
{
    //uint8_t counter = 0;
    bool frame_start = false;
   // uint8_t frame_length = 0;
    char arr[30];
    bool message=false;

    UsartSendString(wifi_man.port, "Wifi Manager Checking\n", 22);
    while (1)
    {
    	//frame_start = false;
    	vTaskDelay(5);
    	if (message == true)
			{
    		wifi_man.callback(arr,Stringcheck(arr),wifi_man.index);

			//UsartSendString(wifi_man.port, "Message ok\n", 11);
    		//UsartSendString(wifi_man.port, arr, wifi_man.index);
    		//Stringcheck(arr, wifi_man.index);
			frame_start = false;
			//if(wifi_man.reply[])
			memset(arr, 0, 30);
			//vTaskDelay(1000);
			message=false;
			wifi_man.index=0;
			}
    	uint8_t data_receive;
        if (UsartReceiveByte(wifi_man.port, &data_receive) == 1)
      	   {
        	//UsartSendString(wifi_man.port, data_receive,1 );
        	//Disp_integer(data_receive);
        	//UsartSendByte(wifi_man.port,data_receive);

        	if (data_receive == 0X3C && frame_start != true)
        	            {
        					//UsartSendString(wifi_man.port, "Frame started.\n",15);
        	                frame_start = true;
        	                wifi_man.index=0;
        	            }
        	else if(frame_start==true)
        	{
        		arr[wifi_man.index]=data_receive;
        		wifi_man.index++;
        		//UsartSendByte(wifi_man.port,data_receive);
        		if((wifi_man.index >= 15)||(data_receive== 0X3E))
					{
					frame_start = false;
					//xSemaphoreGive(wifi_man.wifi_sem);
					//UsartSendString(wifi_man.port, "Frame End.\n",15);
					//Disp_integer(wifi_man.index);
					//UsartSendString(wifi_man.port, " ",1);
        			message=true;
					}
        	}
      	   }

    }
}

void RmsDebugHandler(char *reply, const char **lst, uint16_t len)
{

}

void WifiInit(WifiCallBack callback)
{
    wifi_man.port = InitUsart(COM2, 9600, 0, 48);
    wifi_man.index = 0;
    wifi_man.callback = callback;
    xTaskCreate(WifiTask, "", 256, NULL, 3, NULL);
    //wifi_man.wifi_sem=xSemaphoreCreateBinary();
}
