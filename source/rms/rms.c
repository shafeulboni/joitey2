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
#include "led.h"
#include "timers.h"
#include "gpio.h"
#include "adc.h"
#include "usart.h"
#include "wifi.h"

typedef struct
{
    UsartHandle port;
    uint8_t index;
    char reply[30];
    WifiCallBack callback;
    xSemaphoreHandle wifi_sem;
} WifiManager;

WifiManager wifi_man;


void Sendforaccess(uint32_t cardnumber)
{
	char buf[10] = {0,};
	itoa(cardnumber,buf,10);
	UsartSendString(wifi_man.port, "a", 1);
	UsartSendString(wifi_man.port, buf, strlen(buf));
	UsartSendString(wifi_man.port, "k\n", 2);

}

void WifiTask(void *param)
{
    uint8_t counter = 0;
    bool frame_start = false;
    uint8_t frame_length = 0;
    char arr[30];

    UsartSendString(wifi_man.port, "Wifi Manager Checking\n", 22);
    while (1)
    {
        vTaskDelay(10);
        //UsartSendString(wifi_man.port, "Wifi Manager Checking\n", 22);
    	uint8_t data_receive;
    	if (xSemaphoreTake(wifi_man.wifi_sem, 1000) == pdTRUE)
    	        {

    	        wifi_man.callback(wifi_man.reply);
    	        UsartSendString(wifi_man.port, wifi_man.reply, 30);
    	        //frame_start = false;
    	        //if(wifi_man.reply[])
    	        memset(wifi_man.reply, 0, 32);
    	        //vTaskDelay(1000);
    	        }
        if (UsartReceiveByte(wifi_man.port, &data_receive) == 1)
        {

        	//DebugPrintf(rms_man.dbg, "%02X\n", data_receive);
        	UsartSendString(wifi_man.port, (char*)data_receive , 1);

            //if (data_receive == '<' && frame_start != true)
        	if (data_receive == '<')
            {
              //  DebugInfo(rms_man.dbg, "Frame started.\n");
            	//UsartSendString(wifi_man.port, "Frame Started\n", 22);
            	UsartSendString(wifi_man.port, "Frame\n", 6);
                frame_start = true;
                wifi_man.index = 0;
            }
            else if(frame_start==true)
            {

            	wifi_man.reply[wifi_man.index] = data_receive;
            	wifi_man.index++;
            	if((wifi_man.index >= 30)||(data_receive== '>'))
            	{
            	frame_start = false;
            	xSemaphoreGive(wifi_man.wifi_sem);
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
    wifi_man.port = InitUsart(COM2, 115200, 0, 48);
    wifi_man.index = 0;
    wifi_man.callback = callback;
    xTaskCreate(WifiTask, "", 512, NULL, 3, NULL);
    wifi_man.wifi_sem=xSemaphoreCreateBinary();
}
