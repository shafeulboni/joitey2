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
    char reply[20];
    WifiCallBack callback;
} WifiManager;

WifiManager wifi_man;


void Sendforaccess(uint32_t cardnumber)
{
	char buf[10] = {0,};
	itoa(cardnumber,buf,10);
	UsartSendString(wifi_man.port, "Card:", 5);
	UsartSendString(wifi_man.port, buf, strlen(buf));
	UsartSendString(wifi_man.port, "\n", 1);

}

void WifiTask(void *param)
{
    uint8_t counter = 0;
    bool frame_start = false;
    uint8_t frame_length = 0;

    while (1)
    {
        uint8_t data_receive;
        if (UsartReceiveByte(wifi_man.port, &data_receive) == 1)
        {
            //DebugPrintf(rms_man.dbg, "%02X\n", data_receive);
            if (data_receive == 0xF9 && frame_start != true)
            {
              //  DebugInfo(rms_man.dbg, "Frame started.\n");
                frame_start = true;
            }
            else if (frame_start == true)
            {
                wifi_man.reply[wifi_man.index] = data_receive;
                uint8_t curr_ind = wifi_man.index;
                wifi_man.index++;
                if (wifi_man.index >= 32)
                {
                	wifi_man.index = 0;
                    frame_start = false;
                    frame_length = 0;
                    memset(wifi_man.reply, 0, 32);
                }

                if (curr_ind == 0)
                {
                    frame_length = data_receive;
                }
                else if (curr_ind == frame_length + 1)
                {
                    //DebugPrintf(rms_man.dbg, "Frame length: %d\n", frame_length);
//                    uint8_t crc = CRC8(rms_man.reply, frame_length  + 1 );
//                    if (crc == data_receive)
//                    {
//                       // DebugInfo(rms_man.dbg, "CRC valid in frame\n");
//                        rms_man.callback((ReceiveData *)(&rms_man.reply[1]));
//                    }
//                    else
//                    {
//                       // DebugInfo(rms_man.dbg, "CRC invalid in frame\n");
//                    }

                    memset(wifi_man.reply, 0, 32);
                    wifi_man.index = 0;
                    frame_start = false;
                    frame_length = 0;
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
    xTaskCreate(WifiTask, "", 512, NULL, 3, NULL);
}
