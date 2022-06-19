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
#include "debug.h"
#include "semphr.h"
#include "led.h"
#include "timers.h"
#include "gpio.h"
#include "adc.h"
#include "usart.h"
#include "wifi.h"

typedef struct
{
    DebugChannel dbg;
    UsartHandle port;
    char reply[32];
    uint8_t index;
    RmsCallBack callback;
    Pin rs485_dir;
} RmsManager;

RmsManager rms_man;

char CRC8(const char *data, int length);


char CRC8(const char *data, int length)
{
    char crc = 0x00;
    char extract;
    char sum;
    for (int i = 0; i < length; i++)
    {
        extract = *data;
        for (char tempI = 8; tempI; tempI--)
        {
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



void RmsTask(void *param)
{
    uint8_t counter = 0;
    bool frame_start = false;
    uint8_t frame_length = 0;

    DebugInfo(rms_man.dbg, "RMS Task started.\n");
    //PinWrite(rms_man.rs485_dir, false);
    while (1)
    {
        uint8_t data_receive;
        if (UsartReceiveByte(rms_man.port, &data_receive) == 1)
        {
            //DebugPrintf(rms_man.dbg, "%02X\n", data_receive);
            if (data_receive == 0xF9 && frame_start != true)
            {
                DebugInfo(rms_man.dbg, "Frame started.\n");
                frame_start = true;
            }
            else if (frame_start == true)
            {
                rms_man.reply[rms_man.index] = data_receive;
                uint8_t curr_ind = rms_man.index;
                rms_man.index++;
                if (rms_man.index >= 32)
                {
                    rms_man.index = 0;
                    frame_start = false;
                    frame_length = 0;
                    memset(rms_man.reply, 0, 32);
                }

                if (curr_ind == 0)
                {
                    frame_length = data_receive;
                }       
                else if (curr_ind == frame_length + 1)
                {
                    DebugPrintf(rms_man.dbg, "Frame length: %d\n", frame_length);
                    uint8_t crc = CRC8(rms_man.reply, frame_length  + 1 );
                    if (crc == data_receive)
                    {
                        DebugInfo(rms_man.dbg, "CRC valid in frame\n");
                        rms_man.callback((ReceiveData *)(&rms_man.reply[1]));
                    }
                    else
                    {
                        DebugInfo(rms_man.dbg, "CRC invalid in frame\n");
                    }

                    memset(rms_man.reply, 0, 32);
                    rms_man.index = 0;
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

void RmsInit(RmsCallBack callback)
{
    rms_man.port = InitUsart(COM2, 9600, 0, 48);
    rms_man.index = 0;
    rms_man.callback = callback;
    rms_man.dbg = DebugRegister("RMS", RmsDebugHandler);
    //rms_man.rs485_dir = PinCreate(PORTA, PIN_14, MODE_OUTPUT);
    xTaskCreate(RmsTask, "", 512, NULL, 3, NULL);
}
