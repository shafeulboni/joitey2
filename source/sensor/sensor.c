#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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
//#include "led.h"
#include "timers.h"
#include "exti.h"
#include "sensor.h"
#include "adc.h"

typedef struct
{
    SensorCallback callback;
    AdcChannel limit_1;
    AdcChannel limit_2;
    ExtiId exti_irsense;
    int port_ir;
    ExtiChannel channel_ir;

    bool limit_1_sense;
    bool limit_2_sense;
    bool ir_sense;

} SensorReaderPrv;

typedef struct
{
    SensorReaderPrv sensor;
    xSemaphoreHandle sensor_sem;
} SensorManager;

void SensorTask(void *vibtask)
{

    //xTimerStart(sensor_man.sensor.sensor_timer,100);
/*

    while (1)
    {

        ChannelBuffer adata;
        if (AdcChannelRead(sensor_man.sensor.limit_1, &adata) == ADC_STATUS_SUCCESS)
        {
            uint32_t temp_avg = 0;
            for (size_t i = 0; i < ADC_BUFFER_MAX; i++)
            {
                temp_avg = temp_avg + adata.buffer[i];
            }

            uint16_t tempvoltage = temp_avg / ADC_BUFFER_MAX;

            //DebugPrintf(sensor_man.dbg, "Temperature :%d\n", tempvoltage);
            //sensor_man.sensor.temperatureval=tempvoltage;
            if(tempvoltage<100)
            	{
            	if(sensor_man.sensor.limit_1_sense==false)
            		{
            		sensor_man.sensor.limit_1_sense=true;
            		}
            	}
            else
            	{

            	if(sensor_man.sensor.limit_1_sense==true)
            		{
            		sensor_man.sensor.limit_1_sense=false;
            		}
            	}
        }

        ChannelBuffer bdata;
        if (AdcChannelRead(sensor_man.sensor.limit_1, &bdata) == ADC_STATUS_SUCCESS)
        {
            uint32_t temp_avg = 0;
            for (size_t i = 0; i < ADC_BUFFER_MAX; i++)
            {
                temp_avg = temp_avg + bdata.buffer[i];
            }

            uint16_t tempvoltage = temp_avg / ADC_BUFFER_MAX;

            //DebugPrintf(sensor_man.dbg, "Temperature :%d\n", tempvoltage);
            //sensor_man.sensor.temperatureval=tempvoltage;
            if(tempvoltage<100)
            	{
            	if(sensor_man.sensor.limit_2_sense==false)
            		{
            		sensor_man.sensor.limit_2_sense=true;
            		}
            	}
            else
            	{

            	if(sensor_man.sensor.limit_2_sense==true)
            		{
            		sensor_man.sensor.limit_2_sense=false;
            		}
            	}
        }

        if (xSemaphoreTake(sensor_man.sensor_sem, 1000) == pdTRUE)
        {
           
            if (sensor_man.sensor.callback != NULL)
            {
                //DebugPrintf(sensor_man.dbg, "Temperature :%d\n", sensor_man.sensor.temperatureval);
               // sensor_man.sensor.callback(0, true, sensor_man.sensor.temperatureval);
            }
            
        }
        else
        {
        }
    }
    */
}




void SensorTimerHandler(xTimerHandle timer)
{
   // DebugPrintf(sensor_man.dbg, "Timer ok\n");
   // xSemaphoreGive(sensor_man.sensor_sem);
}


void SensorInit()
{
    //sensor_man.dbg = DebugRegister("SENSOR", SensorDebugHandler);
    //reader_man.w26_sem = xSemaphoreCreateBinary();
    xTaskCreate(SensorTask, "Temp", 256, NULL, 2, NULL);
    //sensor_man.sensor.sensor_timer = xTimerCreate("sensortimer", 10000, pdTRUE, NULL, SensorTimerHandler);
    //sensor_man.sensor_sem = xSemaphoreCreateBinary();
}

