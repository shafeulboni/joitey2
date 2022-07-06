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
//#include "led.h"
#include "timers.h"
#include "gpio.h"
#include "adc.h"
#include "lock.h"
#include "exti.h"

typedef struct
{
	PinHandle slider_pin_1;
	PinHandle slider_pin_2;
	PinHandle motor_pin_1;
	PinHandle motor_pin_2;

    LockAlarmId lock_central;
    bool state;
    DeviceAlarmCallback callback;

} Device;

Device *lock = NULL;
xTimerHandle lock_timer;
xQueueHandle lock_cmd;


typedef struct
{
    SensorCallback callback;

    ExtiId exti_upsense;
    ExtiId exti_downsense;
    ExtiId exti_irsense;
    int port_ir;
    int port_up;
    int port_down;
    ExtiChannel channel_up;
    ExtiChannel channel_down;
    ExtiChannel channel_ir;

    bool limit_1_sense;
    bool limit_2_sense;
    bool ir_sense;
    int voltage;

} SensorReaderPrv;

typedef struct
{
    SensorReaderPrv sensor;
    xSemaphoreHandle sensor_sem;
} SensorManager;

SensorManager sensor_man;

Lock LockCreate(uint8_t sl_port_1, uint16_t sl_pin_1, uint8_t sl_port_2, uint16_t sl_pin_2, uint8_t m_port_1, uint16_t m_pin_1,uint8_t m_port_2, uint16_t m_pin_2, DeviceAlarmCallback callback)
{
    if (lock != NULL)
        return lock;

    lock = pvPortMalloc(sizeof(Device));
    lock->slider_pin_1 = InitPin(sl_port_1, sl_pin_1,OUTPUT);
    lock->slider_pin_2 = InitPin(sl_port_2, sl_pin_2, OUTPUT);
    lock->motor_pin_1 = InitPin(m_port_1, m_pin_1, OUTPUT);
    lock->motor_pin_2 = InitPin(m_port_2, m_pin_2, OUTPUT);
    lock->callback = callback;
    lock->state = false;
    return lock;
}

void SliderEnablePositive(Lock lck)
{
    uint8_t cmd = 1;
    xQueueSend(lock_cmd, &cmd, 100);
}
void SliderEnableNegative(Lock lck)
{
    uint8_t cmd = 2;
    xQueueSend(lock_cmd, &cmd, 100);
}

void SliderDisable(Lock lck)
{
    uint8_t cmd = 3;
   xQueueSend(lock_cmd, &cmd, 100);
}

void Motor_1_Enable(Lock lck)
{
	uint8_t cmd = 4;
	xQueueSend(lock_cmd, &cmd, 100);
}
void Motor_2_Enable(Lock lck)
{
	uint8_t cmd = 5;
	xQueueSend(lock_cmd, &cmd, 100);
}

void MotorDisable(Lock lck)
{
	uint8_t cmd = 6;
	xQueueSend(lock_cmd, &cmd, 100);
}



void LockTaskHandler(void *param)
{

	while (1)
    {
    	uint8_t cmd;
    	//adc_function();
    	        if (xQueueReceive(lock_cmd, &cmd, 1000) == pdTRUE)
    	        {
    	            //DebugPrintf(lock_dbg, "Command id %d\n", cmd);

    	            switch (cmd)
    	            {
    	            case 1:
    	            	SetPinState(lock->slider_pin_1, true);
    	            	SetPinState(lock->slider_pin_2, false);
    	                break;
    	            case 2:
    	            	SetPinState(lock->slider_pin_1, false);
    	            	SetPinState(lock->slider_pin_2, true);
    	                break;
    	            case 3:
    	            	SetPinState(lock->slider_pin_1, false);
    	            	SetPinState(lock->slider_pin_2, false);
    	            	//sensor_man.sensor.callback(0,0,1,sensor_man.sensor.voltage);
    	                break;
    	            case 4:
    	            	SetPinState(lock->motor_pin_1, true);
    	            	SetPinState(lock->motor_pin_2, false);
    	                break;
    	            case 5:
    	            	SetPinState(lock->motor_pin_1, false);
    	            	SetPinState(lock->motor_pin_2, true);
    	                break;
    	            case 6:
    	            	SetPinState(lock->motor_pin_1, false);
    	            	SetPinState(lock->motor_pin_2, false);

						break;
    	            default:
    	                break;
    	            }
    	        }
    }
}


void ExtiHandlerIR(BaseType_t *woke_token, void *param)
{
	sensor_man.sensor.callback(0,0,1);
}

void ExtiHandlerUP(BaseType_t *woke_token, void *param)
{
	sensor_man.sensor.callback(1,0,0);
}

void ExtiHandlerDOWN(BaseType_t *woke_token, void *param)
{
	sensor_man.sensor.callback(0,1,0);
}




void LockInit()
{
   // lock_dbg = DebugRegister("LOCK", LockDebugHandler);
   lock_cmd = xQueueCreate(3, sizeof(uint8_t));
   // lock_sense_timer = xTimerCreate("", 3000, pdTRUE, NULL, LockTimerHandler);
   // lock_timer = xTimerCreate("", 500, pdTRUE, NULL, LockTimerHandler2);
    xTaskCreate(LockTaskHandler, "Lock", 512, NULL, 2, NULL);
}

Sensor SensorCreate(SensorCallback callback, int portup,ExtiId exti_up,int portdown, ExtiId exti_down, int port_ir, ExtiId exti_ir)
{
	sensor_man.sensor.callback = callback;
	sensor_man.sensor.port_up=portup;
	    sensor_man.sensor.exti_upsense=exti_up;
	    sensor_man.sensor.port_down=portdown;
	        sensor_man.sensor.exti_downsense=exti_down;
    sensor_man.sensor.port_ir=port_ir;
    sensor_man.sensor.exti_irsense=exti_ir;
    sensor_man.sensor.channel_up=ExtiChannelCreate(portup,exti_up,EXTI_TRIGGARE_FALLING, ExtiHandlerUP,0);
    sensor_man.sensor.channel_down=ExtiChannelCreate(portdown,exti_down,EXTI_TRIGGARE_FALLING, ExtiHandlerDOWN,0);
    sensor_man.sensor.channel_ir=ExtiChannelCreate(port_ir,exti_ir,EXTI_TRIGGARE_FALLING, ExtiHandlerIR,0);
    ExtiEnable(sensor_man.sensor.channel_ir);
    return &sensor_man.sensor;
}

