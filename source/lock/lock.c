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
#include "lock.h"


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

Lock LockCreate(uint8_t sl_port_1, uint16_t sl_pin_1, uint8_t sl_port_2, uint16_t sl_pin_2, uint8_t m_port_1, uint16_t m_pin_1,uint8_t m_port_2, uint16_t m_pin_2, DeviceAlarmCallback callback)
{
    if (lock != NULL)
        return lock;

    lock = pvPortMalloc(sizeof(Device));
    lock->slider_pin_1 = PinCreate(sl_port_1, sl_pin_1,OUTPUT);
    lock->slider_pin_2 = PinCreate(sl_port_2, sl_pin_2, OUTPUT);
    lock->motor_pin_1 = PinCreate(m_port_1, m_pin_1, OUTPUT);
    lock->motor_pin_2 = PinCreate(m_port_2, m_pin_2, OUTPUT);
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
    	        if (xQueueReceive(lock_cmd, &cmd, 1000) == pdTRUE)
    	        {
    	            //DebugPrintf(lock_dbg, "Command id %d\n", cmd);

    	            switch (cmd)
    	            {
    	            case 1:
    	                PinWrite(lock->slider_pin_1, true);
    	                PinWrite(lock->slider_pin_2, false);
    	                break;
    	            case 2:
    	            	PinWrite(lock->slider_pin_1, false);
    	                PinWrite(lock->slider_pin_2, true);
    	                break;
    	            case 3:
    	                PinWrite(lock->slider_pin_1, false);
    	                PinWrite(lock->slider_pin_2, false);
    	                break;
    	            case 4:
    	                PinWrite(lock->motor_pin_1, true);
    	                PinWrite(lock->motor_pin_2, false);
    	                break;
    	            case 5:
    	                PinWrite(lock->motor_pin_1, false);
    	                PinWrite(lock->motor_pin_2, true);
    	                break;
    	            case 6:
						PinWrite(lock->motor_pin_1, false);
						PinWrite(lock->motor_pin_2, false);
						break;
    	            default:
    	                break;
    	            }
    	        }
    }
}




void LockInit()
{
   // lock_dbg = DebugRegister("LOCK", LockDebugHandler);
    lock_cmd = xQueueCreate(10, sizeof(uint8_t));
   // lock_sense_timer = xTimerCreate("", 3000, pdTRUE, NULL, LockTimerHandler);
   // lock_timer = xTimerCreate("", 500, pdTRUE, NULL, LockTimerHandler2);
    xTaskCreate(LockTaskHandler, "Lock", 1024, NULL, 2, NULL);
}

