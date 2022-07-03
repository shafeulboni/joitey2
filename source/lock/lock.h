#ifndef LOCK_INCLUDE_H_
#define LOCK_INCLUDE_H_



#include <stdint.h>
#include <stdbool.h>
#include "adc.h"

//01810150955
typedef void * Lock;

typedef enum
{
    LOCK_ALARM_DISCONNECT,/////ok
    LOCK_ALARM_LOCK_OPEN,   /////ok
}LockAlarmId;

typedef void (*DeviceAlarmCallback)(LockAlarmId id);

void LockInit();
void LockEnable(Lock lck);
void LockDisable(Lock lck);
Lock LockCreate(uint8_t sl_port_1, uint16_t sl_pin_1, uint8_t sl_port_2, uint16_t sl_pin_2, uint8_t m_port_1, uint16_t m_pin_1,uint8_t m_port_2, uint16_t m_pin_2, DeviceAlarmCallback callback);
bool LockState(Lock lck);
void SliderEnablePositive(Lock lck);
void SliderEnableNegative(Lock lck);
void SliderDisable(Lock lck);
void Motor_1_Enable(Lock lck);
void Motor_2_Enable(Lock lck);
void MotorDisable(Lock lck);

#endif // LOCK_INCLUDE_H_
