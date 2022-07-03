
#ifndef SENSOR_INCLUDE_H
#define SENSOR_INCLUDE_H
#include <stdint.h>
#include <stdbool.h>
#include "exti.h"
#include "adc.h"

typedef void * Sensor;
typedef void (*SensorCallback )( bool uplimit, bool downlimit, bool dispatch_complete);


void SensorInit();
Sensor SensorCreate(SensorCallback callback, AdcChannelId ch1, AdcChannelId ch2, int port_ir, ExtiId exti_ir);


#endif
