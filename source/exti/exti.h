


#ifndef EXTI_INCLUDE_H
#define EXTI_INCLUDE_H

#include <stdint.h>
#include "FreeRTOS.h"

typedef void * ExtiChannel;
typedef void (*ExtiCallback)(BaseType_t * woke_token, void * param);

typedef enum 
{
    EXTI_ID_0 = 0,
    EXTI_ID_1,
    EXTI_ID_2,
    EXTI_ID_3,
    EXTI_ID_4,
    EXTI_ID_5,
    EXTI_ID_6,
    EXTI_ID_7,
    EXTI_ID_8,
    EXTI_ID_9,
    EXTI_ID_10,
    EXTI_ID_11,
    EXTI_ID_12,
    EXTI_ID_13,
    EXTI_ID_14,
    EXTI_ID_15,
    EXTI_ID_16,
    EXTI_ID_17,
    EXTI_ID_18,
    EXTI_ID_19
}ExtiId;

typedef enum 
{
    EXTI_TRIGGARE_RISING,
    EXTI_TRIGGARE_FALLING,
    EXTI_TRIGGARE_RISING_FALLING
}ExtiTriggare;

void ExtiInit();
ExtiChannel ExtiChannelCreate(uint8_t port, ExtiId id, ExtiTriggare trig, ExtiCallback callback, void * userp);
void ExtiEnable(ExtiChannel ch);
void ExtiDisable(ExtiChannel ch);
void ExtiRegisterHandler(ExtiChannel ch, ExtiCallback callback);


#endif