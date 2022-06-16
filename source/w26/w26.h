
#ifndef W26_INCLUDE_H
#define W26_INCLUDE_H
#include <stdint.h>
#include <stdbool.h>
#include "exti.h"
#include "adc.h"

typedef void * W26Reader;
typedef void (*W26ReaderCallback )(int id, bool is_connected,  uint32_t card_number);

typedef enum
{
    CARD_ALARM_DISCONNECT,/////ok
    CARD_ALARM_CARD_OPEN,   /////ok
}CardAlarmId;

void W26Init();
W26Reader W26Create(int reader_id, int port_d0, int port_d1, ExtiId exti_d0, ExtiId exti_d1,W26ReaderCallback callback, AdcChannelId ch);


#endif