

#ifndef LED_INCLUDE_H_
#define LED_INCLUDE_H_

#include <stdint.h>

#define LED_ALWAYES_ON_DURATION     0xFFFF

typedef void *Led;


typedef void (*LedTurnedOffCallback)();

typedef enum
{
    LED_SIGNAL_NORMAL,
    LED_SIGNAL_INVERTED
} LedSignalType;

void LedSystemInit();
Led LedCreate(LedSignalType type, uint8_t port, uint16_t pin, LedTurnedOffCallback callback);
void LedDelete(Led led);

void LedOn(Led led);
void LedOff(Led led);

void LedSchedule(Led led, uint16_t init_delay, uint16_t high, uint16_t low, uint16_t duration);

#endif // LED_INCLUDE_H_