
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "led.h"
#include "gpio.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

typedef enum
{
    LED_ON,
    LED_OFF,
    LED_BLINK
} LedStateHint;

typedef struct _led
{
    LedSignalType type;
    uint8_t high;
    uint8_t low;
    uint8_t initial_delay;
    uint8_t current_state;
    uint8_t next_event;
    uint16_t duration;
    LedStateHint hint;
    LedTurnedOffCallback callback;

    PinHandle pin;
    uint8_t is_configured;
    struct _led *pNext;
} Led_t;

typedef struct
{
    Led_t *pHead;
    TaskHandle_t task_handle;

} LedManager;

void LedTask(void *param);

static LedManager ld_man;

void LedSystemInit()
{
    ld_man.pHead = NULL;
    xTaskCreate(LedTask, "led task", 1024, NULL, 4, &ld_man.task_handle);
}

Led LedCreate(LedSignalType type, uint8_t port, uint16_t pin, LedTurnedOffCallback callback)
{
    Led_t *ld = pvPortMalloc(sizeof(Led_t));
    if (ld == NULL)
        return ld;
    ld->type = type;
    ld->high = 0;
    ld->low = 0;
    ld->initial_delay = 0;
    ld->current_state = 0;
    ld->next_event = 0;
    ld->pin = InitPin(port, pin, OUTPUT);
    ld->pNext = ld_man.pHead;
    ld_man.pHead = ld;
    ld->is_configured = 1;
    ld->hint = LED_OFF;
    ld->duration = 0;
    ld->callback = callback;
    return ld;
}

void LedDelete(Led led)
{
    vPortFree(led);
}

void LedOn(Led led)
{
    Led_t *ld = (Led_t *)led;
    ld->hint = LED_ON;
}
void LedOff(Led led)
{
    Led_t *ld = (Led_t *)led;
    ld->hint = LED_OFF;
}

void LedSchedule(Led led, uint16_t init_delay, uint16_t high, uint16_t low, uint16_t duration)
{
    Led_t *ld = (Led_t *)led;
    ld->initial_delay = init_delay;
    ld->high = high;
    ld->low = low;
    ld->hint = LED_BLINK;
    ld->duration = duration;
}

void LedTask(void *param)
{
    TickType_t prv_time;
    while (1)
    {
        Led_t *node = ld_man.pHead;
        while (node != NULL)
        {
            if (node->hint == LED_OFF)
            {
            	SetPinState(node->pin, LOW);
            }
            else if (node->hint == LED_ON)
            {
            	SetPinState(node->pin, HIGH);
            }
            else
            {
                if (node->initial_delay == 0)
                {
                    if (node->next_event == 0)
                    {
                        if (node->current_state == 0)
                        {
                        	SetPinState(node->pin, HIGH);
                            node->current_state = 1;
                            node->next_event = node->high;
                            if(node->duration != LED_ALWAYES_ON_DURATION)
                            {
                                if(node->duration == 0)
                                {
                                    node->hint = LED_OFF;
                                    if(node->callback != NULL)
                                    {
                                        node->callback();
                                    }
                                }
                                node->duration--;
                            }
                        }
                        else
                        {
                        	SetPinState(node->pin, LOW);
                            node->current_state = 0;
                            node->next_event = node->low;
                        }
                    }
                    else
                    {
                        node->next_event--;
                    }
                }
                else
                {
                    node->initial_delay--;
                }
            }

            node = node->pNext;
        }
        vTaskDelayUntil(&prv_time, 50);
    }
}
