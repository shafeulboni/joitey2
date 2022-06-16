//#include "vibration.h"
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
#include "led.h"
#include "timers.h"
#include "exti.h"
#include "w26.h"
#include "adc.h"
#include <stdbool.h>

typedef struct
{
    int id;
    ExtiId exti_d0;
    ExtiId exti_d1;
    int port_d0;
    int port_d1;
    ExtiChannel channel_d0;
    ExtiChannel channel_d1;
    xTimerHandle timer;
    W26ReaderCallback callback;

    uint32_t cardTempHigh;
    uint32_t cardTemp;
    int16_t bitCount;
    uint32_t code;
    int16_t wiegandType;

    bool card_connected;

} W26ReaderPrv;

typedef struct
{
    W26ReaderPrv reader_list[2];
    DebugChannel dbg;
    xSemaphoreHandle w26_sem;
} W26ReaderManager;

W26ReaderManager reader_man;

uint32_t GetCardId(volatile uint32_t *codehigh, volatile uint32_t *codelow, uint8_t bitlength)
{

    if (bitlength == 26) // EM tag
    {
        return (*codelow & 0x1FFFFFE) >> 1;
    }

    if (bitlength == 34) // Mifare
    {
        *codehigh = *codehigh & 0x03; // only need the 2 LSB of the codehigh
        *codehigh <<= 30;             // shift 2 LSB to MSB
        *codelow >>= 1;
        return *codehigh | *codelow;
    }

    return *codelow; // EM tag or Mifare without parity bits
}

uint8_t translateEnterEscapeKeyPress(uint8_t originalKeyPress)
{
    switch (originalKeyPress)
    {
    case 0x0b:       // 11 or * key
        return 0x0d; // 13 or ASCII ENTER

    case 0x0a:       // 10 or # key
        return 0x1b; // 27 or ASCII ESCAPE

    default:
        return originalKeyPress;
    }
    return originalKeyPress;
}

uint32_t getCode(W26ReaderPrv *wtype)
{
    return wtype->code;
}

int16_t getWiegandType(W26ReaderPrv *wtype)
{
    return wtype->wiegandType;
}

uint8_t wig_available(W26ReaderPrv *cardread)
{
    uint32_t cardID;
    //uint32_t time_wig = HAL_GetTick();

    //if((time_wig - lastWiegand) > 25) // if no more signal coming through after 25ms
    //{
    if ((cardread->bitCount == 24) || (cardread->bitCount == 26) || (cardread->bitCount == 32) || (cardread->bitCount == 34) || (cardread->bitCount == 8) || (cardread->bitCount == 4)) // bitCount for keypress=4 or 8, Wiegand 26=24 or 26, Wiegand 34=32 or 34
    {
        cardread->cardTemp >>= 1; // shift right 1 bit to get back the real value - interrupt done 1 left shift in advance

        if (cardread->bitCount > 32) // bit count more than 32 bits, shift high bits right to make adjustment
        {
            cardread->cardTempHigh >>= 1;
        }

        if (cardread->bitCount == 8) // keypress wiegand with integrity
        {
            // 8-bit Wiegand keyboard data, high nibble is the "NOT" of low nibble
            // eg if key 1 pressed, data=E1 in binary 11100001 , high nibble=1110 , low nibble = 0001
            uint8_t highNibble = (cardread->cardTemp & 0xf0) >> 4;
            uint8_t lowNibble = (cardread->cardTemp & 0x0f);
            cardread->wiegandType = cardread->bitCount;
            cardread->bitCount = 0;
            cardread->cardTemp = 0;
            cardread->cardTempHigh = 0;

            if (lowNibble == (~highNibble & 0x0f)) // check if low nibble matches the "NOT" of high nibble.
            {
                cardread->code = (int16_t)translateEnterEscapeKeyPress(lowNibble);
                return 1;
            }
            else
            {
                //lastWiegand = time_wig;
                cardread->bitCount = 0;
                cardread->cardTemp = 0;
                cardread->cardTempHigh = 0;
                return 0;
            }

            // TODO: Handle validation failure case!
        }
        else if (4 == cardread->bitCount)
        {
            // 4-bit Wiegand codes have no data integrity check so we just
            // read the LOW nibble.
            cardread->code = (int16_t)translateEnterEscapeKeyPress(cardread->cardTemp & 0x0000000F);

            cardread->wiegandType = cardread->bitCount;
            cardread->bitCount = 0;
            cardread->cardTemp = 0;
            cardread->cardTempHigh = 0;

            return 1;
        }
        else // wiegand 26 or wiegand 34
        {
            cardID = GetCardId(&cardread->cardTempHigh, &cardread->cardTemp, cardread->bitCount);
            cardread->wiegandType = cardread->bitCount;
            cardread->bitCount = 0;
            cardread->cardTemp = 0;
            cardread->cardTempHigh = 0;
            cardread->code = cardID;
            return 1;
        }
    }
    else
    {
        // well time over 25 ms and bitCount !=8 , !=26, !=34 , must be noise or nothing then.
        //lastWiegand = time_wig;
        cardread->bitCount = 0;
        cardread->cardTemp = 0;
        cardread->cardTempHigh = 0;
        return 0;
    }
    //}
    // else
    // {
    // 	return 0;
    // }
}

void CardReaderTask(void *vibtask)
{
    reader_man.reader_list[0].card_connected=true;

    while (1)
    {
        if (xSemaphoreTake(reader_man.w26_sem, 1000) == pdTRUE)
        {
            if (wig_available(&reader_man.reader_list[0]))
            {
                 //DebugPrintf(reader_man.dbg, "DEC=%d, Protokol Wiegand-%d\n",  reader_man.reader_list[0].code, reader_man.reader_list[0].wiegandType);
                if (reader_man.reader_list[0].callback != NULL)
                {
                    reader_man.reader_list[0].callback(0, true, reader_man.reader_list[0].code);
                }
            }
        }
        else
        {
        }
    }
}

void W26DebugHandler(char *reply, const char **list, uint16_t len)
{
}

void W26Init()
{
    reader_man.dbg = DebugRegister("W26", W26DebugHandler);
    reader_man.w26_sem = xSemaphoreCreateBinary();
    xTaskCreate(CardReaderTask, "Card", 512, NULL, 2, NULL);
}

void ExtiHandlerD0(BaseType_t *woke_token, void *param)
{
    W26ReaderPrv *w26read = param;

    w26read->bitCount++; // Increament bit count for Interrupt connected to D0

    if (w26read->bitCount > 31) // If bit count more than 31, process high bits
    {
        w26read->cardTempHigh |= ((0x80000000 & w26read->cardTemp) >> 31); //	shift value to high bits
        w26read->cardTempHigh <<= 1;
        w26read->cardTemp <<= 1;
    }
    else
    {
        w26read->cardTemp <<= 1; // D0 represent binary 0, so just left shift card data
    }
    xTimerResetFromISR(w26read->timer, woke_token);
}

void ExtiHandlerD1(BaseType_t *woke_token, void *param)
{
    W26ReaderPrv *w26read = param;
    w26read->bitCount++; // Increment bit count for Interrupt connected to D1

    if (w26read->bitCount > 31) // If bit count more than 31, process high bits
    {
        w26read->cardTempHigh |= ((0x80000000 & w26read->cardTemp) >> 31); // shift value to high bits
        w26read->cardTempHigh <<= 1;
        w26read->cardTemp |= 1;
        w26read->cardTemp <<= 1;
    }
    else
    {
        w26read->cardTemp |= 1;  // D1 represent binary 1, so OR card data with 1 then
        w26read->cardTemp <<= 1; // left shift card data
    }
    xTimerResetFromISR(w26read->timer, woke_token);
}

void CardTimerHandler(xTimerHandle timer)
{
    xSemaphoreGive(reader_man.w26_sem);
}

W26Reader W26Create(int reader_id, int port_d0, int port_d1, ExtiId exti_d0, ExtiId exti_d1, W26ReaderCallback callback)
{
    if (reader_id > 1)
        return NULL;
    reader_man.reader_list[reader_id].id = reader_id;
    reader_man.reader_list[reader_id].port_d0 = port_d0;
    reader_man.reader_list[reader_id].port_d1 = port_d1;
    reader_man.reader_list[reader_id].exti_d0 = exti_d0;
    reader_man.reader_list[reader_id].exti_d1 = exti_d1;
    reader_man.reader_list[reader_id].cardTempHigh = 0;
    reader_man.reader_list[reader_id].cardTemp = 0;
    reader_man.reader_list[reader_id].bitCount = 0;
    reader_man.reader_list[reader_id].code = 0;
    reader_man.reader_list[reader_id].wiegandType = 0;
    reader_man.reader_list[reader_id].callback = callback;
    reader_man.reader_list[reader_id].channel_d0 = ExtiChannelCreate(port_d0, exti_d0, EXTI_TRIGGARE_FALLING, ExtiHandlerD0, &reader_man.reader_list[reader_id]);
    reader_man.reader_list[reader_id].channel_d1 = ExtiChannelCreate(port_d1, exti_d1, EXTI_TRIGGARE_FALLING, ExtiHandlerD1, &reader_man.reader_list[reader_id]);
    reader_man.reader_list[reader_id].timer = xTimerCreate("cardtimer", 30, pdTRUE, &reader_man.reader_list[reader_id], CardTimerHandler);
    ExtiEnable(reader_man.reader_list[reader_id].channel_d0);
    ExtiEnable(reader_man.reader_list[reader_id].channel_d1);

    return &reader_man.reader_list[reader_id];
}
