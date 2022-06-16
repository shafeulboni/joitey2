
#include <stdint.h>
#include <string.h>
#include "adc.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include <stdbool.h>

typedef struct _NNde
{
    AdcChannel ch;
    struct _NNde *pNext;
} ChannelNode;

typedef struct
{
    AdcChannel current_ch;
    xSemaphoreHandle sem_lock;
    xSemaphoreHandle done_lock;
    ChannelBuffer data;
    uint16_t idx;
} AdcManager;

AdcManager adc_man;

typedef struct
{
    AdcChannelId id;
    ChannelBuffer data;
    bool data_available;
} Channel;

uint16_t AdcHchannel(AdcChannel id);

void AdcInit()
{
    // adc_man.data_buffer = xQueueCreate(2, sizeof(ChannelBuffer));
    adc_man.current_ch = NULL;
    //adc_man.active_node_list = NULL;
    adc_man.sem_lock = xSemaphoreCreateBinary();
    adc_man.done_lock = xSemaphoreCreateBinary();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    ADC_InitTypeDef adc_config;

    adc_config.ADC_Mode = ADC_Mode_Independent;
    adc_config.ADC_ContinuousConvMode = DISABLE;
    adc_config.ADC_NbrOfChannel = 1;
    adc_config.ADC_DataAlign = ADC_DataAlign_Right;
    adc_config.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc_config.ADC_ScanConvMode = DISABLE;

    ADC_Init(ADC1, &adc_config);

    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    NVIC_EnableIRQ(ADC1_2_IRQn);
    NVIC_SetPriority(ADC1_2_IRQn, 8);
    xSemaphoreGive(adc_man.sem_lock);
}
void AdcStart(AdcChannel ch)
{
    if (xSemaphoreTake(adc_man.sem_lock, 1000) == pdTRUE)
    {
        adc_man.current_ch = ch;
        ADC_RegularChannelConfig(ADC1, AdcHchannel(adc_man.current_ch), 1, ADC_SampleTime_239Cycles5);
        ADC_Cmd(ADC1, ENABLE);
        adc_man.idx = 0;
        while (xSemaphoreTake(adc_man.done_lock, 0) == pdTRUE)
            ;
        memset(&adc_man.data, 0, sizeof(ChannelBuffer));
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        // wait for conversion finish;
        if (xSemaphoreTake(adc_man.done_lock, 1000) == pdTRUE)
        {
            Channel *chn = ch;
            chn->data = adc_man.data;
            chn->data_available = true;
            xSemaphoreGive(adc_man.sem_lock);
        }
    }
}
void AdcStop()
{
    ADC_SoftwareStartConvCmd(ADC1, DISABLE);
}
AdcChannel AdcChannelCreate(AdcChannelId id)
{
    Channel *ch = pvPortMalloc(sizeof(Channel));
    ch->id = id;
    GPIO_TypeDef * port = NULL;
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    switch (id)
    {
    case ADC_CHANNEL0:
        gpio.GPIO_Pin = GPIO_Pin_0;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        port = GPIOA;
        break;
    case ADC_CHANNEL1:
        gpio.GPIO_Pin = GPIO_Pin_1;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        port = GPIOA;
        break;
    case ADC_CHANNEL2:
        gpio.GPIO_Pin = GPIO_Pin_2;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        port = GPIOA;
        break;
    case ADC_CHANNEL3:
        gpio.GPIO_Pin = GPIO_Pin_3;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        port = GPIOA;
        break;
    case ADC_CHANNEL4:
        gpio.GPIO_Pin = GPIO_Pin_4;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        port = GPIOA;
        break;
    case ADC_CHANNEL5:
        gpio.GPIO_Pin = GPIO_Pin_5;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        port = GPIOA;
        break;
    case ADC_CHANNEL6:
        gpio.GPIO_Pin = GPIO_Pin_6;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        port = GPIOA;
        break;
    case ADC_CHANNEL7:
        gpio.GPIO_Pin = GPIO_Pin_7;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        port = GPIOA;
        break;
    case ADC_CHANNEL8:
        gpio.GPIO_Pin = GPIO_Pin_0;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        port = GPIOB;
        break;
    case ADC_CHANNEL9:
        gpio.GPIO_Pin = GPIO_Pin_1;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        port = GPIOB;
        break;
    case ADC_CHANNEL10:
        gpio.GPIO_Pin = GPIO_Pin_0;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        port = GPIOC;
        break;
    case ADC_CHANNEL11:
        gpio.GPIO_Pin = GPIO_Pin_1;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        port = GPIOC;
        break;
    case ADC_CHANNEL12:
        gpio.GPIO_Pin = GPIO_Pin_2;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        port = GPIOC;
        break;
    case ADC_CHANNEL13:
        gpio.GPIO_Pin = GPIO_Pin_3;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        port = GPIOC;
        break;
    case ADC_CHANNEL14:
        gpio.GPIO_Pin = GPIO_Pin_4;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        port = GPIOC;
        break;
    case ADC_CHANNEL15:
        gpio.GPIO_Pin = GPIO_Pin_5;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        port = GPIOC;
        break;
    default:
        break;
    }
    GPIO_Init(port, &gpio);
    return ch;
}


AdcStatus AdcChannelRead(AdcChannel ch, ChannelBuffer *data)
{
    if (ch == NULL || data == NULL)
        return ADC_STATUS_BAD_PTR;
    Channel *chn = ch;
    AdcStart(ch);
    if (chn->data_available == true)
    {
        chn->data_available = false;
        *data = chn->data;
        return ADC_STATUS_SUCCESS;
    }

    return ADC_STATUS_ERROR;
}

uint16_t AdcHchannel(AdcChannel id)
{
    Channel *ch = (Channel *)id;
    switch (ch->id)
    {
    case ADC_CHANNEL0:
        return ADC_Channel_0;
        break;
    case ADC_CHANNEL1:
        return ADC_Channel_1;
        break;
    case ADC_CHANNEL2:
        return ADC_Channel_2;
        break;
    case ADC_CHANNEL3:
        return ADC_Channel_3;
        break;
    case ADC_CHANNEL4:
        return ADC_Channel_4;
        break;
    case ADC_CHANNEL5:
        return ADC_Channel_5;
        break;
    case ADC_CHANNEL6:
        return ADC_Channel_6;
        break;
    case ADC_CHANNEL7:
        return ADC_Channel_7;
        break;
    case ADC_CHANNEL8:
        return ADC_Channel_8;
        break;
    case ADC_CHANNEL9:
        return ADC_Channel_9;
        break;
    case ADC_CHANNEL10:
        return ADC_Channel_10;
        break;
    case ADC_CHANNEL11:
        return ADC_Channel_11;
        break;
    case ADC_CHANNEL12:
        return ADC_Channel_12;
        break;
    case ADC_CHANNEL13:
        return ADC_Channel_13;
        break;
    case ADC_CHANNEL14:
        return ADC_Channel_14;
        break;
    case ADC_CHANNEL15:
        return ADC_Channel_15;
        break;
    case ADC_CHANNEL16:
        return ADC_Channel_16;
        break;
    case ADC_CHANNEL17:
        return ADC_Channel_17;
        break;
    default:
        break;
    }
    return 0;
}

void ADC1_2_IRQHandler()
{
    BaseType_t high_ptask_token;
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC))
    {
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
        uint16_t adc_data = ADC_GetConversionValue(ADC1);
        if (adc_man.current_ch != NULL)
        {
            Channel *cptr = adc_man.current_ch;
            adc_man.data.buffer[adc_man.idx++] = adc_data;
            if (adc_man.idx >= ADC_BUFFER_MAX)
            {
                adc_man.data.id = cptr->id;
                xSemaphoreGiveFromISR(adc_man.done_lock, &high_ptask_token);
                ADC_Cmd(ADC1, DISABLE);
            }
            else
            {
                ADC_SoftwareStartConvCmd(ADC1, ENABLE);
            }
        }
    }
    portYIELD_FROM_ISR(high_ptask_token);
}
