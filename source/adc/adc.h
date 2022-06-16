


#ifndef ADC_INCLUDE_H
#define ADC_INCLUDE_H



#define ADC_BUFFER_MAX      64

typedef void * AdcChannel;


typedef enum 
{
    ADC_CHANNEL0,
    ADC_CHANNEL1,
    ADC_CHANNEL2,
    ADC_CHANNEL3,
    ADC_CHANNEL4,
    ADC_CHANNEL5,
    ADC_CHANNEL6,
    ADC_CHANNEL7,
    ADC_CHANNEL8,
    ADC_CHANNEL9,
    ADC_CHANNEL10,
    ADC_CHANNEL11,
    ADC_CHANNEL12,
    ADC_CHANNEL13,
    ADC_CHANNEL14,
    ADC_CHANNEL15,
    ADC_CHANNEL16,
    ADC_CHANNEL17
}AdcChannelId;


typedef enum 
{
    ADC_STATUS_SUCCESS = 0,
    ADC_STATUS_ERROR,
    ADC_STATUS_BAD_PTR,
    ADC_STATUS_TIMEOUT
}AdcStatus;


typedef struct 
{
    AdcChannelId id;
    uint16_t buffer[ADC_BUFFER_MAX];
}ChannelBuffer;


void AdcInit();
// void AdcStart(AdcChannel ch);
// void AdcStop();
AdcChannel AdcChannelCreate(AdcChannelId id);
AdcStatus AdcChannelRead(AdcChannel ch, ChannelBuffer * data); 


#endif  // ADC_INCLUDE_H
