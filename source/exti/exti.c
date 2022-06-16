

#include <exti.h>
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include <stdbool.h>



typedef struct
{
    uint8_t port;
    ExtiId id;
    ExtiTriggare trig;
    ExtiCallback callback;
    void * usrp;
}ExtiChannelPrv;


static ExtiChannelPrv * exti_lst[20];

void ExtiInit()
{

}
ExtiChannel ExtiChannelCreate(uint8_t port, ExtiId id, ExtiTriggare trig, ExtiCallback callback, void * userp)
{
    if(exti_lst[id] != NULL)
        return exti_lst[id];
    
    ExtiChannelPrv * ext = pvPortMalloc(sizeof(ExtiChannelPrv));
    ext->id = id;
    ext->port = port;
    ext->trig = trig;
    ext->usrp = userp;
    ext->callback = callback;
    exti_lst[id] = ext;
    return ext;
}

void ExtiEnable(ExtiChannel ch)
{
    if(ch == NULL)
        return;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    ExtiChannelPrv * chn = (ExtiChannelPrv *)ch;
    uint8_t port_source;
    uint8_t pin_source = 0;
    uint32_t exti_line = 0;

    GPIO_InitTypeDef gpio_config;
	gpio_config.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_TypeDef * gpio = NULL;
    switch (chn->port)
    {
    case PORTA:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        gpio = GPIOA;
        port_source = GPIO_PortSourceGPIOA;
        break;
    case PORTB:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        gpio = GPIOB;
        port_source = GPIO_PortSourceGPIOB;
        break;
    case PORTC:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        gpio = GPIOC;
        port_source = GPIO_PortSourceGPIOC;
        break;
    case PORTD:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
        gpio = GPIOD;
        port_source = GPIO_PortSourceGPIOD;
        break;
    case PORTE:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
        gpio = GPIOE;
        port_source = GPIO_PortSourceGPIOE;
        break;  
    case PORTF:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
        gpio = GPIOF;
        port_source = GPIO_PortSourceGPIOF;
        break;
    case PORTG:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
        gpio = GPIOG;
        port_source = GPIO_PortSourceGPIOG;
        break;
    
    default:
        return;
        break;
    }

    switch (chn->id)
    {
    case EXTI_ID_0:
        gpio_config.GPIO_Pin = GPIO_Pin_0;
        pin_source = GPIO_PinSource0;
        exti_line = EXTI_Line0;
        break;
    case EXTI_ID_1:
        gpio_config.GPIO_Pin = GPIO_Pin_1;
        pin_source = GPIO_PinSource1;
        exti_line = EXTI_Line1;
        break;
    case EXTI_ID_2:
        gpio_config.GPIO_Pin = GPIO_Pin_2;
        pin_source = GPIO_PinSource2;
        exti_line = EXTI_Line2;
        break;
    case EXTI_ID_3:
        gpio_config.GPIO_Pin = GPIO_Pin_3;
        pin_source = GPIO_PinSource3;
        exti_line = EXTI_Line3;
        break;
    case EXTI_ID_4:
        gpio_config.GPIO_Pin = GPIO_Pin_4;
        pin_source = GPIO_PinSource4;
        exti_line = EXTI_Line4;
        break;
    case EXTI_ID_5:
        gpio_config.GPIO_Pin = GPIO_Pin_5;
        pin_source = GPIO_PinSource5;
        exti_line = EXTI_Line5;
        break;
    case EXTI_ID_6:
        gpio_config.GPIO_Pin = GPIO_Pin_6;
        pin_source = GPIO_PinSource6;
        exti_line = EXTI_Line6;
        break;
    case EXTI_ID_7:
        gpio_config.GPIO_Pin = GPIO_Pin_7;
        pin_source = GPIO_PinSource7;
        exti_line = EXTI_Line7;
        break;
    case EXTI_ID_8:
        gpio_config.GPIO_Pin = GPIO_Pin_8;
        pin_source = GPIO_PinSource8;
        exti_line = EXTI_Line8;
        break;
    case EXTI_ID_9:
        gpio_config.GPIO_Pin = GPIO_Pin_9;
        pin_source = GPIO_PinSource9;
        exti_line = EXTI_Line9;
        break;
    case EXTI_ID_10:
        gpio_config.GPIO_Pin = GPIO_Pin_10;
        pin_source = GPIO_PinSource10;
        exti_line = EXTI_Line10;
        break;
    case EXTI_ID_11:
        gpio_config.GPIO_Pin = GPIO_Pin_11;
        pin_source = GPIO_PinSource11;
        exti_line = EXTI_Line11;
        break;
    case EXTI_ID_12:
        gpio_config.GPIO_Pin = GPIO_Pin_12;
        pin_source = GPIO_PinSource12;
        exti_line = EXTI_Line12;
        break;
    case EXTI_ID_13:
        gpio_config.GPIO_Pin = GPIO_Pin_13;
        pin_source = GPIO_PinSource13;
        exti_line = EXTI_Line13;
        break;
    case EXTI_ID_14:
        gpio_config.GPIO_Pin = GPIO_Pin_14;
        pin_source = GPIO_PinSource14;
        exti_line = EXTI_Line14;
        break;
    case EXTI_ID_15:
        gpio_config.GPIO_Pin = GPIO_Pin_15;
        pin_source = GPIO_PinSource15;
        exti_line = EXTI_Line15;
        break;
    case EXTI_ID_16:
        exti_line = EXTI_Line16;
        break;
    case EXTI_ID_17:
        exti_line = EXTI_Line17;
        break;
    case EXTI_ID_18:
        exti_line = EXTI_Line18;
        break;
    case EXTI_ID_19:
        exti_line = EXTI_Line19;
        break;
    default:
        break;
    }


    if(chn->id < 16)
    {
        GPIO_Init(gpio, &gpio_config);
        GPIO_EXTILineConfig(port_source, pin_source);
    }
	
    EXTI_InitTypeDef exti_config;
	exti_config.EXTI_Line = exti_line;
	exti_config.EXTI_LineCmd = ENABLE;
	exti_config.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_config.EXTI_Trigger = chn->trig == EXTI_TRIGGARE_RISING? EXTI_Trigger_Rising : chn->trig == EXTI_TRIGGARE_FALLING? EXTI_Trigger_Falling : EXTI_Trigger_Rising_Falling;
	EXTI_Init(&exti_config);

    switch (chn->id)
    {
    case EXTI_ID_0:
        NVIC_SetPriority(EXTI0_IRQn, 5);
	    NVIC_EnableIRQ(EXTI0_IRQn);
        break;
    case EXTI_ID_1:
        NVIC_SetPriority(EXTI1_IRQn, 5);
	    NVIC_EnableIRQ(EXTI1_IRQn);
        break;
    case EXTI_ID_2:
        NVIC_SetPriority(EXTI2_IRQn, 5);
	    NVIC_EnableIRQ(EXTI2_IRQn);
        break;
    case EXTI_ID_3:
        NVIC_SetPriority(EXTI3_IRQn, 5);
	    NVIC_EnableIRQ(EXTI3_IRQn);
        break;
    case EXTI_ID_4:
        NVIC_SetPriority(EXTI4_IRQn, 5);
	    NVIC_EnableIRQ(EXTI4_IRQn);
        break;
    case EXTI_ID_5:
    case EXTI_ID_6:
    case EXTI_ID_7:
    case EXTI_ID_8:
    case EXTI_ID_9:
        NVIC_SetPriority(EXTI9_5_IRQn, 5);
	    NVIC_EnableIRQ(EXTI9_5_IRQn);
        break;
    case EXTI_ID_10:
    case EXTI_ID_11:
    case EXTI_ID_12:
    case EXTI_ID_13:
    case EXTI_ID_14:
    case EXTI_ID_15:
        NVIC_SetPriority(EXTI15_10_IRQn, 5);
	    NVIC_EnableIRQ(EXTI15_10_IRQn);
        break;
    default:
        break;
    }    
}

void ExtiDisable(ExtiChannel ch)
{

}

void ExtiRegisterHandler(ExtiChannel ch, ExtiCallback callback)
{
    if(ch == NULL || callback == NULL)
        return;
    ExtiChannelPrv * chn = (ExtiChannelPrv *) ch;
    chn->callback = callback;
}



void EXTI0_IRQHandler()
{
	BaseType_t wake_token = pdFALSE;
	if (EXTI_GetITStatus(EXTI_Line0) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line0);
        if(exti_lst[0] != NULL && exti_lst[0]->callback != NULL)
            exti_lst[0]->callback(&wake_token, exti_lst[0]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
}


void EXTI1_IRQHandler()
{
	BaseType_t wake_token = pdFALSE;
	if (EXTI_GetITStatus(EXTI_Line1) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line1);
        if(exti_lst[1] != NULL && exti_lst[1]->callback != NULL)
            exti_lst[1]->callback(&wake_token, exti_lst[1]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
}

void EXTI2_IRQHandler()
{
	BaseType_t wake_token = pdFALSE;
	if (EXTI_GetITStatus(EXTI_Line2) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line2);
        if(exti_lst[2] != NULL && exti_lst[2]->callback != NULL)
            exti_lst[2]->callback(&wake_token, exti_lst[2]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
}


void EXTI3_IRQHandler()
{
	BaseType_t wake_token = pdFALSE;
	if (EXTI_GetITStatus(EXTI_Line3) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line3);
        if(exti_lst[3] != NULL && exti_lst[3]->callback != NULL)
            exti_lst[3]->callback(&wake_token, exti_lst[3]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
}


void EXTI4_IRQHandler()
{
	BaseType_t wake_token = pdFALSE;
	if (EXTI_GetITStatus(EXTI_Line4) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line4);
        if(exti_lst[4] != NULL && exti_lst[4]->callback != NULL)
            exti_lst[4]->callback(&wake_token, exti_lst[4]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
}


void EXTI9_5_IRQHandler()
{
	BaseType_t wake_token = pdFALSE;
	if (EXTI_GetITStatus(EXTI_Line5) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line5);
        if(exti_lst[5] != NULL && exti_lst[5]->callback != NULL)
            exti_lst[5]->callback(&wake_token, exti_lst[5]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
    else if (EXTI_GetITStatus(EXTI_Line6) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line6);
        if(exti_lst[6] != NULL && exti_lst[6]->callback != NULL)
            exti_lst[6]->callback(&wake_token, exti_lst[6]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
    else if (EXTI_GetITStatus(EXTI_Line7) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line7);
        if(exti_lst[7] != NULL && exti_lst[7]->callback != NULL)
            exti_lst[7]->callback(&wake_token, exti_lst[7]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
    else if (EXTI_GetITStatus(EXTI_Line8) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line8);
        if(exti_lst[8] != NULL && exti_lst[8]->callback != NULL)
            exti_lst[8]->callback(&wake_token, exti_lst[8]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
    else if (EXTI_GetITStatus(EXTI_Line9) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line9);
        if(exti_lst[9] != NULL && exti_lst[9]->callback != NULL)
            exti_lst[9]->callback(&wake_token, exti_lst[9]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
}

void EXTI15_10_IRQHandler()
{
	BaseType_t wake_token = pdFALSE;
	if (EXTI_GetITStatus(EXTI_Line15) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line15);
        if(exti_lst[15] != NULL && exti_lst[15]->callback != NULL)
            exti_lst[15]->callback(&wake_token, exti_lst[15]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
    else if (EXTI_GetITStatus(EXTI_Line14) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line14);
        if(exti_lst[14] != NULL && exti_lst[14]->callback != NULL)
            exti_lst[14]->callback(&wake_token, exti_lst[14]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
    else if (EXTI_GetITStatus(EXTI_Line13) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line13);
        if(exti_lst[13] != NULL && exti_lst[13]->callback != NULL)
            exti_lst[13]->callback(&wake_token, exti_lst[13]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
    else if (EXTI_GetITStatus(EXTI_Line12) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line12);
        if(exti_lst[12] != NULL && exti_lst[12]->callback != NULL)
            exti_lst[12]->callback(&wake_token, exti_lst[12]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
    else if (EXTI_GetITStatus(EXTI_Line11) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line11);
        if(exti_lst[11] != NULL && exti_lst[11]->callback != NULL)
            exti_lst[11]->callback(&wake_token, exti_lst[11]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
    else if (EXTI_GetITStatus(EXTI_Line10) == SET)
	{
        EXTI_ClearITPendingBit(EXTI_Line10);
        if(exti_lst[10] != NULL && exti_lst[10]->callback != NULL)
            exti_lst[10]->callback(&wake_token, exti_lst[10]->usrp);
		portYIELD_FROM_ISR (wake_token);
	}
}

