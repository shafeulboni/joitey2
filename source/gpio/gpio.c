
#include <stddef.h>
#include "gpio.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stdlib.h>
//#include "FreeRTOS.h"


GPIO_TypeDef * DecodePortId(PortId port_id)
{
	GPIO_TypeDef * gpio_to_rerturn = NULL;
	switch (port_id) {
		case PORTA:
			gpio_to_rerturn = GPIOA;
			break;
		case PORTB:
			gpio_to_rerturn = GPIOB;
			break;
		case PORTC:
			gpio_to_rerturn = GPIOC;
			break;
		case PORTD:
			gpio_to_rerturn = GPIOD;
			break;
		case PORTE:
			gpio_to_rerturn = GPIOE;
			break;
		case PORTF:
			gpio_to_rerturn = GPIOF;
			break;
		case PORTG:
			gpio_to_rerturn = GPIOG;
			break;
		default:
			break;
	}
	return gpio_to_rerturn;
}

uint16_t DecodePinId(PinId pin_id)
{
	uint16_t pin_mask = 0x10;

	switch(pin_id){
		case PIN0:
			pin_mask = GPIO_Pin_0;
			break;
		case PIN1:
			pin_mask = GPIO_Pin_1;
			break;
		case PIN2:
			pin_mask = GPIO_Pin_2;
			break;
		case PIN3:
			pin_mask = GPIO_Pin_3;
			break;
		case PIN4:
			pin_mask = GPIO_Pin_4;
			break;
		case PIN5:
			pin_mask = GPIO_Pin_5;
			break;
		case PIN6:
			pin_mask = GPIO_Pin_6;
			break;
		case PIN7:
			pin_mask = GPIO_Pin_7;
			break;
		case PIN8:
			pin_mask = GPIO_Pin_8;
			break;
		case PIN9:
			pin_mask = GPIO_Pin_9;
			break;
		case PIN10:
			pin_mask = GPIO_Pin_10;
			break;
		case PIN11:
			pin_mask = GPIO_Pin_11;
			break;
		case PIN12:
			pin_mask = GPIO_Pin_12;
			break;
		case PIN13:
			pin_mask = GPIO_Pin_13;
			break;
		case PIN14:
			pin_mask = GPIO_Pin_14;
			break;
		case PIN15:
			pin_mask = GPIO_Pin_15;
			break;
		default:
			break;
	}
	return pin_mask;
}

GPIOMode_TypeDef DecodePinMode(PinMode pin_mode)
{
	GPIOMode_TypeDef gpio_mode = GPIO_Mode_Out_PP;
	switch(pin_mode){
		case OUTPUT:
			gpio_mode = GPIO_Mode_Out_PP;
			break;
		case INPUT:
			gpio_mode = GPIO_Mode_IN_FLOATING;
			break;
		default:
			break;
	}
	return gpio_mode;
}


uint32_t GpioClockMask(PortId gpio_id)
{
	uint32_t gpio_clock_cmd_mask = 0;
	switch(gpio_id)
	{
		case PORTA:
			gpio_clock_cmd_mask = RCC_APB2Periph_GPIOA;
			break;
		case PORTB:
			gpio_clock_cmd_mask = RCC_APB2Periph_GPIOB;
			break;
		case PORTC:
			gpio_clock_cmd_mask = RCC_APB2Periph_GPIOC;
			break;
		case PORTD:
			gpio_clock_cmd_mask = RCC_APB2Periph_GPIOD;
			break;
		case PORTE:
			gpio_clock_cmd_mask = RCC_APB2Periph_GPIOE;
			break;
		case PORTF:
			gpio_clock_cmd_mask = RCC_APB2Periph_GPIOF;
			break;
		case PORTG:
			gpio_clock_cmd_mask = RCC_APB2Periph_GPIOG;
			break;

	}
	return gpio_clock_cmd_mask;
}

PinType pin_list[30 ];
int list_indexer = 0;



PinHandle InitPin(PortId port_id, PinId pin_id, PinMode pin_mode)
{


	PinType * new_pin = &pin_list[list_indexer++];

	if (new_pin == NULL) {
		return NULL;
	}

	new_pin->port_id = port_id;
	new_pin->pin_id = pin_id;
	new_pin->pin_mode = pin_mode;

	GPIO_TypeDef * gpio_base = DecodePortId(port_id);

	GPIO_InitTypeDef config_gpio;
	config_gpio.GPIO_Mode = DecodePinMode(pin_mode);
	config_gpio.GPIO_Pin = DecodePinId(pin_id);
	config_gpio.GPIO_Speed = GPIO_Speed_50MHz;

	RCC_APB2PeriphClockCmd(GpioClockMask(port_id), ENABLE);
	GPIO_Init(gpio_base, &config_gpio);


	return new_pin;
}




void SetPinState(PinHandle handle, PinState expected_state)
{
	assert_param(handle);

	PinType * pin = (PinType *) handle;
	if (pin->pin_mode == INPUT) {
		return;
	}
	if (expected_state == HIGH) {
		GPIO_SetBits(DecodePortId(pin->port_id), DecodePinId(pin->pin_id));
	} else {
		GPIO_ResetBits(DecodePortId(pin->port_id), DecodePinId(pin->pin_id));
	}
}




PinState GetPinState(PinHandle handle)
{
	assert_param(handle);

	PinType * pin = (PinType *) handle;
	if (pin->pin_mode == OUTPUT) {
		return LOW;
	}

	BitAction bit_val = GPIO_ReadInputDataBit(DecodePortId(pin->port_id), DecodePinId(pin->pin_id));

	if (bit_val != Bit_RESET) {
		return HIGH;
	}

	return LOW;
}



void TogglePinState(PinHandle handle)
{
	assert_param(handle);

	PinType * pin = (PinType *) handle;
	if (pin->pin_mode == INPUT) {
		return;
	}
	GPIO_TypeDef * port = DecodePortId(pin->port_id);
	uint16_t spin = DecodePinId(pin->pin_id);
	if(GPIO_ReadOutputDataBit(port, spin) == Bit_SET)
	{
		GPIO_WriteBit(port, spin, Bit_RESET);
	}
	else
	{
		GPIO_WriteBit(port, spin, Bit_SET);
	}
}


