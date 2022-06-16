#include <stddef.h>
#include <stdlib.h>
#include "usart.h"
#include "usart_config.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef struct
{
	ComId port_id;
	uint32_t baud;
	xQueueHandle rx_fifo;
	uint32_t is_initialised;
} UsartType;

static UsartType usart_handle_list[7];

uint16_t PinSourceDecode(uint16_t pin)
{
	uint16_t pin_source = 0x10;
	switch (pin)
	{
	case GPIO_Pin_0:
		pin_source = GPIO_PinSource0;
		break;
	case GPIO_Pin_1:
		pin_source = GPIO_PinSource1;
		break;
	case GPIO_Pin_2:
		pin_source = GPIO_PinSource2;
		break;
	case GPIO_Pin_3:
		pin_source = GPIO_PinSource3;
		break;
	case GPIO_Pin_4:
		pin_source = GPIO_PinSource4;
		break;
	case GPIO_Pin_5:
		pin_source = GPIO_PinSource5;
		break;
	case GPIO_Pin_6:
		pin_source = GPIO_PinSource6;
		break;
	case GPIO_Pin_7:
		pin_source = GPIO_PinSource7;
		break;
	case GPIO_Pin_8:
		pin_source = GPIO_PinSource8;
		break;
	case GPIO_Pin_9:
		pin_source = GPIO_PinSource9;
		break;
	case GPIO_Pin_10:
		pin_source = GPIO_PinSource10;
		break;
	case GPIO_Pin_11:
		pin_source = GPIO_PinSource11;
		break;
	case GPIO_Pin_12:
		pin_source = GPIO_PinSource12;
		break;
	case GPIO_Pin_13:
		pin_source = GPIO_PinSource13;
		break;
	case GPIO_Pin_14:
		pin_source = GPIO_PinSource14;
		break;
	case GPIO_Pin_15:
		pin_source = GPIO_PinSource15;
		break;
	default:
		break;
	}
	return pin_source;
}

IRQn_Type GetIRQnFromId(ComId id)
{
	IRQn_Type irq_type = BusFault_IRQn;
	switch (id)
	{
	case COM1:
		irq_type = USART1_IRQn;
		break;
	case COM2:
		irq_type = USART2_IRQn;
		break;
	case COM3:
		irq_type = USART3_IRQn;
		break;
	default:
		break;
	}
	return irq_type;
}

uint32_t GetGpioClockMask(GPIO_TypeDef *gpio)
{
	uint32_t gpio_clock_cmd_mask;
	if (gpio == GPIOA)
		gpio_clock_cmd_mask = RCC_APB2Periph_GPIOA;
	else if (gpio == GPIOB)
		gpio_clock_cmd_mask = RCC_APB2Periph_GPIOB;
	else if (gpio == GPIOC)
		gpio_clock_cmd_mask = RCC_APB2Periph_GPIOC;
	else if (gpio == GPIOD)
		gpio_clock_cmd_mask = RCC_APB2Periph_GPIOD;
	else if (gpio == GPIOE)
		gpio_clock_cmd_mask = RCC_APB2Periph_GPIOE;
	else if (gpio == GPIOF)
		gpio_clock_cmd_mask = RCC_APB2Periph_GPIOF;
	else if (gpio == GPIOG)
		gpio_clock_cmd_mask = RCC_APB2Periph_GPIOG;
	else
		gpio_clock_cmd_mask = 0x00;

	return gpio_clock_cmd_mask;
}

uint32_t ConfigUsartGpio(ComId id)
{
	GPIO_InitTypeDef config_gpio;
	GPIO_TypeDef *tx_port;
	GPIO_TypeDef *rx_port;
	uint16_t tx_pin;
	uint16_t rx_pin;

	// Get pin and port name of current USART
	switch (id)
	{
	case COM1:
		rx_port = USATR1_RX_PORT;
		tx_port = USATR1_TX_PORT;
		rx_pin = USART1_RX_PIN;
		tx_pin = USART1_TX_PIN;
		break;
	case COM2:
		rx_port = USATR2_RX_PORT;
		tx_port = USATR2_TX_PORT;
		rx_pin = USART2_RX_PIN;
		tx_pin = USART2_TX_PIN;
		break;
	case COM3:
		rx_port = USATR3_RX_PORT;
		tx_port = USATR3_TX_PORT;
		rx_pin = USART3_RX_PIN;
		tx_pin = USART3_TX_PIN;
		break;
	default:
		return 0;
		break;
	}
	// Fill usart config struct member
	config_gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	config_gpio.GPIO_Pin = tx_pin;
	config_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	//enable gpio clock.
	RCC_APB2PeriphClockCmd(GetGpioClockMask(tx_port), ENABLE);
	//now called hardware initialise function
	GPIO_Init(tx_port, &config_gpio);

	// Fill usart config struct member
	config_gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	config_gpio.GPIO_Pin = rx_pin;
	config_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	//enable gpio clock.
	RCC_APB2PeriphClockCmd(GetGpioClockMask(rx_port), ENABLE);
	//now called hardware initialise function
	GPIO_Init(rx_port, &config_gpio);
	return 1;
}

USART_TypeDef *DecodeUsartId(ComId id)
{
	USART_TypeDef *com_handle = NULL;
	switch (id)
	{
	case COM1:
		com_handle = USART1;
		break;
	case COM2:
		com_handle = USART2;
		break;
	case COM3:
		com_handle = USART3;
		break;
	case COM4:
		com_handle = UART4;
		break;
	case COM5:
		com_handle = UART5;
		break;
	default:
		com_handle = NULL;
		break;
	}
	return com_handle;
}

void StartUsartClock(ComId id)
{
	uint32_t usart_clock_cmd_mask = 0x00;

	switch (id)
	{
	case COM1:
		usart_clock_cmd_mask = RCC_APB2Periph_USART1;
		RCC_APB2PeriphClockCmd(usart_clock_cmd_mask, ENABLE);
		break;
	case COM2:
		usart_clock_cmd_mask = RCC_APB1Periph_USART2;
		RCC_APB1PeriphClockCmd(usart_clock_cmd_mask, ENABLE);
		break;
	case COM3:
		usart_clock_cmd_mask = RCC_APB1Periph_USART3;
		RCC_APB1PeriphClockCmd(usart_clock_cmd_mask, ENABLE);
		break;
	case COM4:
		usart_clock_cmd_mask = RCC_APB1Periph_UART4;
		RCC_APB1PeriphClockCmd(usart_clock_cmd_mask, ENABLE);
		break;
	case COM5:
		usart_clock_cmd_mask = RCC_APB1Periph_UART5;
		RCC_APB1PeriphClockCmd(usart_clock_cmd_mask, ENABLE);
		break;
	default:
		break;
	}
}

UsartHandle InitUsart(ComId id, uint32_t baud, uint32_t tx_buffer_limit, uint32_t rx_buffer_limit)
{

	if (ConfigUsartGpio(id) == 0)
		return NULL;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	USART_InitTypeDef config_usart;
	// fill usart configuration structure.
	config_usart.USART_BaudRate = baud;
	config_usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	config_usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	config_usart.USART_Parity = USART_Parity_No;
	config_usart.USART_StopBits = USART_StopBits_1;
	config_usart.USART_WordLength = USART_WordLength_8b;
	//Enable UART clock.
	StartUsartClock(id);

	//Initialise UART using configuration structure.

	USART_Init(DecodeUsartId(id), &config_usart);
	//Enable UART receive interrupt service
	USART_ITConfig(DecodeUsartId(id), USART_IT_RXNE, ENABLE);
	// Enable the specific UART
	NVIC_EnableIRQ(GetIRQnFromId(id));
	NVIC_SetPriority(GetIRQnFromId(id), INTERRUPT_PRIORITY_USART);
	USART_Cmd(DecodeUsartId(id), ENABLE);

	// Hardware UART initialisation finished.
	// Now FIFO initialisation done here.
	UsartType *usart = &usart_handle_list[id];

	if (usart != NULL)
	{
		usart->rx_fifo = xQueueCreate(rx_buffer_limit, sizeof(uint8_t));
		usart->baud = baud;
		usart->port_id = id;
		// Initialise receiver buffer.

		usart->is_initialised = 1;
	}
	else
	{
		return NULL;
	}
	return usart;
}

void UsartSendByte(UsartHandle handle, uint8_t data)
{
	assert_param(handle);
	UsartType *usart = (UsartType *)handle;
	// Load transmit buffer with the value.
	//while(PushByte(usart->tx_buffer, data) != FIFOOK);
	//xQueueSend(usart->tx_buffer,&data, 100);
	USART_SendData(DecodeUsartId(usart->port_id), data);
	while (USART_GetFlagStatus(DecodeUsartId(usart->port_id), USART_FLAG_TXE) == 0)
		;
	//Enable Transmit Register Empty interrupt service so FIFO data transmission will began
	//USART_ITConfig(DecodeUsartId(usart->port_id), USART_IT_TXE, ENABLE);
}

void UsartSendString(UsartHandle handle, const char *string, uint32_t len)
{
	assert_param(handle);
	assert_param(string);

	for (int i = 0; i < len; i++)
	{
		UsartSendByte(handle, string[i]);
	}
}

uint32_t UsartReceiveByte(UsartHandle handle, uint8_t *data)
{

	assert_param(handle);
	assert_param(data);
	UsartType *usart = (UsartType *)handle;
	if (xQueueReceive(usart->rx_fifo, data, 10000) == pdFALSE)
		return 0;
	return 1;
}

void ResetUsartBuffer(UsartHandle handle)
{
	assert_param(handle);

	//UsartType * usart = (UsartType *) handle;
	//ResetByteFifo(usart->rx_buffer);
	//ResetByteFifo(usart->tx_buffer);
}
uint32_t IsInitialised(UsartHandle handle)
{
	assert_param(handle);

	UsartType *usart = (UsartType *)handle;

	return usart->is_initialised;
}

///Interrupt Handler for all serial port.

void USART1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	UsartType *usart = &usart_handle_list[COM1];

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == 1)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		uint8_t data = (uint8_t)USART_ReceiveData(USART1);
		xQueueSendFromISR(usart->rx_fifo, &data, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void USART2_IRQHandler(void)
{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		UsartType * usart = &usart_handle_list[COM2];
		
		if (USART_GetITStatus(USART2, USART_IT_RXNE) == 1) {
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);
			uint8_t data = USART_ReceiveData(USART2);
			xQueueSendFromISR(usart->rx_fifo, &data, &xHigherPriorityTaskWoken);
		}
		portYIELD_FROM_ISR (xHigherPriorityTaskWoken);

}

// void USART3_IRQHandler(void)
// {
// 		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
// 		UsartType * usart = &usart_handle_list[COM3];
		
// 		if (USART_GetITStatus(USART3, USART_IT_RXNE) == 1) {
// 			USART_ClearITPendingBit(USART3, USART_IT_RXNE);
// 			uint8_t data = USART_ReceiveData(USART3);
// 			xQueueSendFromISR(usart->rx_fifo, &data, &xHigherPriorityTaskWoken);
// 		}
// 		portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
		
// }
/*
void UART4_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	UsartType *usart = &usart_handle_list[COM4];
	
	if (USART_GetITStatus(UART4, USART_IT_RXNE) == 1)
	{
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		uint8_t data = USART_ReceiveData(UART4);
		xQueueSendFromISR(usart->rx_fifo, &data, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
}
*/
//
//void UART5_IRQHandler(void)
//{
//	BaseType_t xHigherPriorityTaskWoken;
//	xHigherPriorityTaskWoken = pdFALSE;
//	UsartType * usart = usart_handle_list[COM5];
//	uint8_t data;
//	if (USART_GetITStatus(UART5, USART_IT_RXNE) == 1) {
//		data = USART_ReceiveData(UART5);
//		xQueueSendFromISR(usart->rx_buffer, &data, &xHigherPriorityTaskWoken);
//	}
//	else if(USART_GetITStatus(UART5, USART_IT_TXE) == 1) {
//		if (xQueueReceiveFromISR(usart->tx_buffer, &data, & xHigherPriorityTaskWoken) == pdPASS ) {
//			USART_SendData(UART5, data);
//		}
//		else {
//			USART_ITConfig(UART5, USART_IT_TXE, DISABLE);
//		}
//	}
//	if( xHigherPriorityTaskWoken )
//	{
//		// Actual macro used here is port specific.
//		portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
//	}
//}

/*void USART6_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	UsartType * usart = usart_handle_list[COM6];
	uint8_t data;
	if (USART_GetITStatus(USART6, USART_IT_RXNE) == 1) {
		data = USART_ReceiveData(USART6);
		xQueueSendFromISR(usart->rx_buffer, &data, &xHigherPriorityTaskWoken);
	}
	else if(USART_GetITStatus(USART6, USART_IT_TXE) == 1) {
		if (xQueueReceiveFromISR(usart->tx_buffer, &data, & xHigherPriorityTaskWoken) == pdPASS ) {
			USART_SendData(USART6, data);
		}
		else {
			USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
		}
	}
	if( xHigherPriorityTaskWoken )
	{
		// Actual macro used here is port specific.
		portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
	}
}
*/
