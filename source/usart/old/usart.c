#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "usart.h"
#include "usart_config.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "FreeRTOS.h"



static UsartHandle usart_handle_list[4];



IRQn_Type GetIRQnFromId(ComId id)
{
	IRQn_Type irq_type = BusFault_IRQn;
	switch(id)
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


uint32_t GetGpioClockMask(GPIO_TypeDef * gpio)
{
	uint32_t gpio_clock_cmd_mask;
	if(gpio == GPIOA)
		gpio_clock_cmd_mask = RCC_APB2Periph_GPIOA;
	else if (gpio == GPIOB)
		gpio_clock_cmd_mask = RCC_APB2Periph_GPIOB;
	else if (gpio == GPIOC)
		gpio_clock_cmd_mask = RCC_APB2Periph_GPIOC;
	else
		gpio_clock_cmd_mask = 0x00;

	return gpio_clock_cmd_mask;
}



uint32_t ConfigUsartGpio(ComId id)
{
	GPIO_InitTypeDef config_gpio;
	GPIO_TypeDef * tx_port;
	GPIO_TypeDef * rx_port;
	uint16_t tx_pin;
	uint16_t rx_pin;

	// Get pin and port name of current USART
	switch(id)
	{
		case COM1:
			rx_port = USATR1_RX_PORT;
			tx_port = USATR1_TX_PORT;
			rx_pin  = USART1_RX_PIN;
			tx_pin  = USART1_TX_PIN;
			break;
		case COM2:
			rx_port = USATR2_RX_PORT;
			tx_port = USATR2_TX_PORT;
			rx_pin  = USART2_RX_PIN;
			tx_pin  = USART2_TX_PIN;
			break;
		case COM3:
			rx_port = USATR3_RX_PORT;
			tx_port = USATR3_TX_PORT;
			rx_pin  = USART3_RX_PIN;
			tx_pin  = USART3_TX_PIN;
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

USART_TypeDef * DecodeUsartId(ComId id)
{
	USART_TypeDef * com_handle = NULL;
	switch(id)
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
		default:
			com_handle = NULL;
			break;
	}
	return com_handle;
}


void StartUsartClock(ComId id)
{
	switch(id)
	{
		case COM1:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
			break;
		case COM2:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
			break;
		case COM3:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
			break;
		default:
			break;
	}
}


UsartHandle InitUsart(ComId id, uint32_t baud, uint32_t tx_buffer_limit, uint32_t rx_buffer_limit)
{
//	UsartType * temp = (UsartType *)usart_handle_list[id];
//	if( temp != NULL || temp->is_initialised )
//		return temp;
	if(ConfigUsartGpio(id) == 0)
		return NULL;

	USART_InitTypeDef config_usart;
	// fill usart configuration structure.
	config_usart.USART_BaudRate = baud;
	config_usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	config_usart.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
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
	NVIC_SetPriority(GetIRQnFromId(id), 6);
	USART_Cmd(DecodeUsartId(id), ENABLE);


	// Hardware UART initialisation finished.
	// Now FIFO initialisation done here.
	UsartType * usart = pvPortMalloc(sizeof(UsartType));

	if(usart != NULL)
	{

		usart->baud = baud;
		usart->port_id = id;
		// Initialise receiver buffer.
		usart->rx_buffer = xQueueCreate(rx_buffer_limit, sizeof(uint8_t));
		if (usart->rx_buffer == NULL) {
			//kFree(usart);
			return NULL;
		}
		usart->is_initialised = 1;
	}
	else
	{
		return NULL;
	}
	usart_handle_list[id] = usart;
	return usart;
}





void UsartSendByte(UsartHandle handle, uint8_t data)
{

	UsartType * usart = (UsartType *) handle;
	// Load transmit buffer with the value.
	while(USART_GetFlagStatus(DecodeUsartId(usart->port_id), USART_FLAG_TXE) == 0);
	USART_SendData(DecodeUsartId(usart->port_id), data);
}


void UsartSendString(UsartHandle handle, const char * string)
{
	uint16_t len = strlen(string);

	for (int i = 0; i < len; i++)
	{
		UsartSendByte(handle, string[i]);
	}
}

void UsartSendBinary(UsartHandle handle, uint8_t * buffer, uint16_t length)
{
	for(int i = 0; i < length; i++)
	{
		UsartSendByte(handle, buffer[i]);
	}
}

uint32_t UsartReceiveByte(UsartHandle handle, uint8_t * data)
{

	UsartType * usart = (UsartType *) handle;
	if(xQueueReceive(usart->rx_buffer, data, 20000) == pdPASS)
	{
		return 1;
	}
	return 0;
}


void ResetUsartBuffer(UsartHandle handle)
{


	//UsartType * usart = (UsartType *) handle;
	//ResetByteFifo(usart->rx_buffer);
	//ResetByteFifo(usart->tx_buffer);
}
uint32_t IsInitialised(UsartHandle handle)
{


	UsartType * usart = (UsartType *) handle;

	return usart->is_initialised;
}



///Interrupt Handler for all serial port.


void USART1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	UsartType * usart = usart_handle_list[COM1];
	uint8_t data;


	if (USART_GetITStatus(USART1, USART_IT_RXNE) == 1) {
		data = USART_ReceiveData(USART1);
		//PushByteFromISR(usart->rx_buffer, data);
		xQueueSendFromISR(usart->rx_buffer, &data, &xHigherPriorityTaskWoken);
	}

	if( xHigherPriorityTaskWoken )
	{
		// Actual macro used here is port specific.
		portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
	}
}





void USART2_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	UsartType * usart = usart_handle_list[COM2];
	uint8_t data;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == 1) {
		data = USART_ReceiveData(USART2);
		xQueueSendFromISR(usart->rx_buffer, &data, &xHigherPriorityTaskWoken);
	}
	if( xHigherPriorityTaskWoken )
	{
		// Actual macro used here is port specific.
		portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
	}
}


void USART3_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	UsartType * usart = usart_handle_list[COM3];
	uint8_t data;
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == 1) {
		data = USART_ReceiveData(USART3);
		xQueueSendFromISR(usart->rx_buffer, &data, &xHigherPriorityTaskWoken);
	}
	if( xHigherPriorityTaskWoken )
	{
		// Actual macro used here is port specific.
		portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
	}
}

