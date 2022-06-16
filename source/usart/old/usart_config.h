

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

#define USATR1_TX_PORT GPIOA
#define USATR1_RX_PORT GPIOA
#define USART1_TX_PIN GPIO_Pin_9
#define USART1_RX_PIN GPIO_Pin_10


#define USATR2_TX_PORT GPIOA
#define USATR2_RX_PORT GPIOA
#define USART2_TX_PIN GPIO_Pin_2
#define USART2_RX_PIN GPIO_Pin_3


#define USATR3_TX_PORT GPIOB
#define USATR3_RX_PORT GPIOB
#define USART3_TX_PIN GPIO_Pin_10
#define USART3_RX_PIN GPIO_Pin_11




// define the priority level for each interrupt.
#define INTERRUPT_PRIORITY_USART 	6
#define INTERRUPT_PRIORITY_USART1 	6
#define INTERRUPT_PRIORITY_USART2	6
