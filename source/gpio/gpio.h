


#ifndef GPIO_H
#define GPIO_H

typedef void * PinHandle;

typedef enum
{
	PORTA,
	PORTB,
	PORTC,
	PORTD,
	PORTE,
	PORTF,
	PORTG
}PortId;

typedef enum
{
	PIN0,
	PIN1,
	PIN2,
	PIN3,
	PIN4,
	PIN5,
	PIN6,
	PIN7,
	PIN8,
	PIN9,
	PIN10,
	PIN11,
	PIN12,
	PIN13,
	PIN14,
	PIN15
}PinId;

typedef enum
{
	OUTPUT,
	INPUT,
	ANALOG,
	ALTERNATE
}PinMode;

typedef enum
{
	LOW = 0,
	HIGH
}PinState;

typedef struct
{
	PinId pin_id;
	PortId port_id;
	PinMode pin_mode;
}PinType;

PinHandle InitPin(PortId port_id, PinId pin_id, PinMode pin_mode);
void SetPinState(PinHandle handle, PinState expected_state);
void TogglePinState(PinHandle handle);
PinState GetPinState(PinHandle handle);



#endif // GPIO_H

