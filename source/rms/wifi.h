#ifndef WIFI_INCLUDE_H
#define WIFI_INCLUDE_H

#include "usart.h"


typedef union {
	struct {
		union {
			struct {
				uint16_t cur1;
				uint16_t cur2;
				uint16_t vol1;
			};
		};
	};
	uint8_t data[30];
} ReceiveData;

typedef void (*WifiCallBack)( char *msg, int status, int length);

void WifiInit(WifiCallBack callback);
void Sendforaccess(uint32_t cardnumber);

#endif  //RMS_INCLUDE_H
