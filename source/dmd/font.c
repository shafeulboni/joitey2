



#include <stddef.h>
#include "font.h"
#include "font_7.h"
#include "font_8.h"
#include "font_9.h"
#include "font_10.h"
#include "font_11.h"
#include "font_12.h"
#include "font_13.h"
#include "font_15.h"

char * arial[] = {};


const uint8_t * const * FindFont(int font_size)
{
	const uint8_t * const * selected_font = font_15;
	switch (font_size) {

		case 7:
			selected_font = font_7;
			break;
		case 8:
			selected_font = font_8;
			break;
		case 9:
			selected_font = font_9;
			break;
		case 10:
			selected_font = font_10;
			break;
		case 11:
			selected_font = font_11;
			break;
		case 12:
			selected_font = font_12;
			break;
		case 13:
		case 14:
			selected_font = font_13;
			break;
		case 15:
		case 16:
			selected_font = font_15;
			break;
		default:
			break;
	}
	return selected_font;
}
