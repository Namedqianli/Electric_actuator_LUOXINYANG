#ifndef _KEY_H_
#define _KEY_H_

#include <reg51.h>

#define uint8_t unsigned char
#define uchar unsigned char

typedef enum
{
	KEY_BACK_UP				= 1,
	KEY_BACK_DOWN			= 2,
	KEY_THIGH_UP			= 3,
	KEY_THIGH_DOWN		= 4,
	KEY_CHAIR					= 5,
	KEY_BED						= 6,
}action_t;

uint8_t key_scan(uint8_t mode);

#endif
