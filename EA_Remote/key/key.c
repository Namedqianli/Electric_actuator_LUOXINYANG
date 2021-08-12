#include "key.h"

sbit KEY1 = P2^0;
sbit KEY2 = P2^1;
sbit KEY3 = P2^2;
sbit KEY4 = P2^3;
sbit KEY5 = P2^4;
sbit KEY6 = P2^5;
sbit KEY7 = P2^6;
sbit KEY8 = P2^7;
sbit KEY9 = P1^6;
sbit KEY10 = P1^7;

uint8_t key_scan(uint8_t mode)
{
	static uchar key_up = 1;
	
	if(mode == 1) key_up = 1;
	
}
