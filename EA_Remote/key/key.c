#include "key.h"

typedef enum 
{
	SIGNAL_BACK_UP				= 0x01,				//Ì§±³
	SIGNAL_BACK_DOWN			= 0x02,				//½µ±³
	SIGNAL_FOOT_UP				= 0x03,				//Ì§ÍÈ
	SIGNAL_FOOT_DOWN			= 0x04,				//½µÍÈ
	SIGNAL_ROTATE_CHAIR		= 0x05,				//²à×øÏÂ´²£¬Ðý×ª³ÉÒÎ
	SIGNAL_ROTATE_BED			= 0x06,				//ÉÏ´²Æ½ÌÉ
	SIGNAL_PART_MOVE			= 0x07,				//·ÖÀëÒÆ¶¯
	SIGNAL_RESET					= 0x08,				//Ò»¼üÆ½ÌÉ						
} receive_signal_t;

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

void key_delay(uint8_t t)
{
   uint8_t x,y;
   for(x=t;x>0;x--)
   {
    for(y=110;y>0;y--);
   }
}

uint8_t key_scan(uint8_t mode)
{
	static uchar key_up = 1;
	
	if(mode == 1) key_up = 1;
	P2 = 0xFF;
	if(key_up && P2 != 0xFF) {
		key_delay(20);
		key_up = 0;
		if(KEY1 == 0)	return SIGNAL_BACK_UP;
		if(KEY2 == 0)	return SIGNAL_BACK_DOWN;
		if(KEY3 == 0)	return SIGNAL_FOOT_UP;
		if(KEY4 == 0)	return SIGNAL_FOOT_DOWN;
		if(KEY5 == 0)	return SIGNAL_ROTATE_CHAIR;
		if(KEY6 == 0)	return SIGNAL_ROTATE_BED;
		if(KEY7 == 0)	return SIGNAL_PART_MOVE;
		if(KEY8 == 0)	return SIGNAL_RESET;
	}
	
	return 0xff;
}
