#include <reg51.h>
#include "nrf24l01.h"
#include "key.h"

int main(void)
{
	uchar key = 0;
	//��ʼ��nrf
	init_io();
	
	while(1) {
		key = key_scan(0);
		TX_Mode(&key);
	}
	
	return 0;
}
