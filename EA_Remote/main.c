#include "reg52.h"
#include "stdio.h"
#include "nrf24l01.h"
#include "SI24R1.h"
#include "key.h"

#define uchar unsigned char
#define uint unsigned int 

void uart_cfg();
void send_byte(uchar by);
void send_string(uchar *p);
void delayms(uchar i);

uchar str[] = {"aaaaaaa"};

void main()
{
	uart_cfg(); //������4800    ,0xf9
//	NRF24L01Int();
//	while(NRF24L01_Check())
//		send_string("Error\n");
//	send_string("Successed!\n");
//	NRFSetRXMode();
	SI24R1_Init();
	SI24R1_TX_Mode();
	while(1){
//		if(NRFRevDate(str)) {
//			send_string(str);
//		}			
//		NRFSetTxMode(str);
//		NRFDelay(1000);
//		if(!SI24R1_RxPacket(str)) {
//			send_string(str);
//		}
		SI24R1_TxPacket("a");
	}
}

 void uart_cfg()
{
	SCON = 0X50;//MODE 1
	TMOD&= 0X0F;//���T1�Ŀ���λ
	TMOD|= 0X20;//T1�Ĺ���ģʽ2
	PCON|= 0X80;//��Ƶ

	TH1 = 0xf3; //4800
	TL1 = TH1;

	ET1 = 0;//��ֹT1�ж� 
	//      EA = 1; 
	TR1 = 1;      
	//      ES = 1;//ʹ�ܴ����ж� ,������TI/RI���֣�ֻҪ�жϴ򿪣���Ƭ���ͽ����жϺ�����
}

 /*�жϴ�����*/
// void uart_interrupt() interrupt 4{
//
//        if(RI==1)RI = 0;
//
//        if(TI==1);
//               //TI = 0;
//    }

 
/*����һ�� �ַ�*/
void send_byte(uchar by)
{
	SBUF = by;
	while(!TI);//��д������ʱ�򣬾Ͳ�Ҫ���жϺ���������дTI = 0;����ˣ���Ȼ�����жϺ�����TI����֮�󣬳���ͻ�һֱ��������
	TI = 0;       //�����ｫTI����
}
/*����һ���ַ���*/
void send_string(uchar *p)
{
	while(*p!= '\0'){
		send_byte(*p);
		p++;
	}
}
/*����ʱ*/
void delayms(uchar i)
{
	uint j;
	while(i--)
	{
		for(j = 0; j < 150; j++);
	}
}