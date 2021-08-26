/*****************************************
���ܣ�NRF24L01��Ƶģ��C�ļ�(�ڵ㷢�䲿��)
*****************************************/
#include"reg52.h"
#include"nrf24l01.h"
sbit CE=P1^5;  //RX0/TXģʽѡ���
sbit CSN=P1^4; //SPIƬѡ��//����SS
sbit SCLK=P1^3;    //SPIʱ�Ӷ�
sbit MOSI=P1^2;    //SPI��������ӻ������
sbit MISO=P1^1;    //SPI��������ӻ������
sbit IRQ=P1^0; //�������ж϶�
unchar code TxAddr[]={0x34,0x43,0x10,0x10,0x01};//���͵�ַ
/*****************״̬��־*****************************************/
unchar bdata sta;   //״̬��־
sbit RX_DR=sta^6;
sbit TX_DS=sta^5;
sbit MAX_RT=sta^4;
/*****************SPIʱ����******************************************/
unchar NRFSPI(unchar date)
{
    unchar i;
       for(i=0;i<8;i++)          // ѭ��8��
       {
      if(date&0x80)
        MOSI=1;
      else
        MOSI=0;   // byte���λ�����MOSI
         date<<=1;             // ��һλ��λ�����λ
         SCLK=1;
      if(MISO)               // ����SCK��nRF24L01��MOSI����1λ���ݣ�ͬʱ��MISO���1λ����
           date|=0x01;           // ��MISO��byte���λ
         SCLK=0;                // SCK�õ�
       }
    return(date);               // ���ض�����һ�ֽ�
}
/**********************��ʱ����**************************/
void NRFDelay(unchar t)
{
   unchar x,y;
   for(x=t;x>0;x--)
   {
    for(y=110;y>0;y--);
   }
}
/**********************NRF24L01��ʼ������*******************************/
void NRF24L01Int()
{
    NRFDelay(2);//��ϵͳʲô������
    CE=0; //����ģʽ1
    CSN=1;
    SCLK=0;
    IRQ=1;
}
/*****************SPI���Ĵ���һ�ֽں���*********************************/
unchar NRFReadReg(unchar RegAddr)
{
   unchar BackDate;
   CSN=0;//����ʱ��
   NRFSPI(RegAddr);//д�Ĵ�����ַ
   BackDate=NRFSPI(0x00);//д����Ĵ���ָ��
   CSN=1;
   return(BackDate); //����״̬
}
/*****************SPIд�Ĵ���һ�ֽں���*********************************/
unchar NRFWriteReg(unchar RegAddr,unchar date)
{
   unchar BackDate;
   CSN=0;//����ʱ��
   BackDate=NRFSPI(RegAddr);//д���ַ
   NRFSPI(date);//д��ֵ
   CSN=1;
   return(BackDate);
}
/*****************SPI��ȡRXFIFO�Ĵ�����ֵ********************************/
unchar NRFReadRxDate(unchar RegAddr,unchar *RxDate,unchar DateLen)
{  //�Ĵ�����ַ//��ȡ���ݴ�ű���//��ȡ���ݳ���//���ڽ���
    unchar BackDate,i;
    CSN=0;//����ʱ��
    BackDate=NRFSPI(RegAddr);//д��Ҫ��ȡ�ļĴ�����ַ
    for(i=0;i<DateLen;i++) //��ȡ����
      {
         RxDate[i]=NRFSPI(0);
      }
    CSN=1;
   return(BackDate);
}
/*****************SPIд��TXFIFO�Ĵ�����ֵ**********************************/
unchar NRFWriteTxDate(unchar RegAddr,unchar *TxDate,unchar DateLen)
{ //�Ĵ�����ַ//д�����ݴ�ű���//��ȡ���ݳ���//���ڷ���
   unchar BackDate,i;
   CSN=0;
   BackDate=NRFSPI(RegAddr);//д��Ҫд��Ĵ����ĵ�ַ
   for(i=0;i<DateLen;i++)//д������
     {
        NRFSPI(*TxDate++);
     }
   CSN=1;
   return(BackDate);
}
/*****************NRF����Ϊ����ģʽ����������******************************/
void NRFSetTxMode(unchar *TxDate)
{//����ģʽ
    CE=0;
		NRFWriteTxDate(W_REGISTER+TX_ADDR,TxAddr,TX_ADDR_WITDH);//д�Ĵ���ָ��+���յ�ַʹ��ָ��+���յ�ַ+��ַ���
		NRFWriteTxDate(W_REGISTER+RX_ADDR_P0,TxAddr,TX_ADDR_WITDH);//Ϊ��Ӧ������豸������ͨ��0��ַ�ͷ��͵�ַ��ͬ
    NRFWriteTxDate(W_TX_PAYLOAD,TxDate,TX_DATA_WITDH);//д������
    /******�����йؼĴ�������**************/
		NRFWriteReg(W_REGISTER+EN_AA,0x01);       // ʹ�ܽ���ͨ��0�Զ�Ӧ��
		NRFWriteReg(W_REGISTER+EN_RXADDR,0x01);   // ʹ�ܽ���ͨ��0
		NRFWriteReg(W_REGISTER+SETUP_RETR,0x0a);  // �Զ��ط���ʱ�ȴ�250us+86us���Զ��ط�10��
		NRFWriteReg(W_REGISTER+RF_CH,0x40);         // ѡ����Ƶͨ��0x40
		NRFWriteReg(W_REGISTER+RF_SETUP,0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
		NRFWriteReg(W_REGISTER+CONFIG,0x0e);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�
    CE=1;
    NRFDelay(10);//����10us������
}
/*****************NRF����Ϊ����ģʽ����������******************************/
//��Ҫ����ģʽ
void NRFSetRXMode()
{
    CE=0;
      NRFWriteTxDate(W_REGISTER+RX_ADDR_P0,TxAddr,TX_ADDR_WITDH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
      NRFWriteReg(W_REGISTER+EN_AA,0x01);               // ʹ�ܽ���ͨ��0�Զ�Ӧ��
      NRFWriteReg(W_REGISTER+EN_RXADDR,0x01);           // ʹ�ܽ���ͨ��0
      NRFWriteReg(W_REGISTER+RF_CH,0x40);                 // ѡ����Ƶͨ��0x40
      NRFWriteReg(W_REGISTER+RX_PW_P0,TX_DATA_WITDH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
      NRFWriteReg(W_REGISTER+RF_SETUP,0x07);            // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������*/
      NRFWriteReg(W_REGISTER+CONFIG,0x0f);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
      CE = 1;
    NRFDelay(10);//����10us������
}
/****************************���Ӧ���ź�******************************/
unchar CheckACK()
{  //���ڷ���
    sta=NRFReadReg(R_REGISTER+STATUS);                    // ����״̬�Ĵ���
    if(TX_DS||MAX_RT) //��������ж�
    {
       NRFWriteReg(W_REGISTER+STATUS,0xff);  // ���TX_DS��MAX_RT�жϱ�־
       CSN=0;
       NRFSPI(FLUSH_TX);//�������FIFO �����ؼ�������Ȼ��������벻���ĺ����������Ҽ�ס����
       CSN=1;
       return(0);
    }
    else
       return(1);
}
/******************�ж��Ƿ�����յ����ݣ��ӵ��ʹ�RXȡ��*********************/
//���ڽ���ģʽ
unchar NRFRevDate(unchar *RevDate)
{
     unchar RevFlags=0;
     sta=NRFReadReg(R_REGISTER+STATUS);//�������ݺ��ȡ״̬�Ĵ���
     if(RX_DR)                // �ж��Ƿ���յ�����
     {
        CE=0;             //SPIʹ��
        NRFReadRxDate(R_RX_PAYLOAD,RevDate,RX_DATA_WITDH);// ��RXFIFO��ȡ����
        RevFlags=1;       //��ȡ������ɱ�־
      }
     NRFWriteReg(W_REGISTER+STATUS,0xff); //���յ����ݺ�RX_DR,TX_DS,MAX_PT���ø�Ϊ1��ͨ��д1������жϱ�
     return(RevFlags);
}

unchar NRF24L01_Check(void)
{
	unchar buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	unchar i;

	NRFWriteTxDate(W_REGISTER+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRFReadRxDate(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//���24L01����	
	return 0;		 	//��⵽24L01
}	