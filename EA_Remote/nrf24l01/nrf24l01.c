#include "nrf24l01.h"

uchar code TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};  // ����һ����̬���͵�ַ
uchar RX_BUF[TX_PLOAD_WIDTH];
uchar TX_BUF[TX_PLOAD_WIDTH];
 
uchar flag;
uchar DATA = 0x01;
uchar bdata sta;
sbit  RX_DR     = sta^6;
sbit  TX_DS     = sta^5;
sbit  MAX_RT		= sta^4;
 
sbit CE =  P1^5;
sbit CSN=  P1^4;
sbit SCK=  P1^3;
sbit MOSI= P1^2;
sbit MISO= P1^1;
sbit IRQ = P1^0;

//--����SPIҪʹ�õ� IO--//
sbit MOSIO = P3^4;
sbit R_CLK = P3^5;
sbit S_CLK = P3^6;

/**************************************************
����: init_io()
����:
    ��ʼ��IO
/**************************************************/
void init_io(void)
{
    CE  = 0;        // ����
    CSN = 1;        // SPI��ֹ
    SCK = 0;        // SPIʱ���õ�
    IRQ = 1;        // �жϸ�λ
//    LED = 1;        // �ر�ָʾ��
}
/**************************************************/
 
/**************************************************
������delay_ms()
������
    �ӳ�x����
/**************************************************/
void delay_ms(uchar x)
{
    uchar i, j;
    i = 0;
    for(i=0; i<x; i++)
    {
       j = 250;
       while(--j);
       j = 250;
       while(--j);
    }
}
/**************************************************/
 
/**************************************************
������SPI_RW()
������
    ����SPIЭ�飬дһ�ֽ����ݵ�nRF24L01��ͬʱ��nRF24L01
    ����һ�ֽ�
/**************************************************/
uchar SPI_RW(uchar byte)
{
    uchar i;
       for(i=0; i<8; i++)          // ѭ��8��
       {
           MOSI = (byte & 0x80);   // byte���λ�����MOSI
           byte <<= 1;             // ��һλ��λ�����λ
           SCK = 1;                // ����SCK��nRF24L01��MOSI����1λ���ݣ�ͬʱ��MISO���1λ����
           byte |= MISO;           // ��MISO��byte���λ
           SCK = 0;                // SCK�õ�
       }
    return(byte);               // ���ض�����һ�ֽ�
}
/**************************************************/
 
/**************************************************
������SPI_RW_Reg()
������
    д����value��reg�Ĵ���
/**************************************************/
uchar SPI_RW_Reg(uchar reg, uchar value)
{
    uchar status;
      CSN = 0;                   // CSN�õͣ���ʼ��������
      status = SPI_RW(reg);      // ѡ��Ĵ�����ͬʱ����״̬��
      SPI_RW(value);             // Ȼ��д���ݵ��üĴ���
      CSN = 1;                   // CSN���ߣ��������ݴ���
      return(status);            // ����״̬�Ĵ���
}
/**************************************************/
 
/**************************************************
������SPI_Read()
������
    ��reg�Ĵ�����һ�ֽ�
/**************************************************/
uchar SPI_Read(uchar reg)
{
    uchar reg_val;
      CSN = 0;                    // CSN�õͣ���ʼ��������
      SPI_RW(reg);                // ѡ��Ĵ���
      reg_val = SPI_RW(0);        // Ȼ��ӸüĴ���������
      CSN = 1;                    // CSN���ߣ��������ݴ���
      return(reg_val);            // ���ؼĴ�������
}
/**************************************************/
 
/**************************************************
������SPI_Read_Buf()
������
    ��reg�Ĵ�������bytes���ֽڣ�ͨ��������ȡ����ͨ��
    ���ݻ����/���͵�ַ
/**************************************************/
uchar SPI_Read_Buf(uchar reg, uchar * pBuf, uchar bytes)
{
    uchar status, i;
      CSN = 0;                    // CSN�õͣ���ʼ��������
      status = SPI_RW(reg);       // ѡ��Ĵ�����ͬʱ����״̬��
      for(i=0; i<bytes; i++)
        pBuf[i] = SPI_RW(0);    // ����ֽڴ�nRF24L01����
      CSN = 1;                    // CSN���ߣ��������ݴ���
      return(status);             // ����״̬�Ĵ���
}
/**************************************************/
 
/**************************************************
������SPI_Write_Buf()
������
    ��pBuf�����е�����д�뵽nRF24L01��ͨ������д�뷢
    ��ͨ�����ݻ����/���͵�ַ
/**************************************************/
uchar SPI_Write_Buf(uchar reg, uchar * pBuf, uchar bytes)
{
    uchar status, i;
      CSN = 0;                    // CSN�õͣ���ʼ��������
      status = SPI_RW(reg);       // ѡ��Ĵ�����ͬʱ����״̬��
      for(i=0; i<bytes; i++)
        SPI_RW(pBuf[i]);        // ����ֽ�д��nRF24L01
      CSN = 1;                    // CSN���ߣ��������ݴ���
      return(status);             // ����״̬�Ĵ���
}
/**************************************************/
 
/**************************************************
������RX_Mode()
������
    �����������nRF24L01Ϊ����ģʽ���ȴ����շ����豸�����ݰ�
/**************************************************/
void RX_Mode(void)
{
    CE = 0;
      SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
      SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);               // ʹ�ܽ���ͨ��0�Զ�Ӧ��
      SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);           // ʹ�ܽ���ͨ��0
      SPI_RW_Reg(WRITE_REG + RF_CH, 40);                 // ѡ����Ƶͨ��0x40
      SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
      SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);            // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
      SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
    delay_ms(150);
      CE = 1;                                            // ����CE���������豸
}
/**************************************************/
 
/**************************************************
������TX_Mode()
������
    �����������nRF24L01Ϊ����ģʽ����CE=1��������10us����
    130us���������䣬���ݷ��ͽ����󣬷���ģ���Զ�ת�����
    ģʽ�ȴ�Ӧ���źš�
/**************************************************/
void TX_Mode(uchar * BUF)
{
    CE = 0;
      SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // д�뷢�͵�ַ
      SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // Ϊ��Ӧ������豸������ͨ��0��ַ�ͷ��͵�ַ��ͬ
      SPI_Write_Buf(WR_TX_PLOAD, BUF, TX_PLOAD_WIDTH);                  // д���ݰ���TX FIFO
      SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);       // ʹ�ܽ���ͨ��0�Զ�Ӧ��
      SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);   // ʹ�ܽ���ͨ��0
      SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x0a);  // �Զ��ط���ʱ�ȴ�250us+86us���Զ��ط�10��
      SPI_RW_Reg(WRITE_REG + RF_CH, 40);         // ѡ����Ƶͨ��0x40
      SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
      SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�
    delay_ms(150);
    CE = 1;
}
/**************************************************/
 
/**************************************************
������Check_ACK()
������
    �������豸���޽��յ����ݰ����趨û���յ�Ӧ����
    ���Ƿ��ط�
/**************************************************/
uchar Check_ACK(bit clear)
{
    delay_ms(200);
    while(IRQ);
    sta = SPI_RW(NOP);                    // ����״̬�Ĵ���
    if(TX_DS)
    {
        blink(3);
    }
    //blink(5);
    if(MAX_RT)
        if(clear)                         // �Ƿ����TX FIFO��û������ڸ�λMAX_RT�жϱ�־���ط�
            SPI_RW(FLUSH_TX);
    SPI_RW_Reg(WRITE_REG + STATUS, sta);  // ���TX_DS��MAX_RT�жϱ�־
    IRQ = 1;
    if(TX_DS)
        return(0x00);
    else
        return(0xff);
}
/**************************************************/

