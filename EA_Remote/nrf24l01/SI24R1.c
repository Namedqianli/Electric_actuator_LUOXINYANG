#include "SI24R1.h"

sbit CE=P1^5;  //RX0/TXģʽѡ���
sbit CSN=P1^4; //SPIƬѡ��//����SS
sbit SCK=P1^3;    //SPIʱ�Ӷ�
sbit MOSI=P1^2;    //SPI��������ӻ������
sbit MISO=P1^1;    //SPI��������ӻ������
sbit IRQ=P1^0; //�������ж϶�

u8 code TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};  // ����һ����̬���͵�ַ

static u8 SPI_RW(u8 byte)
{
	u8 bit_ctr;
	for(bit_ctr=0; bit_ctr<8; bit_ctr++)
	{
		if(byte & 0x80)
			MOSI = 1;
		else
			MOSI = 0;																 
		byte = (byte << 1);                      
		SCK = 1;                                   
		byte |= MISO;                             
		SCK = 0;                                
	}
	return(byte);                              
}


/********************************************************
�������ܣ�SI24R1���ų�ʼ��                
��ڲ�������
����  ֵ����
*********************************************************/
void SI24R1_Init(void)
{
	SCK = 0; 													//SPIʱ��������
	CSN = 1;				
	CE 	= 0;				
	IRQ = 1;
}


/********************************************************
�������ܣ�д�Ĵ�����ֵ�����ֽڣ�                
��ڲ�����reg:�Ĵ���ӳ���ַ����ʽ��WRITE_REG��reg��
					value:�Ĵ�����ֵ
����  ֵ��״̬�Ĵ�����ֵ
*********************************************************/
u8 SI24R1_Write_Reg(u8 reg, u8 value)
{
	u8 status;

	CSN = 0;                 
	status = SPI_RW(reg);				
	SPI_RW(value);
	CSN = 1;  
	
	return(status);
}


/********************************************************
�������ܣ�д�Ĵ�����ֵ�����ֽڣ�                  
��ڲ�����reg:�Ĵ���ӳ���ַ����ʽ��WRITE_REG��reg��
					pBuf:д�����׵�ַ
					bytes:д�����ֽ���
����  ֵ��״̬�Ĵ�����ֵ
*********************************************************/
u8 SI24R1_Write_Buf(u8 reg, const u8 *pBuf, u8 bytes)
{
	u8 status,byte_ctr;

  CSN = 0;                                  			
  status = SPI_RW(reg);                          
  for(byte_ctr=0; byte_ctr<bytes; byte_ctr++)     
    SPI_RW(*pBuf++);
  CSN = 1;                                      	

  return(status);       
}							  					   


/********************************************************
�������ܣ���ȡ�Ĵ�����ֵ�����ֽڣ�                  
��ڲ�����reg:�Ĵ���ӳ���ַ����ʽ��READ_REG��reg��
����  ֵ���Ĵ���ֵ
*********************************************************/
u8 SI24R1_Read_Reg(u8 reg)
{
 	u8 value;

	CSN = 0;    
	SPI_RW(reg);			
	value = SPI_RW(0);
	CSN = 1;              

	return(value);
}


/********************************************************
�������ܣ���ȡ�Ĵ�����ֵ�����ֽڣ�                  
��ڲ�����reg:�Ĵ���ӳ���ַ��READ_REG��reg��
					pBuf:���ջ��������׵�ַ
					bytes:��ȡ�ֽ���
����  ֵ��״̬�Ĵ�����ֵ
*********************************************************/
u8 SI24R1_Read_Buf(u8 reg, u8 *pBuf, u8 bytes)
{
	u8 status,byte_ctr;

  CSN = 0;                                        
  status = SPI_RW(reg);                           
  for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
    pBuf[byte_ctr] = SPI_RW(0);                   //��ȡ���ݣ����ֽ���ǰ
  CSN = 1;                                        

  return(status);    
}


/********************************************************
�������ܣ�SI24R1����ģʽ��ʼ��                      
��ڲ�������
����  ֵ����
*********************************************************/
void SI24R1_RX_Mode(void)
{
	CE = 0;
	SI24R1_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);	// �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
	SI24R1_Write_Reg(WRITE_REG + EN_AA, 0x01);               						// ʹ�ܽ���ͨ��0�Զ�Ӧ��
	SI24R1_Write_Reg(WRITE_REG + EN_RXADDR, 0x01);           						// ʹ�ܽ���ͨ��0
	SI24R1_Write_Reg(WRITE_REG + RF_CH, 40);                 						// ѡ����Ƶͨ��0x40
	SI24R1_Write_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);  						// ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
	SI24R1_Write_Reg(WRITE_REG + RF_SETUP, 0x0f);            						// ���ݴ�����2Mbps�����书��7dBm
	SI24R1_Write_Reg(WRITE_REG + CONFIG, 0x0f);              						// CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
	SI24R1_Write_Reg(WRITE_REG + STATUS, 0xff);  												//������е��жϱ�־λ
	CE = 1;                                            									// ����CE���������豸
}						


/********************************************************
�������ܣ�SI24R1����ģʽ��ʼ��                      
��ڲ�������
����  ֵ����
*********************************************************/
void SI24R1_TX_Mode(void)
{
	CE = 0;
	SI24R1_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // д�뷢�͵�ַ
	SI24R1_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // Ϊ��Ӧ������豸������ͨ��0��ַ�ͷ��͵�ַ��ͬ

	SI24R1_Write_Reg(WRITE_REG + EN_AA, 0x01);       											// ʹ�ܽ���ͨ��0�Զ�Ӧ��
	SI24R1_Write_Reg(WRITE_REG + EN_RXADDR, 0x01);   											// ʹ�ܽ���ͨ��0
	SI24R1_Write_Reg(WRITE_REG + SETUP_RETR, 0x0a);  											// �Զ��ط���ʱ�ȴ�250us+86us���Զ��ط�10��
	SI24R1_Write_Reg(WRITE_REG + RF_CH, 40);         											// ѡ����Ƶͨ��0x40
	SI24R1_Write_Reg(WRITE_REG + RF_SETUP, 0x0f);    											// ���ݴ�����2Mbps�����书��7dBm
	SI24R1_Write_Reg(WRITE_REG + CONFIG, 0x0e);      											// CRCʹ�ܣ�16λCRCУ�飬�ϵ�
	//CE = 1;
}


/********************************************************
�������ܣ���ȡ��������                       
��ڲ�����rxbuf:�������ݴ���׵�ַ
����  ֵ��0:���յ�����
          1:û�н��յ�����
*********************************************************/
u8 SI24R1_RxPacket(u8 *rxbuf)
{
	u8 state;
	state = SI24R1_Read_Reg(STATUS);  			                 //��ȡ״̬�Ĵ�����ֵ    	  
	SI24R1_Write_Reg(WRITE_REG+STATUS,state);               //���RX_DS�жϱ�־

	if(state & RX_DR)								                           //���յ�����
	{
		SI24R1_Read_Buf(RD_RX_PLOAD,rxbuf,TX_PLOAD_WIDTH);     //��ȡ����
		SI24R1_Write_Reg(FLUSH_RX,0xff);					              //���RX FIFO�Ĵ���
		return 0; 
	}	   
	return 1;                                                   //û�յ��κ�����
}


/********************************************************
�������ܣ�����һ�����ݰ�                      
��ڲ�����txbuf:Ҫ���͵�����
����  ֵ��0x10:�ﵽ����ط�����������ʧ�� 
          0x20:���ͳɹ�            
          0xff:����ʧ��                  
*********************************************************/
u8 SI24R1_TxPacket(u8 *txbuf)
{
	u8 state;
	CE=0;																										  //CE���ͣ�ʹ��SI24R1����
  SI24R1_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);	    //д���ݵ�TX FIFO,32���ֽ�
 	CE=1;																										  //CE�øߣ�ʹ�ܷ���	   
	
	while(IRQ == 1);																				  //�ȴ��������
	state = SI24R1_Read_Reg(STATUS);  											  //��ȡ״̬�Ĵ�����ֵ	   
	SI24R1_Write_Reg(WRITE_REG+STATUS, state); 								//���TX_DS��MAX_RT�жϱ�־
	if(state&MAX_RT)																			    //�ﵽ����ط�����
	{
		SI24R1_Write_Reg(FLUSH_TX,0xff);										    //���TX FIFO�Ĵ��� 
		return MAX_RT; 
	}
	if(state&TX_DS)																			      //�������
	{
		return TX_DS;
	}
	return 0XFF;																						  //����ʧ��
}

