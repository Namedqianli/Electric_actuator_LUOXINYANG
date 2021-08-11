/**
  ******************************************************************************
  * �ļ�����: bsp_usartx.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: ���ش��ڵײ���������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "usart/bsp_usartx.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
UART_HandleTypeDef husartx;
__IO uint8_t RxBuf[FRAME_LENTH] ; // ���ջ�����
__IO uint8_t TxBuf[FRAME_LENTH] ; // ���ͻ�����
void (*ptr_Fun_)(void) = NULL ;//����ָ��  ptr_Fun_ = Analyse_Data_Callback;
/* ��չ���� ------------------------------------------------------------------*/
MSG_TypeDef Msg;
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����Ӳ����ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart == &husartx)
  {
    /* ʹ�ܴ��ڹ�������GPIOʱ�� */
    USARTx_GPIO_ClK_ENABLE();
    /* �������蹦��GPIO���� */
    GPIO_InitStruct.Pin = USARTx_Tx_GPIO_PIN|USARTx_Rx_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = USARTx_AFx;
    HAL_GPIO_Init(USARTx_Tx_GPIO, &GPIO_InitStruct);
#ifdef _RS485_
    /* 485ʹ������ */
    RS485_REDE_GPIO_ClK_ENABLE();
    
    GPIO_InitStruct.Pin = RS485_REDE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(RS485_REDE_PORT, &GPIO_InitStruct);
#endif
  }
}

/**
  * ��������: ����Ӳ������ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart == &husartx)
  {
    /* ��������ʱ�ӽ��� */
    USART_RCC_CLK_DISABLE();
  
    /* �������蹦��GPIO���� */
    HAL_GPIO_DeInit(USARTx_Tx_GPIO, USARTx_Tx_GPIO_PIN);
    HAL_GPIO_DeInit(USARTx_Rx_GPIO, USARTx_Rx_GPIO_PIN);
    
    /* �����жϽ��� */
    HAL_NVIC_DisableIRQ(USARTx_IRQn);
  }
}

/**
  * ��������: NVIC����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void MX_NVIC_USARTx_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USARTx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

/**
  * ��������: ���ڲ�������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void MX_USARTx_Init(void)
{
  /* ��������ʱ��ʹ�� */
  USART_RCC_CLK_ENABLE();
  
  husartx.Instance = USARTx;
  husartx.Init.BaudRate = USARTx_BAUDRATE;
  husartx.Init.WordLength = UART_WORDLENGTH_8B;
  husartx.Init.StopBits = UART_STOPBITS_1;
  husartx.Init.Parity = UART_PARITY_NONE;
  husartx.Init.Mode = UART_MODE_TX_RX;
  husartx.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husartx.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husartx);
  
  /* ���ô����жϲ�ʹ�ܣ���Ҫ����HAL_UART_Init������ִ���޸Ĳ���Ч */
  HAL_UART_Receive_IT(&husartx,(uint8_t *)&RxBuf,FRAME_LENTH); // ����ʹ�ܽ����ж�
  MX_NVIC_USARTx_Init();
}

/**
  * ��������: �ض���c�⺯��printf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fputc(int ch, FILE *f)
{
#ifdef _RS485_
  RS485_TX_MODE();
#endif 
  HAL_UART_Transmit(&husartx, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/**
  * ��������: �ض���c�⺯��getchar,scanf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fgetc(FILE * f)
{
  uint8_t ch = 0;
#ifdef _RS485_
  RS485_RX_MODE();
#endif 
  while(HAL_UART_Receive(&husartx,&ch, 1, 0xffff)!=HAL_OK);
  return ch;
}

/**
  * ��������: ��������
  * �������: Ptr:������У��͵�������ʼ��ַ , Num:��������ֽ���
  * �� �� ֵ: ����õ���У���
  * ˵    ��: ��������
  */
uint8_t CheckSum(uint8_t *Ptr,uint8_t Num )
{
  uint8_t Sum = 0;
  while(Num--)
  {
    Sum += *Ptr;
    Ptr++;
  }
  return Sum;
}
/**
  * ��������: �����ж���Ӧ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��Ӧ����ȫ���ж�����
  */
void USARTx_IRQHANDLER()
{
  HAL_UART_IRQHandler(&husartx);
}
/**
  * ��������: ���ڽ����жϻص�����
  * �������: huart�����ھ��ָ��
  * �� �� ֵ: ��
  * ˵    ��: ���̶��ֳ���������֡,ͬʱ��֤֡ͷ֡β��У����
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &husartx)
  {
    if(RxBuf[0] != FRAME_START )    // ֡ͷ��ȷ
    {
      return;
    }
    if(RxBuf[FRAME_LENTH-1] == FRAME_END ) // ֡β��ȷ
    { 
      /* �ж�У���� */
      if(CheckSum((uint8_t*)&RxBuf[FRAME_CHECK_BEGIN],FRAME_CHECK_NUM) != RxBuf[FRAME_CHECKSUM] )
      {
        Msg.Code = NULL;
        return;
      }
      else
      {
        /* ��������֡ */
        
        if(ptr_Fun_ == NULL)
          return ;
        else 
          ptr_Fun_();
      }
    }
    HAL_UART_Receive_IT(huart,(uint8_t *)&RxBuf,FRAME_LENTH); // ����ʹ�ܽ����ж�
  }
}
/**
  * ��������: ���ͷ���ֵ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ������ֵ���͵�����
  */
void Transmit_FB( __IO int32_t *Feedback)
{
  uint8_t i = 0;
  for(i=0;i<FRAME_LENTH;i++)
  {
    TxBuf[i] = FILL_VALUE;  // ������� 0x55
  }
  
  Msg.data[0].Int = *Feedback;//����ֵ �ٶ�
  
  TxBuf[0] = FRAME_START;   // ֡ͷ
  TxBuf[1] = 0x80|CODE_SETTGT; // ָ����
  TxBuf[2] = Msg.data[0].Ch[0];
  TxBuf[3] = Msg.data[0].Ch[1];
  TxBuf[4] = Msg.data[0].Ch[2];
  TxBuf[5] = Msg.data[0].Ch[3];
  
  TxBuf[FRAME_CHECKSUM] = CheckSum((uint8_t*)&TxBuf[FRAME_CHECK_BEGIN],FRAME_CHECK_NUM);  // ����У���
  TxBuf[FRAME_LENTH-1] = FRAME_END;   // ����֡β
#ifdef _RS485_
  RS485_TX_MODE();
#endif 
  HAL_UART_Transmit_IT(&husartx,(uint8_t *)&TxBuf,FRAME_LENTH); // ��������֡
}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
