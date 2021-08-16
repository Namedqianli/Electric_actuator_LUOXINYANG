/**
  ******************************************************************************
  * �ļ�����: bsp_EncoderTIM.c
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-6-06
  * ��    ��: ������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  *
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "encoder/bsp_encoder.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
int32_t ENCODER1_OverflowCNT = 0;//��ʱ���������
int32_t ENCODER2_OverflowCNT = 0;//��ʱ���������
/* Timer handler declaration */
TIM_HandleTypeDef    htimx_Encoder1;
TIM_HandleTypeDef    htimx_Encoder2;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ͨ�ö�ʱ����ʼ��������ͨ��PWM���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void ENCODER1_TIMx_Init(void)
{
  /* Timer Encoder Configuration Structure declaration */
  TIM_Encoder_InitTypeDef sEncoderConfig;

  ENCODER1_TIM_RCC_CLK_ENABLE();
  htimx_Encoder1.Instance = ENCODER1_TIMx;
  htimx_Encoder1.Init.Prescaler = ENCODER1_TIM_PRESCALER;
  htimx_Encoder1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx_Encoder1.Init.Period = ENCODER1_TIM_PERIOD;
  htimx_Encoder1.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;

  sEncoderConfig.EncoderMode        = TIM_ENCODER1MODE_TIx;
  sEncoderConfig.IC1Polarity        = TIM_ICPOLARITY_RISING;
  sEncoderConfig.IC1Selection       = TIM_ICSELECTION_DIRECTTI;
  sEncoderConfig.IC1Prescaler       = TIM_ICPSC_DIV1;
  sEncoderConfig.IC1Filter          = 0;

  sEncoderConfig.IC2Polarity        = TIM_ICPOLARITY_RISING;
  sEncoderConfig.IC2Selection       = TIM_ICSELECTION_DIRECTTI;
  sEncoderConfig.IC2Prescaler       = TIM_ICPSC_DIV1;
  sEncoderConfig.IC2Filter          = 0;
  __HAL_TIM_SET_COUNTER(&htimx_Encoder1,0);

  HAL_TIM_Encoder_Init(&htimx_Encoder1, &sEncoderConfig);

  __HAL_TIM_CLEAR_IT(&htimx_Encoder1, TIM_IT_UPDATE);  // ��������жϱ�־λ
  __HAL_TIM_URS_ENABLE(&htimx_Encoder1);               // ���������������Ų��������ж�
  __HAL_TIM_ENABLE_IT(&htimx_Encoder1,TIM_IT_UPDATE);  // ʹ�ܸ����ж�

  HAL_NVIC_SetPriority(ENCODER1_TIM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ENCODER1_TIM_IRQn);

  HAL_TIM_Encoder_Start(&htimx_Encoder1, TIM_CHANNEL_ALL);
}

/**
  * ��������: ͨ�ö�ʱ����ʼ��������ͨ��PWM���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void ENCODER2_TIMx_Init(void)
{
  /* Timer Encoder Configuration Structure declaration */
  TIM_Encoder_InitTypeDef sEncoderConfig;

  ENCODER2_TIM_RCC_CLK_ENABLE();
  htimx_Encoder2.Instance = ENCODER2_TIMx;
  htimx_Encoder2.Init.Prescaler = ENCODER2_TIM_PRESCALER;
  htimx_Encoder2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx_Encoder2.Init.Period = ENCODER2_TIM_PERIOD;
  htimx_Encoder2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;

  sEncoderConfig.EncoderMode        = TIM_ENCODER2MODE_TIx;
  sEncoderConfig.IC1Polarity        = TIM_ICPOLARITY_RISING;
  sEncoderConfig.IC1Selection       = TIM_ICSELECTION_DIRECTTI;
  sEncoderConfig.IC1Prescaler       = TIM_ICPSC_DIV1;
  sEncoderConfig.IC1Filter          = 0;

  sEncoderConfig.IC2Polarity        = TIM_ICPOLARITY_RISING;
  sEncoderConfig.IC2Selection       = TIM_ICSELECTION_DIRECTTI;
  sEncoderConfig.IC2Prescaler       = TIM_ICPSC_DIV1;
  sEncoderConfig.IC2Filter          = 0;
  __HAL_TIM_SET_COUNTER(&htimx_Encoder2,0);

  HAL_TIM_Encoder_Init(&htimx_Encoder2, &sEncoderConfig);

  __HAL_TIM_CLEAR_IT(&htimx_Encoder2, TIM_IT_UPDATE);  // ��������жϱ�־λ
  __HAL_TIM_URS_ENABLE(&htimx_Encoder2);               // ���������������Ų��������ж�
  __HAL_TIM_ENABLE_IT(&htimx_Encoder2,TIM_IT_UPDATE);  // ʹ�ܸ����ж�

  HAL_NVIC_SetPriority(ENCODER2_TIM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ENCODER2_TIM_IRQn);

  HAL_TIM_Encoder_Start(&htimx_Encoder2, TIM_CHANNEL_ALL);
}

/**
  * ��������: ������ʱ��Ӳ����ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base == &htimx_Encoder1)
  {
    /* ������ʱ������ʱ��ʹ�� */
    ENCODER1_TIM_GPIO_CLK_ENABLE();

    /* ��ʱ��ͨ��1��������IO��ʼ�� */
    GPIO_InitStruct.Pin = ENCODER1_TIM_CH1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull=GPIO_PULLUP;
    GPIO_InitStruct.Alternate = ENCODER1_GPIO_AFx_TIMx;
    HAL_GPIO_Init(ENCODER1_TIM_CH1_GPIO, &GPIO_InitStruct);

    /* ��ʱ��ͨ��2��������IO��ʼ�� */
    GPIO_InitStruct.Pin = ENCODER1_TIM_CH2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = ENCODER1_GPIO_AFx_TIMx;
    HAL_GPIO_Init(ENCODER1_TIM_CH2_GPIO, &GPIO_InitStruct);
  }
  if(htim_base == &htimx_Encoder2)
  {
    /* ������ʱ������ʱ��ʹ�� */
    ENCODER2_TIM_GPIO_CLK_ENABLE();

    /* ��ʱ��ͨ��1��������IO��ʼ�� */
    GPIO_InitStruct.Pin = ENCODER2_TIM_CH1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull=GPIO_PULLUP;
    GPIO_InitStruct.Alternate = ENCODER2_GPIO_AFx_TIMx;
    HAL_GPIO_Init(ENCODER2_TIM_CH1_GPIO, &GPIO_InitStruct);

    /* ��ʱ��ͨ��2��������IO��ʼ�� */
    GPIO_InitStruct.Pin = ENCODER2_TIM_CH2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = ENCODER2_GPIO_AFx_TIMx;
    HAL_GPIO_Init(ENCODER2_TIM_CH2_GPIO, &GPIO_InitStruct);
  }
}

/**
  * ��������: ������ʱ��Ӳ������ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base == &htimx_Encoder1)
  {
    /* ������ʱ������ʱ�ӽ��� */
    ENCODER1_TIM_RCC_CLK_DISABLE();

    HAL_GPIO_DeInit(ENCODER1_TIM_CH1_GPIO, ENCODER1_TIM_CH1_PIN);
    HAL_GPIO_DeInit(ENCODER1_TIM_CH2_GPIO, ENCODER1_TIM_CH2_PIN);
  }
  if(htim_base == &htimx_Encoder2)
  {
    /* ������ʱ������ʱ�ӽ��� */
    ENCODER2_TIM_RCC_CLK_DISABLE();

    HAL_GPIO_DeInit(ENCODER2_TIM_CH1_GPIO, ENCODER2_TIM_CH1_PIN);
    HAL_GPIO_DeInit(ENCODER2_TIM_CH2_GPIO, ENCODER2_TIM_CH2_PIN);
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htimx_Encoder1)
  {
    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htimx_Encoder1))
      ENCODER1_OverflowCNT--;       //���¼������
    else
      ENCODER1_OverflowCNT++;       //���ϼ������
  }
  if(htim == &htimx_Encoder2)
  {
    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htimx_Encoder2))
      ENCODER2_OverflowCNT--;       //���¼������
    else
      ENCODER2_OverflowCNT++;       //���ϼ������
  }
}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
