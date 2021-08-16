/**
  ******************************************************************************
  * �ļ�����: bsp_BDCMOTOR1.c
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.1
  * ��д����: 2018-03-05
  * ��    ��: 2����ٵ��GM37545_����������
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
#include "DCMotor/bsp_BDCMotor.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_BDCMOTOR1;
TIM_HandleTypeDef htimx_BDCMOTOR2;
__IO int32_t PWM_Duty1=500;         // ռ�ձȣ�PWM_Duty/BDCMOTOR1_TIM_PERIOD*100%
__IO int32_t PWM_Duty2=500;         // ռ�ձȣ�PWM_Duty/BDCMOTOR1_TIM_PERIOD*100%

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ������ʱ��Ӳ����ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: BDCMOTOR1���GPIO��ʼ������,�ú�����HAL���ڲ�����.
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  /* BDCMOTOR1���GPIO��ʼ������ */
  if(htim == &htimx_BDCMOTOR1)
  {
    GPIO_InitTypeDef GPIO_InitStruct;
    /* ���Ŷ˿�ʱ��ʹ�� */
    BDCMOTOR1_TIM_CH1_GPIO_CLK_ENABLE();
    BDCMOTOR1_TIM_CH1N_GPIO_CLK_ENABLE();
    BDCMOTOR1_SHUTDOWN_GPIO_CLK_ENABLE();

    /* BDCMOTOR1��������������IO��ʼ�� */
    GPIO_InitStruct.Pin = BDCMOTOR1_TIM_CH1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(BDCMOTOR1_TIM_CH1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BDCMOTOR1_TIM_CH1N_PIN;
    HAL_GPIO_Init(BDCMOTOR1_TIM_CH1N_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = BDCMOTOR1_SHUTDOWN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(BDCMOTOR1_SHUTDOWN_PORT, &GPIO_InitStruct);

    /* ʹ�ܵ���������� */
    ENABLE_MOTOR1();
  }
  /* BDCMOTOR2���GPIO��ʼ������ */
  if(htim == &htimx_BDCMOTOR2)
  {
    GPIO_InitTypeDef GPIO_InitStruct;
    /* ���Ŷ˿�ʱ��ʹ�� */
    BDCMOTOR2_TIM_CH1_GPIO_CLK_ENABLE();
    BDCMOTOR2_TIM_CH1N_GPIO_CLK_ENABLE();
    BDCMOTOR2_SHUTDOWN_GPIO_CLK_ENABLE();

    /* BDCMOTOR2��������������IO��ʼ�� */
    GPIO_InitStruct.Pin = BDCMOTOR2_TIM_CH1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(BDCMOTOR2_TIM_CH1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BDCMOTOR2_TIM_CH1N_PIN;
    HAL_GPIO_Init(BDCMOTOR2_TIM_CH1N_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = BDCMOTOR2_SHUTDOWN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(BDCMOTOR2_SHUTDOWN_PORT, &GPIO_InitStruct);

    /* ʹ�ܵ���������� */
    ENABLE_MOTOR2();
  }
}

/**
  * ��������: BDCMOTOR1��ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void BDCMOTOR1_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;                          // ��ʱ��ͨ���Ƚ����
  TIM_BreakDeadTimeConfigTypeDef  sBDTConfig;            // ��ʱ������ʱ��Ƚ����

  /* ������ʱ������ʱ��ʹ�� */
  BDCMOTOR1_TIM_RCC_CLK_ENABLE();

  /* ��ʱ�������������� */
  htimx_BDCMOTOR1.Instance = BDCMOTOR1_TIMx;                                 // ��ʱ�����
  htimx_BDCMOTOR1.Init.Prescaler = BDCMOTOR1_TIM_PRESCALER;                  // ��ʱ��Ԥ��Ƶ��
  htimx_BDCMOTOR1.Init.CounterMode = TIM_COUNTERMODE_UP;                  // �����������ϼ���
  htimx_BDCMOTOR1.Init.Period = BDCMOTOR1_TIM_PERIOD;                        // ��ʱ������
  htimx_BDCMOTOR1.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;              // ʱ�ӷ�Ƶ
  htimx_BDCMOTOR1.Init.RepetitionCounter = BDCMOTOR1_TIM_REPETITIONCOUNTER;  // �ظ�������
  /* ��ʼ����ʱ���Ƚ�������� */
  HAL_TIM_PWM_Init(&htimx_BDCMOTOR1);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htimx_BDCMOTOR1, &sClockSourceConfig);

  /* ����ɲ������,������Ч��ƽ�ǵ͵�ƽ */
  sBDTConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE ;
  sBDTConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW ;
  sBDTConfig.BreakState = TIM_BREAK_DISABLE ;
  sBDTConfig.DeadTime = 0 ;
  sBDTConfig.LockLevel = TIM_LOCKLEVEL_OFF ;
  sBDTConfig.OffStateIDLEMode= TIM_OSSI_DISABLE ;
  sBDTConfig.OffStateRunMode = TIM_OSSR_ENABLE ;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx_BDCMOTOR1,&sBDTConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;                  // �Ƚ����ģʽ��PWM1ģʽ
  sConfigOC.Pulse =  PWM_Duty1;                         // ռ�ձ�
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;          // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;        // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // ����ͨ�����е�ƽ
  HAL_TIM_PWM_ConfigChannel(&htimx_BDCMOTOR1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_Base_Start(&htimx_BDCMOTOR1);

    /* ������ʱ��ͨ���ͻ���ͨ��PWM��� */
  HAL_TIM_PWM_Start(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);

}


/**
  * ��������: ������ʱ��Ӳ������ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==BDCMOTOR1_TIMx)
  {
    /* ������ʱ������ʱ�ӽ��� */
    BDCMOTOR1_TIM_RCC_CLK_DISABLE();

    HAL_GPIO_DeInit(BDCMOTOR1_TIM_CH1_PORT,BDCMOTOR1_TIM_CH1_PIN);
    HAL_GPIO_DeInit(BDCMOTOR1_TIM_CH1N_PORT,BDCMOTOR1_TIM_CH1N_PIN);
  }
  if(htim_base->Instance==BDCMOTOR2_TIMx)
  {
    /* ������ʱ������ʱ�ӽ��� */
    BDCMOTOR2_TIM_RCC_CLK_DISABLE();

    HAL_GPIO_DeInit(BDCMOTOR2_TIM_CH1_PORT,BDCMOTOR2_TIM_CH1_PIN);
    HAL_GPIO_DeInit(BDCMOTOR2_TIM_CH1N_PORT,BDCMOTOR2_TIM_CH1N_PIN);
  }

}
/**
  * ��������: ���õ���ٶ�
  * ���뺯��: Duty,�������ռ�ձ�
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SetMotor1Speed(int16_t Duty)
{
  __HAL_TIM_SetCompare(&htimx_BDCMOTOR1,TIM_CHANNEL_1,Duty);
}

/**
  * ��������: ���õ��ת������
  * ���뺯��: Dir,���ת������
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SetMotor1Dir(int16_t Dir)
{
  if(Dir)
  {
    HAL_TIM_PWM_Start(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);         // ֹͣ���
  }
  else
  {
    HAL_TIM_PWM_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htimx_BDCMOTOR1,TIM_CHANNEL_1);         // ֹͣ���
  }
}

/* --------------------------------------------------------------------- */
/* ��ˢֱ����� */

/**
  * ��������: BDCMOTOR2��ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void BDCMOTOR2_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;                          // ��ʱ��ͨ���Ƚ����
  TIM_BreakDeadTimeConfigTypeDef  sBDTConfig;            // ��ʱ������ʱ��Ƚ����

  /* ������ʱ������ʱ��ʹ�� */
  BDCMOTOR2_TIM_RCC_CLK_ENABLE();

  /* ��ʱ�������������� */
  htimx_BDCMOTOR2.Instance = BDCMOTOR2_TIMx;                                 // ��ʱ�����
  htimx_BDCMOTOR2.Init.Prescaler = BDCMOTOR2_TIM_PRESCALER;                  // ��ʱ��Ԥ��Ƶ��
  htimx_BDCMOTOR2.Init.CounterMode = TIM_COUNTERMODE_UP;                  // �����������ϼ���
  htimx_BDCMOTOR2.Init.Period = BDCMOTOR2_TIM_PERIOD;                        // ��ʱ������
  htimx_BDCMOTOR2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;              // ʱ�ӷ�Ƶ
  htimx_BDCMOTOR2.Init.RepetitionCounter = BDCMOTOR2_TIM_REPETITIONCOUNTER;  // �ظ�������
  /* ��ʼ����ʱ���Ƚ�������� */
  HAL_TIM_PWM_Init(&htimx_BDCMOTOR2);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htimx_BDCMOTOR2, &sClockSourceConfig);

  /* ����ɲ������,������Ч��ƽ�ǵ͵�ƽ */
  sBDTConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE ;
  sBDTConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW ;
  sBDTConfig.BreakState = TIM_BREAK_DISABLE ;
  sBDTConfig.DeadTime = 0 ;
  sBDTConfig.LockLevel = TIM_LOCKLEVEL_OFF ;
  sBDTConfig.OffStateIDLEMode= TIM_OSSI_DISABLE ;
  sBDTConfig.OffStateRunMode = TIM_OSSR_ENABLE ;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx_BDCMOTOR2,&sBDTConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;                  // �Ƚ����ģʽ��PWM1ģʽ
  sConfigOC.Pulse =  PWM_Duty2;                         // ռ�ձ�
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;          // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;        // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // ����ͨ�����е�ƽ
  HAL_TIM_PWM_ConfigChannel(&htimx_BDCMOTOR2, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_Base_Start(&htimx_BDCMOTOR2);

    /* ������ʱ��ͨ���ͻ���ͨ��PWM��� */
  HAL_TIM_PWM_Start(&htimx_BDCMOTOR2,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR2,TIM_CHANNEL_1);

}

/**
  * ��������: ���õ���ٶ�
  * ���뺯��: Duty,�������ռ�ձ�
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SetMotor2Speed(int16_t Duty)
{
  __HAL_TIM_SetCompare(&htimx_BDCMOTOR2,TIM_CHANNEL_1,Duty);
}

/**
  * ��������: ���õ��ת������
  * ���뺯��: Dir,���ת������
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SetMotor2Dir(int16_t Dir)
{
  if(Dir)
  {
    HAL_TIM_PWM_Start(&htimx_BDCMOTOR2,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR2,TIM_CHANNEL_1);         // ֹͣ���
  }
  else
  {
    HAL_TIM_PWM_Stop(&htimx_BDCMOTOR2,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htimx_BDCMOTOR2,TIM_CHANNEL_1);         // ֹͣ���
  }
}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
