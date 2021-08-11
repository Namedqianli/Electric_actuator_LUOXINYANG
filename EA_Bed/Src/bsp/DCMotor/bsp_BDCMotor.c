/**
  ******************************************************************************
  * �ļ�����: bsp_BDCMOTOR.c
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-10-09
  * ��    ��: ��ˢֱ����������������������
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
TIM_HandleTypeDef htimx_BDCMOTOR;
__IO int32_t PWM_Duty=BDCMOTOR_DUTY_ZERO;         // ռ�ձȣ�PWM_Duty/BDCMOTOR_TIM_PERIOD*100%
                                    // ��ת���򲻽�������йأ�Ҳ���������йأ���Ҫ�������
                                    // �򵥵ķ����ǣ�������Ʒ�����Ҫ���෴����������PWM�����߽ӷ�
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ������ʱ��Ӳ����ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: BDCMOTOR���GPIO��ʼ������,�ú�����HAL���ڲ�����.
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  /* BDCMOTOR���GPIO��ʼ������ */
  if(htim == &htimx_BDCMOTOR)
  {
    GPIO_InitTypeDef GPIO_InitStruct;
    /* ���Ŷ˿�ʱ��ʹ�� */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    BDCMOTOR_TIM_CH1_GPIO_CLK_ENABLE();
    BDCMOTOR_TIM_CH1N_GPIO_CLK_ENABLE();
    SHUTDOWN_GPIO_CLK_ENABLE();

    /* BDCMOTOR��������������IO��ʼ�� */
    GPIO_InitStruct.Pin = BDCMOTOR_TIM_CH1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(BDCMOTOR_TIM_CH1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BDCMOTOR_TIM_CH1N_PIN;
    HAL_GPIO_Init(BDCMOTOR_TIM_CH1N_PORT, &GPIO_InitStruct);

    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SHUTDOWN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(SHUTDOWN_PORT, &GPIO_InitStruct);

    /* ʹ�ܵ���������� */
    ENABLE_MOTOR();

  }
}

/**
  * ��������: BDCMOTOR��ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void BDCMOTOR_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef  sBDTConfig;            // ��ʱ������ʱ��Ƚ����

  /* ������ʱ������ʱ��ʹ�� */
  BDCMOTOR_TIM_RCC_CLK_ENABLE();

  /* ��ʱ�������������� */
  htimx_BDCMOTOR.Instance = BDCMOTOR_TIMx;                                 // ��ʱ�����
  htimx_BDCMOTOR.Init.Prescaler = BDCMOTOR_TIM_PRESCALER;                  // ��ʱ��Ԥ��Ƶ��
  htimx_BDCMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;                  // �����������ϼ���
  htimx_BDCMOTOR.Init.Period = BDCMOTOR_TIM_PERIOD;                        // ��ʱ������
  htimx_BDCMOTOR.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;              // ʱ�ӷ�Ƶ
  htimx_BDCMOTOR.Init.RepetitionCounter = BDCMOTOR_TIM_REPETITIONCOUNTER;  // �ظ�������
  /* ��ʼ����ʱ���Ƚ�������� */
  HAL_TIM_PWM_Init(&htimx_BDCMOTOR);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htimx_BDCMOTOR, &sClockSourceConfig);

  /* ����ɲ������,ʵ����������Ч��ƽ�Ǹߵ�ƽ */
  sBDTConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE ;
  sBDTConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW ;
  sBDTConfig.BreakState = TIM_BREAK_DISABLE ;
  sBDTConfig.DeadTime = 0 ;
  sBDTConfig.LockLevel = TIM_LOCKLEVEL_OFF ;
  sBDTConfig.OffStateIDLEMode= TIM_OSSI_DISABLE ;
  sBDTConfig.OffStateRunMode = TIM_OSSR_ENABLE ;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx_BDCMOTOR,&sBDTConfig);

/* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;                  // �Ƚ����ģʽ��PWM1ģʽ
  sConfigOC.Pulse =  PWM_Duty;                         // ռ�ձ�
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;          // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;        // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // ����ͨ�����е�ƽ
  HAL_TIM_PWM_ConfigChannel(&htimx_BDCMOTOR, &sConfigOC, TIM_CHANNEL_1);

	/* ������ʱ�� */
  HAL_TIM_Base_Start(&htimx_BDCMOTOR);
}


/**
  * ��������: ������ʱ��Ӳ������ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==BDCMOTOR_TIMx)
  {
    /* ������ʱ������ʱ�ӽ��� */
    BDCMOTOR_TIM_RCC_CLK_DISABLE();

    HAL_GPIO_DeInit(BDCMOTOR_TIM_CH1_PORT,BDCMOTOR_TIM_CH1_PIN);
    HAL_GPIO_DeInit(BDCMOTOR_TIM_CH1N_PORT,BDCMOTOR_TIM_CH1N_PIN);
  }
}

/**
  * ��������: ���õ���ٶ�
  * ���뺯��: Duty,�������ռ�ձ�
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SetMotorSpeed(int16_t Duty)
{
  __HAL_TIM_SetCompare(&htimx_BDCMOTOR,TIM_CHANNEL_1,Duty);
}

/**
  * ��������: ���õ��ת������
  * ���뺯��: Dir,���ת������
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SetMotorDir(int16_t Dir)
{
  if(Dir)
  {
    HAL_TIM_PWM_Start(&htimx_BDCMOTOR,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR,TIM_CHANNEL_1);         // ֹͣ���
  }
  else
  {
    HAL_TIM_PWM_Stop(&htimx_BDCMOTOR,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htimx_BDCMOTOR,TIM_CHANNEL_1);         // ֹͣ���
  }
}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
