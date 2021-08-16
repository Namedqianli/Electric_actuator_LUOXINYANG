/**
  ******************************************************************************
  * �ļ�����: main.c
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.1
  * ��д����: 2018-03-06
  * ��    ��: 2��ֱ����ˢ���_����������
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
#include "stm32f4xx_hal.h"
#include "DCMotor/bsp_BDCMotor.h"
#include "key/bsp_key.h"
#include "encoder/bsp_encoder.h"
#include "usart/bsp_usartx.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
#define ENCODER1_LINE     11           // ����������
#define ENCODER1_SPEEDRATIO  270       // ������ٱ�
#define ENCODER1_PPR         (ENCODER1_SPEEDRATIO*ENCODER1_LINE*4) // Pulse/r ÿȦ�ɲ����������
#define ENCODER2_LINE     11           // ����������
#define ENCODER2_SPEEDRATIO  270       // ������ٱ�
#define ENCODER2_PPR         (ENCODER2_SPEEDRATIO*ENCODER2_LINE*4) // Pulse/r ÿȦ�ɲ����������
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
__IO uint8_t  start_flag=0;
__IO uint16_t time_count=0;        // ʱ�������ÿ1ms����һ(��ζ�ʱ��Ƶ���й�)
__IO int32_t CaptureNumber=0;     // ���벶����
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();                                     // ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // ���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        // ��HSE
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    // ��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            // PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 // 8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               // 336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     // 2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 // USB/SDIO/������������ȵ���PLL��Ƶϵ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2ʱ�ӣ�84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // ʹ��CSS���ܣ�����ʹ���ⲿ�����ڲ�ʱ��ԴΪ����

 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                 // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();
    /* ���ڳ�ʼ�� */
  MX_USARTx_Init();
  /* ������ʼ�� */
  KEY_GPIO_Init();

  /* ��������ʼ����ʹ�ܱ�����ģʽ */
  ENCODER1_TIMx_Init();
  ENCODER2_TIMx_Init();

  PWM_Duty1 = 500;
  PWM_Duty2 = 500;
  SetMotor1Speed( PWM_Duty1 );
  SetMotor2Speed( PWM_Duty1 );
  /* �߼����ƶ�ʱ����ʼ��������PWM������� */
  BDCMOTOR1_TIMx_Init();
  BDCMOTOR2_TIMx_Init();
  start_flag = 1;
  /* ����ѭ�� */
  while (1)
  {
    /* ���1����  */
    if(KEY1_StateRead() == KEY_DOWN)
    {
      PWM_Duty1 += 100;
      if(PWM_Duty1 >= BDCMOTOR1_TIM_PERIOD)
      {
        PWM_Duty1 = BDCMOTOR1_DUTY_FULL;
      }
      SetMotor1Speed( PWM_Duty1 );
    }
    /* ���1���� */
    if(KEY2_StateRead() == KEY_DOWN)
    {
      PWM_Duty1 -= 100;
      if(PWM_Duty1 <= BDCMOTOR1_DUTY_ZERO)
      {
        PWM_Duty1 = BDCMOTOR1_DUTY_ZERO;
      }
      SetMotor1Speed( PWM_Duty1 );
    }
    /* ���2����  */
    if(KEY3_StateRead() == KEY_DOWN)
    {
      PWM_Duty2 += 100;
      if(PWM_Duty2 >= BDCMOTOR2_TIM_PERIOD)
      {
        PWM_Duty2 = BDCMOTOR2_DUTY_FULL;
      }
      SetMotor2Speed( PWM_Duty2 );
    }
    /* ���2���� */
    if(KEY4_StateRead() == KEY_DOWN)
    {
      PWM_Duty2 -= 100;
      if(PWM_Duty2 <= BDCMOTOR2_DUTY_ZERO)
      {
        PWM_Duty2 = BDCMOTOR2_DUTY_ZERO;
      }
      SetMotor2Speed( PWM_Duty2 );
    }
   /* ֹͣ */
    if(KEY5_StateRead() == KEY_DOWN)
    {
      HAL_TIM_PWM_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);

      HAL_TIM_PWM_Stop(&htimx_BDCMOTOR2,TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR2,TIM_CHANNEL_1);
    }
  }
}

/**
  * ��������: ϵͳ�δ�ʱ���жϻص�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ÿ����һ�εδ�ʱ���жϽ���ûص�����һ��
  */
void HAL_SYSTICK_Callback(void)
{
  if(start_flag) // �ȴ����������ſ�ʼ��ʱ
  {
    time_count++;         // ÿ1ms�Զ���һ
    if(time_count==1000)  // 1s
    {
      float Speed = 0;
      /* ��ȡ������1��ֵ */
      CaptureNumber = ( int32_t )__HAL_TIM_GET_COUNTER(&htimx_Encoder1)+ENCODER1_OverflowCNT*65536;
      printf("Input1:%d \n",CaptureNumber);
      // 4 : ʹ�ö�ʱ���������ӿڲ���AB��������غ��½��أ�һ������*4.
      // 11������������(ת��һȦ���������)
      // 270����������ȣ��ڲ����ת��Ȧ�����������ת��Ȧ���ȣ������ٳ��ֱ�
      Speed = (float)CaptureNumber/ENCODER1_PPR;
      printf("���1ʵ��ת���ٶ�%0.2f r/s \n",Speed);
      ENCODER1_OverflowCNT = 0;
      __HAL_TIM_SET_COUNTER(&htimx_Encoder1,0);

      /* ------------------------------------------------------------- */
      CaptureNumber = ( int32_t )__HAL_TIM_GET_COUNTER(&htimx_Encoder2)+ENCODER2_OverflowCNT*65536;
      printf("Input2:%d \n",CaptureNumber);
      // 4 : ʹ�ö�ʱ���������ӿڲ���AB��������غ��½��أ�һ������*4.
      // 11������������(ת��һȦ���������)
      // 270����������ȣ��ڲ����ת��Ȧ�����������ת��Ȧ���ȣ������ٳ��ֱ�
      Speed = (float)CaptureNumber/ENCODER2_PPR;
      printf("���2ʵ��ת���ٶ�%0.2f r/s \n",Speed);
      ENCODER2_OverflowCNT = 0;
      __HAL_TIM_SET_COUNTER(&htimx_Encoder2,0);

      time_count=0;
    }
  }
}


/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
