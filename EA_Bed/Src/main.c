/**
  ******************************************************************************
  * �ļ�����: main.c
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2018-01-13
  * ��    ��: ��ˢֱ�����λ�ñջ�����_λ��ʽPID
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
#include "key/bsp_key.h"
#include "encoder/bsp_encoder.h"
#include "usart/bsp_usartx.h"
#include "DCMotor/bsp_BDCMotor.h"
#include "led/bsp_led.h"
#include "nrf24l01/nrf24l01.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
typedef struct
{
  __IO int32_t  SetPoint;                                 //�趨Ŀ�� Desired Value
  __IO float    SumError;                                //����ۼ�
  __IO float    Proportion;                               //�������� Proportional Const
  __IO float    Integral;                                 //���ֳ��� Integral Const
  __IO float    Derivative;                               //΢�ֳ��� Derivative Const
  __IO int      LastError;                                //Error[-1]
  __IO int      PrevError;                                //Error[-2]
}PID_TypeDef;

typedef enum
{
	ACTION_BACK_UP			= 1,
	ACTION_BACK_DOWN		= 2,
	ACTION_THIGH_UP			= 3,
	ACTION_THIGH_DOWN		= 4,
	ACTION_CHAIR				= 5,
	ACTION_BED					= 6,
}action_t;

/* ˽�к궨�� ----------------------------------------------------------------*/

/*************************************/
// ����PID��غ�
// �����������趨�Ե������Ӱ��ǳ���
// PID����������ʱ��ϢϢ���
/*************************************/
#define  SPD_P_DATA      1.025f        // P����
#define  SPD_I_DATA      0.215f        // I����
#define  SPD_D_DATA      0.1f         // D����
#define  TARGET_LOC    11880        // Ŀ���ٶ�    1 r

/* ˽�б��� ------------------------------------------------------------------*/
__IO uint8_t  Start_flag = 0;       // PID ��ʼ��־
__IO uint32_t Motor_Dir = CW;             // �������
__IO int32_t Loc_Pulse;              // ����������ֵ Pulse
uint8_t Bed_status = 0;
uint8_t Is_running = 0;

/* ��չ���� ------------------------------------------------------------------ */
extern __IO uint32_t uwTick;

/* PID�ṹ�� */
PID_TypeDef  sPID;               // PID�����ṹ��

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
void PID_ParamInit(void) ;
int32_t LocPIDCalc(int32_t NextPoint);
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
	uint8_t nrf_rx_buf[16] = {0};
	
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();
  /* ���ڳ�ʼ�� */
  MX_USARTx_Init();
	/* NRF24L01��ʼ�� */
	NRF24L01_Init();
	NRF24L01_RX_Mode();
  /* LED��ʼ�� */
  LED_GPIO_Init();
  /* ������ʼ�� */
  KEY_GPIO_Init();
  /* ��������ʼ����ʹ�ܱ�����ģʽ */
  ENCODER_TIMx_Init();
	/* �߼����ƶ�ʱ����ʼ��������PWM������� */
  BDCMOTOR_TIMx_Init();
  /* �趨ռ�ձ� */
  PWM_Duty = 0;
  SetMotorSpeed(PWM_Duty);  // 0%
  /* PID ������ʼ�� */
  PID_ParamInit();
  /* ����ѭ�� */
  while (1)
  {
//    /* ֹͣ��ť */
//    if(KEY1_StateRead()==KEY_DOWN)
//    {
//      if(sPID.SetPoint > 0)
//      {
//        Motor_Dir = CCW;
//        BDDCMOTOR_DIR_CCW();
//      }
//      else
//      {
//        Motor_Dir = CW;
//        BDDCMOTOR_DIR_CW();
//      }
//      Start_flag = 1;
//    }
//    if(KEY2_StateRead()==KEY_DOWN)
//    {
//      SHUTDOWN_MOTOR();
//      HAL_TIM_PWM_Stop(&htimx_BDCMOTOR,TIM_CHANNEL_1);
//      HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR,TIM_CHANNEL_1);         // ֹͣ���
//    }
//    if(KEY3_StateRead()==KEY_DOWN)//����
//    {
//      sPID.SetPoint += 11880; // +1 r
//    }
//    if(KEY4_StateRead()==KEY_DOWN)//����
//    {
//      sPID.SetPoint -= 11880; // -1 r
//    }
		// receive data
		if(NRF24L01_RxPacket(nrf_rx_buf) == 0 && Is_running == 0) {
			switch (nrf_rx_buf[0]) {
				case ACTION_BACK_UP:
					break;
				case ACTION_BACK_DOWN:
					break;
				case ACTION_CHAIR:
					break;
				case ACTION_BED:
					break;
			}
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
  int32_t tmpPWM_Duty = 0;
  /* �ٶȻ�����100ms */
  if(uwTick % 100 == 0)
  {
    Loc_Pulse = (OverflowCount*CNT_MAX) + (int32_t)__HAL_TIM_GET_COUNTER(&htimx_Encoder);

    /* ����PID��� */
    if(Start_flag == 1)
    {
      /* �޶��ٶ� */
      PWM_Duty = LocPIDCalc(Loc_Pulse);
      if(PWM_Duty >= BDCMOTOR_DUTY_FULL/2)
        PWM_Duty = BDCMOTOR_DUTY_FULL/2;
      if(PWM_Duty <= -BDCMOTOR_DUTY_FULL/2)
        PWM_Duty = -BDCMOTOR_DUTY_FULL/2;

      /* �жϵ�ǰ�˶����� */
      if(PWM_Duty < 0)
      {
        Motor_Dir = CW;
        BDDCMOTOR_DIR_CW();
        tmpPWM_Duty = -PWM_Duty;
      }
      else
      {
        Motor_Dir = CCW;
        BDDCMOTOR_DIR_CCW();
        tmpPWM_Duty = PWM_Duty;
      }
      /* ���PWM */
      SetMotorSpeed( tmpPWM_Duty );
    }
    printf(" Loc: %d (Pulse) = %.3f (r) \n",Loc_Pulse, (float)Loc_Pulse/11880.0f);
  }
}
/******************** PID ������� ***************************/
/**
  * ��������: PID������ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void PID_ParamInit()
{
    sPID.LastError = 0;               // Error[-1]
    sPID.PrevError = 0;               // Error[-2]
    sPID.Proportion = SPD_P_DATA; // �������� Proportional Const
    sPID.Integral = SPD_I_DATA;   // ���ֳ���  Integral Const
    sPID.Derivative = SPD_D_DATA; // ΢�ֳ��� Derivative Const
    sPID.SetPoint = TARGET_LOC;     // �趨Ŀ��Desired Value
}
/**
  * �������ƣ�λ�ñջ�PID�������
  * �����������ǰ������
  * �� �� ֵ��Ŀ�������
  * ˵    ������
  */
int32_t LocPIDCalc(int32_t NextPoint)
{
  int32_t iError,dError;
  iError = sPID.SetPoint - NextPoint; //ƫ��

  if( (iError<50) && (iError>-50) )
    iError = 0;

  /* �޶��������� */
  if((iError<400 )&& (iError>-400))
  {
    sPID.SumError += iError; //����
    /* �趨�������� */
    if(sPID.SumError >= (TARGET_LOC*10))
       sPID.SumError  = (TARGET_LOC*10);
    if(sPID.SumError <= -(TARGET_LOC*10))
      sPID.SumError = -(TARGET_LOC*10);
  }

  dError = iError - sPID.LastError; //΢��
  sPID.LastError = iError;
  return (int32_t)( (sPID.Proportion * (float)iError) //������
                    + (sPID.Integral * (float)sPID.SumError) //������
                    + (sPID.Derivative * (float)dError) ); //΢����
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
