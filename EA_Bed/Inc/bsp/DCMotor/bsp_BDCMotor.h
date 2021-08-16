#ifndef __BDCMOTOR1_TIM_H__
#define __BDCMOTOR1_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
#define BDCMOTOR1_TIMx                         TIM1
#define BDCMOTOR1_TIM_RCC_CLK_ENABLE()         __HAL_RCC_TIM1_CLK_ENABLE()
#define BDCMOTOR1_TIM_RCC_CLK_DISABLE()        __HAL_RCC_TIM1_CLK_DISABLE()

#define BDCMOTOR1_TIM_CH1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()     // ���PWM���������������ĵ�IN����
#define BDCMOTOR1_TIM_CH1_PORT                 GPIOA                            // CH1��CH1N������������ʹ��
#define BDCMOTOR1_TIM_CH1_PIN                  GPIO_PIN_8                       // ������������������OUT1��OUT2������
#define BDCMOTOR1_TIM_CH1N_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()     // CH1��CH1N��Ӧ����IN1��IN2
#define BDCMOTOR1_TIM_CH1N_PORT                GPIOB                            // ������������������OUT3��OUT4������
#define BDCMOTOR1_TIM_CH1N_PIN                 GPIO_PIN_13                      // CH1��CH1N��Ӧ����IN3��IN4

#define BDCMOTOR1_SHUTDOWN_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOH_CLK_ENABLE()     // CH1��CH1N��Ӧ����IN1��IN2
#define BDCMOTOR1_SHUTDOWN_PORT                GPIOH                            // ������������������OUT3��OUT4������
#define BDCMOTOR1_SHUTDOWN_PIN                 GPIO_PIN_6                       // CH1��CH1N��Ӧ����IN3��IN4

#define ENABLE_MOTOR1()                        HAL_GPIO_WritePin(BDCMOTOR1_SHUTDOWN_PORT,BDCMOTOR1_SHUTDOWN_PIN,GPIO_PIN_RESET)
#define SHUTDOWN_MOTOR1()                      HAL_GPIO_WritePin(BDCMOTOR1_SHUTDOWN_PORT,BDCMOTOR1_SHUTDOWN_PIN,GPIO_PIN_SET)

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��BDCMOTOR1_TIMx_PRESCALER+1��
#define BDCMOTOR1_TIM_PRESCALER               1    // ʵ��ʱ��Ƶ��Ϊ��84MHz

// ���嶨ʱ�����ڣ�PWMƵ��Ϊ��168MHz/��BDCMOTOR1_TIMx_PRESCALER+1��/��BDCMOTOR1_TIM_PERIOD+1��
#define BDCMOTOR1_TIM_PERIOD                  4199  // PWMƵ��Ϊ84MHz/(4199+1)=20KHz

#define BDCMOTOR1_DUTY_ZERO                   (0) // 0%ռ�ձ�
#define BDCMOTOR1_DUTY_FULL                   (BDCMOTOR1_TIM_PERIOD-100)
// ����߼���ʱ���ظ������Ĵ���ֵ
// ʵ��PWMƵ��Ϊ��168MHz/��BDCMOTOR1_TIMx_PRESCALER+1��/��BDCMOTOR1_TIM_PERIOD+1��/��BDCMOTOR1_TIM_REPETITIONCOUNTER+1��
#define BDCMOTOR1_TIM_REPETITIONCOUNTER       0


/* ------------------------------------------------------------------------- */
/* ��ˢֱ�����2 */

#define BDCMOTOR2_TIMx                         TIM8
#define BDCMOTOR2_TIM_RCC_CLK_ENABLE()         __HAL_RCC_TIM8_CLK_ENABLE()
#define BDCMOTOR2_TIM_RCC_CLK_DISABLE()        __HAL_RCC_TIM8_CLK_DISABLE()

#define BDCMOTOR2_TIM_CH1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOI_CLK_ENABLE()     // ���PWM���������������ĵ�IN����
#define BDCMOTOR2_TIM_CH1_PORT                 GPIOI                            // CH1��CH1N������������ʹ��
#define BDCMOTOR2_TIM_CH1_PIN                  GPIO_PIN_5                       // ������������������OUT1��OUT2������
#define BDCMOTOR2_TIM_CH1N_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOH_CLK_ENABLE()     // CH1��CH1N��Ӧ����IN1��IN2
#define BDCMOTOR2_TIM_CH1N_PORT                GPIOH                            // ������������������OUT3��OUT4������
#define BDCMOTOR2_TIM_CH1N_PIN                 GPIO_PIN_13                      // CH1��CH1N��Ӧ����IN3��IN4

#define BDCMOTOR2_SHUTDOWN_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOH_CLK_ENABLE()     // CH1��CH1N��Ӧ����IN1��IN2
#define BDCMOTOR2_SHUTDOWN_PORT                GPIOH                            // ������������������OUT3��OUT4������
#define BDCMOTOR2_SHUTDOWN_PIN                 GPIO_PIN_9                       // CH1��CH1N��Ӧ����IN3��IN4

#define ENABLE_MOTOR2()                        HAL_GPIO_WritePin(BDCMOTOR2_SHUTDOWN_PORT,BDCMOTOR2_SHUTDOWN_PIN,GPIO_PIN_RESET)
#define SHUTDOWN_MOTOR2()                      HAL_GPIO_WritePin(BDCMOTOR2_SHUTDOWN_PORT,BDCMOTOR2_SHUTDOWN_PIN,GPIO_PIN_SET)

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��BDCMOTOR2_TIMx_PRESCALER+1��
#define BDCMOTOR2_TIM_PRESCALER               1    // ʵ��ʱ��Ƶ��Ϊ��84MHz

// ���嶨ʱ�����ڣ�PWMƵ��Ϊ��168MHz/��BDCMOTOR2_TIMx_PRESCALER+1��/��BDCMOTOR2_TIM_PERIOD+1��
#define BDCMOTOR2_TIM_PERIOD                  4199  // PWMƵ��Ϊ84MHz/(4199+1)=20KHz

#define BDCMOTOR2_DUTY_ZERO                   (0) // 0%ռ�ձ�
#define BDCMOTOR2_DUTY_FULL                   (BDCMOTOR2_TIM_PERIOD-100)
// ����߼���ʱ���ظ������Ĵ���ֵ
// ʵ��PWMƵ��Ϊ��168MHz/��BDCMOTOR2_TIMx_PRESCALER+1��/��BDCMOTOR2_TIM_PERIOD+1��/��BDCMOTOR2_TIM_REPETITIONCOUNTER+1��
#define BDCMOTOR2_TIM_REPETITIONCOUNTER       0


/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_BDCMOTOR1;
extern TIM_HandleTypeDef htimx_BDCMOTOR2;
extern __IO int32_t PWM_Duty1;
extern __IO int32_t PWM_Duty2;

/* �������� ------------------------------------------------------------------*/

void BDCMOTOR1_TIMx_Init(void);
void SetMotor1Speed(int16_t Duty);
void SetMotor1Dir(int16_t Dir);

void BDCMOTOR2_TIMx_Init(void);
void SetMotor2Speed(int16_t Duty);
void SetMotor2Dir(int16_t Dir);

#endif	/* __BDCMOTOR1_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
