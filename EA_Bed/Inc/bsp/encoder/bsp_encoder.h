#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
#define ENCODER1_TIMx                        TIM3
#define ENCODER1_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM3_CLK_ENABLE()
#define ENCODER1_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM3_CLK_DISABLE()

#define ENCODER1_TIM_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOC_CLK_ENABLE()
#define ENCODER1_TIM_CH1_PIN                 GPIO_PIN_6
#define ENCODER1_TIM_CH1_GPIO                GPIOC
#define ENCODER1_TIM_CH2_PIN                 GPIO_PIN_7
#define ENCODER1_TIM_CH2_GPIO                GPIOC

#define TIM_ENCODER1MODE_TIx                 TIM_ENCODERMODE_TI12

#define ENCODER1_TIM_IRQn                    TIM3_IRQn
#define ENCODER1_TIM_IRQHANDLER              TIM3_IRQHandler

#define ENCODER1_GPIO_AFx_TIMx               GPIO_AF2_TIM3

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��84MHz/��ENCODER1_TIMx_PRESCALER+1��
#define ENCODER1_TIM_PRESCALER               0  // ʵ��ʱ��Ƶ��Ϊ��84MHz

// ���嶨ʱ�����ڣ�����ʱ����ʼ������ENCODER1_TIMx_PERIODֵ�Ǹ��¶�ʱ�������ɶ�Ӧ�¼����ж�
#define ENCODER1_TIM_PERIOD                  0xFFFF

/* ------------------------------------------------------------------------- */
/* ֱ����ˢ����ӿ�2������2 */
#define ENCODER2_TIMx                        TIM2
#define ENCODER2_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM2_CLK_ENABLE()
#define ENCODER2_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM2_CLK_DISABLE()

#define ENCODER2_TIM_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define ENCODER2_TIM_CH1_PIN                 GPIO_PIN_15
#define ENCODER2_TIM_CH1_GPIO                GPIOA
#define ENCODER2_TIM_CH2_PIN                 GPIO_PIN_3
#define ENCODER2_TIM_CH2_GPIO                GPIOB

#define TIM_ENCODER2MODE_TIx                 TIM_ENCODERMODE_TI12

#define ENCODER2_TIM_IRQn                    TIM2_IRQn
#define ENCODER2_TIM_IRQHANDLER              TIM2_IRQHandler

#define ENCODER2_GPIO_AFx_TIMx               GPIO_AF1_TIM2

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��84MHz/��ENCODER2_TIMx_PRESCALER+1��
#define ENCODER2_TIM_PRESCALER               0  // ʵ��ʱ��Ƶ��Ϊ��84MHz

// ���嶨ʱ�����ڣ�����ʱ����ʼ������ENCODER2_TIMx_PERIODֵ�Ǹ��¶�ʱ�������ɶ�Ӧ�¼����ж�
#define ENCODER2_TIM_PERIOD                  0xFFFF//TIM2��32 bits������,��߿ɴ�0xFFFFFFFF

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_Encoder1;
extern TIM_HandleTypeDef htimx_Encoder2;

extern int32_t ENCODER1_OverflowCNT ;//��ʱ���������
extern int32_t ENCODER2_OverflowCNT  ;//��ʱ���������
/* �������� ------------------------------------------------------------------*/
void ENCODER1_TIMx_Init(void);
void ENCODER2_TIMx_Init(void);

#endif	/* __ENCODER_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

