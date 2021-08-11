#ifndef __ADC_H__
#define	__ADC_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
// ע�⣺����ADC�ɼ���IO����û�и��ã�����ɼ���ѹ����Ӱ��
/********************ADC����ͨ�������ţ�����**************************/
#define ADCx_RCC_CLK_ENABLE()            __HAL_RCC_ADC3_CLK_ENABLE()
#define ADCx_RCC_CLK_DISABLE()           __HAL_RCC_ADC3_CLK_DISABLE()
#define ADCx                             ADC3
#define ADC_CURRENT_CHANNEL              ADC_CHANNEL_8   
#define ADC_VOLT_CHANNEL                 ADC_CHANNEL_6
#define ADC_OVP_IRQx                     ADC_IRQn
#define ADC_OVP_IRQHandler               ADC_IRQHandler

#define DMAx_RCC_CLK_ENABLE()            __HAL_RCC_DMA2_CLK_ENABLE()
#define ADCx_DMA_IRQx                    DMA2_Stream0_IRQn
#define ADCx_DMA_IRQx_Handler            DMA2_Stream0_IRQHandler
#define DMAx_Stream_x                    DMA2_Stream0
#define DMAx_CHANNEL_x                   DMA_CHANNEL_2

#define ADC_CUR_GPIO_ClK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()
#define ADC_CUR_GPIO                     GPIOF
#define ADC_CUR_GPIO_PIN                 GPIO_PIN_10        

#define ADC_VOLT_GPIO_ClK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()
#define ADC_VOLT_GPIO                    GPIOF
#define ADC_VOLT_GPIO_PIN                GPIO_PIN_8
                                 
/* ��չ���� ------------------------------------------------------------------*/
extern ADC_HandleTypeDef hadcx;
extern DMA_HandleTypeDef hdma_adcx;

/* �������� ------------------------------------------------------------------*/
void MX_ADCx_Init(void);
void MX_DMA_Init(void) ;
#endif /* __ADC_H__ */

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
