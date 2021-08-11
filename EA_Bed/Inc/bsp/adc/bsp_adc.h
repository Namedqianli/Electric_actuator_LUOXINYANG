#ifndef __ADC_H__
#define	__ADC_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
// 注意：用作ADC采集的IO必须没有复用，否则采集电压会有影响
/********************ADC输入通道（引脚）配置**************************/
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
                                 
/* 扩展变量 ------------------------------------------------------------------*/
extern ADC_HandleTypeDef hadcx;
extern DMA_HandleTypeDef hdma_adcx;

/* 函数声明 ------------------------------------------------------------------*/
void MX_ADCx_Init(void);
void MX_DMA_Init(void) ;
#endif /* __ADC_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
