#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
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

// 定义定时器预分频，定时器实际时钟频率为：84MHz/（ENCODER1_TIMx_PRESCALER+1）
#define ENCODER1_TIM_PRESCALER               0  // 实际时钟频率为：84MHz

// 定义定时器周期，当定时器开始计数到ENCODER1_TIMx_PERIOD值是更新定时器并生成对应事件和中断
#define ENCODER1_TIM_PERIOD                  0xFFFF

/* ------------------------------------------------------------------------- */
/* 直流有刷电机接口2编码器2 */
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

// 定义定时器预分频，定时器实际时钟频率为：84MHz/（ENCODER2_TIMx_PRESCALER+1）
#define ENCODER2_TIM_PRESCALER               0  // 实际时钟频率为：84MHz

// 定义定时器周期，当定时器开始计数到ENCODER2_TIMx_PERIOD值是更新定时器并生成对应事件和中断
#define ENCODER2_TIM_PERIOD                  0xFFFF//TIM2是32 bits计数器,最高可达0xFFFFFFFF

/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_Encoder1;
extern TIM_HandleTypeDef htimx_Encoder2;

extern int32_t ENCODER1_OverflowCNT ;//定时器溢出次数
extern int32_t ENCODER2_OverflowCNT  ;//定时器溢出次数
/* 函数声明 ------------------------------------------------------------------*/
void ENCODER1_TIMx_Init(void);
void ENCODER2_TIMx_Init(void);

#endif	/* __ENCODER_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

