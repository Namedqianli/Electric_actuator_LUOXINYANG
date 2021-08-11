#ifndef __BDCMOTOR_TIM_H__
#define __BDCMOTOR_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define BDCMOTOR_TIMx                         TIM1
#define BDCMOTOR_TIM_RCC_CLK_ENABLE()         __HAL_RCC_TIM1_CLK_ENABLE()
#define BDCMOTOR_TIM_RCC_CLK_DISABLE()        __HAL_RCC_TIM1_CLK_DISABLE()

#define BDCMOTOR_TIM_CH1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()     // 输出PWM脉冲给电机控制器的的IN引脚
#define BDCMOTOR_TIM_CH1_PORT                 GPIOA                            // CH1和CH1N两个引脚配套使用
#define BDCMOTOR_TIM_CH1_PIN                  GPIO_PIN_8                       // 如果电机接在驱动器的OUT1和OUT2端子上
#define BDCMOTOR_TIM_CH1N_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()     // CH1和CH1N对应接在IN1和IN2
#define BDCMOTOR_TIM_CH1N_PORT                GPIOB                            // 如果电机接在驱动器的OUT3和OUT4端子上
#define BDCMOTOR_TIM_CH1N_PIN                 GPIO_PIN_13                      // CH1和CH1N对应接在IN3和IN4

#define SHUTDOWN_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOH_CLK_ENABLE()     // CH1和CH1N对应接在IN1和IN2
#define SHUTDOWN_PORT                         GPIOH                            // 如果电机接在驱动器的OUT3和OUT4端子上
#define SHUTDOWN_PIN                          GPIO_PIN_6                       // CH1和CH1N对应接在IN3和IN4

#define ENABLE_MOTOR()                        HAL_GPIO_WritePin(SHUTDOWN_PORT,SHUTDOWN_PIN,GPIO_PIN_RESET)
#define SHUTDOWN_MOTOR()                      HAL_GPIO_WritePin(SHUTDOWN_PORT,SHUTDOWN_PIN,GPIO_PIN_SET)

#define BDCMOTOR_TIM_CC_IRQx                  TIM1_CC_IRQn
#define BDCMOTOR_TIM_CC_IRQxHandler           TIM1_CC_IRQHandler

// 定义定时器预分频，定时器实际时钟频率为：168MHz/（BDCMOTOR_TIMx_PRESCALER+1）
#define BDCMOTOR_TIM_PRESCALER               1    // 实际时钟频率为：84MHz

// 定义定时器周期，PWM频率为：168MHz/（BDCMOTOR_TIMx_PRESCALER+1）/（BDCMOTOR_TIM_PERIOD+1）
#define BDCMOTOR_TIM_PERIOD                  4199  // PWM频率为84MHz/(4199+1)=20KHz

#define BDCMOTOR_DUTY_ZERO                   (0)       // 0%占空比
#define BDCMOTOR_DUTY_FULL                   (BDCMOTOR_TIM_PERIOD-100)              // 97.625%占空比
#define VC_OFFSET

#define BDDCMOTOR_DIR_CW()                      {HAL_TIM_PWM_Stop(&htimx_BDCMOTOR,TIM_CHANNEL_1);\
                                              HAL_TIMEx_PWMN_Start(&htimx_BDCMOTOR,TIM_CHANNEL_1);}
#define BDDCMOTOR_DIR_CCW()                     {HAL_TIM_PWM_Start(&htimx_BDCMOTOR,TIM_CHANNEL_1);\
                                              HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR,TIM_CHANNEL_1);}
#define CW                                   1
#define CCW                                  0

// 定义高级定时器重复计数寄存器值
// 实际PWM频率为：168MHz/（BDCMOTOR_TIMx_PRESCALER+1）/（BDCMOTOR_TIM_PERIOD+1）/（BDCMOTOR_TIM_REPETITIONCOUNTER+1）
#define BDCMOTOR_TIM_REPETITIONCOUNTER       0

/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_BDCMOTOR;
extern __IO int32_t PWM_Duty;

/* 函数声明 ------------------------------------------------------------------*/

void BDCMOTOR_TIMx_Init(void);
void SetMotorDir(int16_t Dir);
void SetMotorSpeed(int16_t Duty);
#endif	/* __BDCMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
