#ifndef __BDCMOTOR1_TIM_H__
#define __BDCMOTOR1_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define BDCMOTOR1_TIMx                         TIM1
#define BDCMOTOR1_TIM_RCC_CLK_ENABLE()         __HAL_RCC_TIM1_CLK_ENABLE()
#define BDCMOTOR1_TIM_RCC_CLK_DISABLE()        __HAL_RCC_TIM1_CLK_DISABLE()

#define BDCMOTOR1_TIM_CH1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()     // 输出PWM脉冲给电机控制器的的IN引脚
#define BDCMOTOR1_TIM_CH1_PORT                 GPIOA                            // CH1和CH1N两个引脚配套使用
#define BDCMOTOR1_TIM_CH1_PIN                  GPIO_PIN_8                       // 如果电机接在驱动器的OUT1和OUT2端子上
#define BDCMOTOR1_TIM_CH1N_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()     // CH1和CH1N对应接在IN1和IN2
#define BDCMOTOR1_TIM_CH1N_PORT                GPIOB                            // 如果电机接在驱动器的OUT3和OUT4端子上
#define BDCMOTOR1_TIM_CH1N_PIN                 GPIO_PIN_13                      // CH1和CH1N对应接在IN3和IN4

#define BDCMOTOR1_SHUTDOWN_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOH_CLK_ENABLE()     // CH1和CH1N对应接在IN1和IN2
#define BDCMOTOR1_SHUTDOWN_PORT                GPIOH                            // 如果电机接在驱动器的OUT3和OUT4端子上
#define BDCMOTOR1_SHUTDOWN_PIN                 GPIO_PIN_6                       // CH1和CH1N对应接在IN3和IN4

#define ENABLE_MOTOR1()                        HAL_GPIO_WritePin(BDCMOTOR1_SHUTDOWN_PORT,BDCMOTOR1_SHUTDOWN_PIN,GPIO_PIN_RESET)
#define SHUTDOWN_MOTOR1()                      HAL_GPIO_WritePin(BDCMOTOR1_SHUTDOWN_PORT,BDCMOTOR1_SHUTDOWN_PIN,GPIO_PIN_SET)

// 定义定时器预分频，定时器实际时钟频率为：168MHz/（BDCMOTOR1_TIMx_PRESCALER+1）
#define BDCMOTOR1_TIM_PRESCALER               1    // 实际时钟频率为：84MHz

// 定义定时器周期，PWM频率为：168MHz/（BDCMOTOR1_TIMx_PRESCALER+1）/（BDCMOTOR1_TIM_PERIOD+1）
#define BDCMOTOR1_TIM_PERIOD                  4199  // PWM频率为84MHz/(4199+1)=20KHz

#define BDCMOTOR1_DUTY_ZERO                   (0) // 0%占空比
#define BDCMOTOR1_DUTY_FULL                   (BDCMOTOR1_TIM_PERIOD-100)
// 定义高级定时器重复计数寄存器值
// 实际PWM频率为：168MHz/（BDCMOTOR1_TIMx_PRESCALER+1）/（BDCMOTOR1_TIM_PERIOD+1）/（BDCMOTOR1_TIM_REPETITIONCOUNTER+1）
#define BDCMOTOR1_TIM_REPETITIONCOUNTER       0


/* ------------------------------------------------------------------------- */
/* 有刷直流电机2 */

#define BDCMOTOR2_TIMx                         TIM8
#define BDCMOTOR2_TIM_RCC_CLK_ENABLE()         __HAL_RCC_TIM8_CLK_ENABLE()
#define BDCMOTOR2_TIM_RCC_CLK_DISABLE()        __HAL_RCC_TIM8_CLK_DISABLE()

#define BDCMOTOR2_TIM_CH1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOI_CLK_ENABLE()     // 输出PWM脉冲给电机控制器的的IN引脚
#define BDCMOTOR2_TIM_CH1_PORT                 GPIOI                            // CH1和CH1N两个引脚配套使用
#define BDCMOTOR2_TIM_CH1_PIN                  GPIO_PIN_5                       // 如果电机接在驱动器的OUT1和OUT2端子上
#define BDCMOTOR2_TIM_CH1N_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOH_CLK_ENABLE()     // CH1和CH1N对应接在IN1和IN2
#define BDCMOTOR2_TIM_CH1N_PORT                GPIOH                            // 如果电机接在驱动器的OUT3和OUT4端子上
#define BDCMOTOR2_TIM_CH1N_PIN                 GPIO_PIN_13                      // CH1和CH1N对应接在IN3和IN4

#define BDCMOTOR2_SHUTDOWN_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOH_CLK_ENABLE()     // CH1和CH1N对应接在IN1和IN2
#define BDCMOTOR2_SHUTDOWN_PORT                GPIOH                            // 如果电机接在驱动器的OUT3和OUT4端子上
#define BDCMOTOR2_SHUTDOWN_PIN                 GPIO_PIN_9                       // CH1和CH1N对应接在IN3和IN4

#define ENABLE_MOTOR2()                        HAL_GPIO_WritePin(BDCMOTOR2_SHUTDOWN_PORT,BDCMOTOR2_SHUTDOWN_PIN,GPIO_PIN_RESET)
#define SHUTDOWN_MOTOR2()                      HAL_GPIO_WritePin(BDCMOTOR2_SHUTDOWN_PORT,BDCMOTOR2_SHUTDOWN_PIN,GPIO_PIN_SET)

// 定义定时器预分频，定时器实际时钟频率为：168MHz/（BDCMOTOR2_TIMx_PRESCALER+1）
#define BDCMOTOR2_TIM_PRESCALER               1    // 实际时钟频率为：84MHz

// 定义定时器周期，PWM频率为：168MHz/（BDCMOTOR2_TIMx_PRESCALER+1）/（BDCMOTOR2_TIM_PERIOD+1）
#define BDCMOTOR2_TIM_PERIOD                  4199  // PWM频率为84MHz/(4199+1)=20KHz

#define BDCMOTOR2_DUTY_ZERO                   (0) // 0%占空比
#define BDCMOTOR2_DUTY_FULL                   (BDCMOTOR2_TIM_PERIOD-100)
// 定义高级定时器重复计数寄存器值
// 实际PWM频率为：168MHz/（BDCMOTOR2_TIMx_PRESCALER+1）/（BDCMOTOR2_TIM_PERIOD+1）/（BDCMOTOR2_TIM_REPETITIONCOUNTER+1）
#define BDCMOTOR2_TIM_REPETITIONCOUNTER       0


/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_BDCMOTOR1;
extern TIM_HandleTypeDef htimx_BDCMOTOR2;
extern __IO int32_t PWM_Duty1;
extern __IO int32_t PWM_Duty2;

/* 函数声明 ------------------------------------------------------------------*/

void BDCMOTOR1_TIMx_Init(void);
void SetMotor1Speed(int16_t Duty);
void SetMotor1Dir(int16_t Dir);

void BDCMOTOR2_TIMx_Init(void);
void SetMotor2Speed(int16_t Duty);
void SetMotor2Dir(int16_t Dir);

#endif	/* __BDCMOTOR1_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
