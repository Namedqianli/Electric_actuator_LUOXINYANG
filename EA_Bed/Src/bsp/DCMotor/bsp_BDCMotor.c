/**
  ******************************************************************************
  * 文件名程: bsp_BDCMOTOR1.c
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.1
  * 编写日期: 2018-03-05
  * 功    能: 2轴减速电机GM37545_编码器测速
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  *
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "DCMotor/bsp_BDCMotor.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_BDCMOTOR1;
TIM_HandleTypeDef htimx_BDCMOTOR2;
__IO int32_t PWM_Duty1=500;         // 占空比：PWM_Duty/BDCMOTOR1_TIM_PERIOD*100%
__IO int32_t PWM_Duty2=500;         // 占空比：PWM_Duty/BDCMOTOR1_TIM_PERIOD*100%

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 基本定时器硬件初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: BDCMOTOR1相关GPIO初始化配置,该函数被HAL库内部调用.
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  /* BDCMOTOR1相关GPIO初始化配置 */
  if(htim == &htimx_BDCMOTOR1)
  {
    GPIO_InitTypeDef GPIO_InitStruct;
    /* 引脚端口时钟使能 */
    BDCMOTOR1_TIM_CH1_GPIO_CLK_ENABLE();
    BDCMOTOR1_TIM_CH1N_GPIO_CLK_ENABLE();
    BDCMOTOR1_SHUTDOWN_GPIO_CLK_ENABLE();

    /* BDCMOTOR1输出脉冲控制引脚IO初始化 */
    GPIO_InitStruct.Pin = BDCMOTOR1_TIM_CH1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(BDCMOTOR1_TIM_CH1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BDCMOTOR1_TIM_CH1N_PIN;
    HAL_GPIO_Init(BDCMOTOR1_TIM_CH1N_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = BDCMOTOR1_SHUTDOWN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(BDCMOTOR1_SHUTDOWN_PORT, &GPIO_InitStruct);

    /* 使能电机控制引脚 */
    ENABLE_MOTOR1();
  }
  /* BDCMOTOR2相关GPIO初始化配置 */
  if(htim == &htimx_BDCMOTOR2)
  {
    GPIO_InitTypeDef GPIO_InitStruct;
    /* 引脚端口时钟使能 */
    BDCMOTOR2_TIM_CH1_GPIO_CLK_ENABLE();
    BDCMOTOR2_TIM_CH1N_GPIO_CLK_ENABLE();
    BDCMOTOR2_SHUTDOWN_GPIO_CLK_ENABLE();

    /* BDCMOTOR2输出脉冲控制引脚IO初始化 */
    GPIO_InitStruct.Pin = BDCMOTOR2_TIM_CH1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(BDCMOTOR2_TIM_CH1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BDCMOTOR2_TIM_CH1N_PIN;
    HAL_GPIO_Init(BDCMOTOR2_TIM_CH1N_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = BDCMOTOR2_SHUTDOWN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(BDCMOTOR2_SHUTDOWN_PORT, &GPIO_InitStruct);

    /* 使能电机控制引脚 */
    ENABLE_MOTOR2();
  }
}

/**
  * 函数功能: BDCMOTOR1定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void BDCMOTOR1_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // 定时器时钟
  TIM_OC_InitTypeDef sConfigOC;                          // 定时器通道比较输出
  TIM_BreakDeadTimeConfigTypeDef  sBDTConfig;            // 定时器死区时间比较输出

  /* 基本定时器外设时钟使能 */
  BDCMOTOR1_TIM_RCC_CLK_ENABLE();

  /* 定时器基本环境配置 */
  htimx_BDCMOTOR1.Instance = BDCMOTOR1_TIMx;                                 // 定时器编号
  htimx_BDCMOTOR1.Init.Prescaler = BDCMOTOR1_TIM_PRESCALER;                  // 定时器预分频器
  htimx_BDCMOTOR1.Init.CounterMode = TIM_COUNTERMODE_UP;                  // 计数方向：向上计数
  htimx_BDCMOTOR1.Init.Period = BDCMOTOR1_TIM_PERIOD;                        // 定时器周期
  htimx_BDCMOTOR1.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;              // 时钟分频
  htimx_BDCMOTOR1.Init.RepetitionCounter = BDCMOTOR1_TIM_REPETITIONCOUNTER;  // 重复计数器
  /* 初始化定时器比较输出环境 */
  HAL_TIM_PWM_Init(&htimx_BDCMOTOR1);

  /* 定时器时钟源配置 */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       // 使用内部时钟源
  HAL_TIM_ConfigClockSource(&htimx_BDCMOTOR1, &sClockSourceConfig);

  /* 死区刹车配置,配置有效电平是低电平 */
  sBDTConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE ;
  sBDTConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW ;
  sBDTConfig.BreakState = TIM_BREAK_DISABLE ;
  sBDTConfig.DeadTime = 0 ;
  sBDTConfig.LockLevel = TIM_LOCKLEVEL_OFF ;
  sBDTConfig.OffStateIDLEMode= TIM_OSSI_DISABLE ;
  sBDTConfig.OffStateRunMode = TIM_OSSR_ENABLE ;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx_BDCMOTOR1,&sBDTConfig);

  /* 定时器比较输出配置 */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;                  // 比较输出模式：PWM1模式
  sConfigOC.Pulse =  PWM_Duty1;                         // 占空比
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;          // 输出极性
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;        // 互补通道输出极性
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // 快速模式
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // 空闲电平
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // 互补通道空闲电平
  HAL_TIM_PWM_ConfigChannel(&htimx_BDCMOTOR1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_Base_Start(&htimx_BDCMOTOR1);

    /* 启动定时器通道和互补通道PWM输出 */
  HAL_TIM_PWM_Start(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);

}


/**
  * 函数功能: 基本定时器硬件反初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==BDCMOTOR1_TIMx)
  {
    /* 基本定时器外设时钟禁用 */
    BDCMOTOR1_TIM_RCC_CLK_DISABLE();

    HAL_GPIO_DeInit(BDCMOTOR1_TIM_CH1_PORT,BDCMOTOR1_TIM_CH1_PIN);
    HAL_GPIO_DeInit(BDCMOTOR1_TIM_CH1N_PORT,BDCMOTOR1_TIM_CH1N_PIN);
  }
  if(htim_base->Instance==BDCMOTOR2_TIMx)
  {
    /* 基本定时器外设时钟禁用 */
    BDCMOTOR2_TIM_RCC_CLK_DISABLE();

    HAL_GPIO_DeInit(BDCMOTOR2_TIM_CH1_PORT,BDCMOTOR2_TIM_CH1_PIN);
    HAL_GPIO_DeInit(BDCMOTOR2_TIM_CH1N_PORT,BDCMOTOR2_TIM_CH1N_PIN);
  }

}
/**
  * 函数功能: 设置电机速度
  * 输入函数: Duty,输出脉冲占空比
  * 返 回 值: 无
  * 说    明: 无
  */
void SetMotor1Speed(int16_t Duty)
{
  __HAL_TIM_SetCompare(&htimx_BDCMOTOR1,TIM_CHANNEL_1,Duty);
}

/**
  * 函数功能: 设置电机转动方向
  * 输入函数: Dir,电机转动方向
  * 返 回 值: 无
  * 说    明: 无
  */
void SetMotor1Dir(int16_t Dir)
{
  if(Dir)
  {
    HAL_TIM_PWM_Start(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);         // 停止输出
  }
  else
  {
    HAL_TIM_PWM_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htimx_BDCMOTOR1,TIM_CHANNEL_1);         // 停止输出
  }
}

/* --------------------------------------------------------------------- */
/* 有刷直流电机 */

/**
  * 函数功能: BDCMOTOR2定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void BDCMOTOR2_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // 定时器时钟
  TIM_OC_InitTypeDef sConfigOC;                          // 定时器通道比较输出
  TIM_BreakDeadTimeConfigTypeDef  sBDTConfig;            // 定时器死区时间比较输出

  /* 基本定时器外设时钟使能 */
  BDCMOTOR2_TIM_RCC_CLK_ENABLE();

  /* 定时器基本环境配置 */
  htimx_BDCMOTOR2.Instance = BDCMOTOR2_TIMx;                                 // 定时器编号
  htimx_BDCMOTOR2.Init.Prescaler = BDCMOTOR2_TIM_PRESCALER;                  // 定时器预分频器
  htimx_BDCMOTOR2.Init.CounterMode = TIM_COUNTERMODE_UP;                  // 计数方向：向上计数
  htimx_BDCMOTOR2.Init.Period = BDCMOTOR2_TIM_PERIOD;                        // 定时器周期
  htimx_BDCMOTOR2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;              // 时钟分频
  htimx_BDCMOTOR2.Init.RepetitionCounter = BDCMOTOR2_TIM_REPETITIONCOUNTER;  // 重复计数器
  /* 初始化定时器比较输出环境 */
  HAL_TIM_PWM_Init(&htimx_BDCMOTOR2);

  /* 定时器时钟源配置 */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       // 使用内部时钟源
  HAL_TIM_ConfigClockSource(&htimx_BDCMOTOR2, &sClockSourceConfig);

  /* 死区刹车配置,配置有效电平是低电平 */
  sBDTConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE ;
  sBDTConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW ;
  sBDTConfig.BreakState = TIM_BREAK_DISABLE ;
  sBDTConfig.DeadTime = 0 ;
  sBDTConfig.LockLevel = TIM_LOCKLEVEL_OFF ;
  sBDTConfig.OffStateIDLEMode= TIM_OSSI_DISABLE ;
  sBDTConfig.OffStateRunMode = TIM_OSSR_ENABLE ;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx_BDCMOTOR2,&sBDTConfig);

  /* 定时器比较输出配置 */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;                  // 比较输出模式：PWM1模式
  sConfigOC.Pulse =  PWM_Duty2;                         // 占空比
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;          // 输出极性
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;        // 互补通道输出极性
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // 快速模式
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // 空闲电平
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // 互补通道空闲电平
  HAL_TIM_PWM_ConfigChannel(&htimx_BDCMOTOR2, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_Base_Start(&htimx_BDCMOTOR2);

    /* 启动定时器通道和互补通道PWM输出 */
  HAL_TIM_PWM_Start(&htimx_BDCMOTOR2,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR2,TIM_CHANNEL_1);

}

/**
  * 函数功能: 设置电机速度
  * 输入函数: Duty,输出脉冲占空比
  * 返 回 值: 无
  * 说    明: 无
  */
void SetMotor2Speed(int16_t Duty)
{
  __HAL_TIM_SetCompare(&htimx_BDCMOTOR2,TIM_CHANNEL_1,Duty);
}

/**
  * 函数功能: 设置电机转动方向
  * 输入函数: Dir,电机转动方向
  * 返 回 值: 无
  * 说    明: 无
  */
void SetMotor2Dir(int16_t Dir)
{
  if(Dir)
  {
    HAL_TIM_PWM_Start(&htimx_BDCMOTOR2,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR2,TIM_CHANNEL_1);         // 停止输出
  }
  else
  {
    HAL_TIM_PWM_Stop(&htimx_BDCMOTOR2,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htimx_BDCMOTOR2,TIM_CHANNEL_1);         // 停止输出
  }
}
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
