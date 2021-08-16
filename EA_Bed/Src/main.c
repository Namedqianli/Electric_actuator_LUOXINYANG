/**
  ******************************************************************************
  * 文件名程: main.c
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.1
  * 编写日期: 2018-03-06
  * 功    能: 2轴直流有刷电机_编码器测速
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  *
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "DCMotor/bsp_BDCMotor.h"
#include "key/bsp_key.h"
#include "encoder/bsp_encoder.h"
#include "usart/bsp_usartx.h"
/* 私有类型定义 --------------------------------------------------------------*/
#define ENCODER1_LINE     11           // 编码器线数
#define ENCODER1_SPEEDRATIO  270       // 电机减速比
#define ENCODER1_PPR         (ENCODER1_SPEEDRATIO*ENCODER1_LINE*4) // Pulse/r 每圈可捕获的脉冲数
#define ENCODER2_LINE     11           // 编码器线数
#define ENCODER2_SPEEDRATIO  270       // 电机减速比
#define ENCODER2_PPR         (ENCODER2_SPEEDRATIO*ENCODER2_LINE*4) // Pulse/r 每圈可捕获的脉冲数
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
__IO uint8_t  start_flag=0;
__IO uint16_t time_count=0;        // 时间计数，每1ms增加一(与滴定时器频率有关)
__IO int32_t CaptureNumber=0;     // 输入捕获数
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();                                     // 使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  // 设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        // 打开HSE
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    // 打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            // PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 // 8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               // 336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     // 2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 // USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用

 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                 // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();
    /* 串口初始化 */
  MX_USARTx_Init();
  /* 按键初始化 */
  KEY_GPIO_Init();

  /* 编码器初始化及使能编码器模式 */
  ENCODER1_TIMx_Init();
  ENCODER2_TIMx_Init();

  PWM_Duty1 = 500;
  PWM_Duty2 = 500;
  SetMotor1Speed( PWM_Duty1 );
  SetMotor2Speed( PWM_Duty1 );
  /* 高级控制定时器初始化并配置PWM输出功能 */
  BDCMOTOR1_TIMx_Init();
  BDCMOTOR2_TIMx_Init();
  start_flag = 1;
  /* 无限循环 */
  while (1)
  {
    /* 电机1加速  */
    if(KEY1_StateRead() == KEY_DOWN)
    {
      PWM_Duty1 += 100;
      if(PWM_Duty1 >= BDCMOTOR1_TIM_PERIOD)
      {
        PWM_Duty1 = BDCMOTOR1_DUTY_FULL;
      }
      SetMotor1Speed( PWM_Duty1 );
    }
    /* 电机1减速 */
    if(KEY2_StateRead() == KEY_DOWN)
    {
      PWM_Duty1 -= 100;
      if(PWM_Duty1 <= BDCMOTOR1_DUTY_ZERO)
      {
        PWM_Duty1 = BDCMOTOR1_DUTY_ZERO;
      }
      SetMotor1Speed( PWM_Duty1 );
    }
    /* 电机2加速  */
    if(KEY3_StateRead() == KEY_DOWN)
    {
      PWM_Duty2 += 100;
      if(PWM_Duty2 >= BDCMOTOR2_TIM_PERIOD)
      {
        PWM_Duty2 = BDCMOTOR2_DUTY_FULL;
      }
      SetMotor2Speed( PWM_Duty2 );
    }
    /* 电机2减速 */
    if(KEY4_StateRead() == KEY_DOWN)
    {
      PWM_Duty2 -= 100;
      if(PWM_Duty2 <= BDCMOTOR2_DUTY_ZERO)
      {
        PWM_Duty2 = BDCMOTOR2_DUTY_ZERO;
      }
      SetMotor2Speed( PWM_Duty2 );
    }
   /* 停止 */
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
  * 函数功能: 系统滴答定时器中断回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 每发生一次滴答定时器中断进入该回调函数一次
  */
void HAL_SYSTICK_Callback(void)
{
  if(start_flag) // 等待脉冲输出后才开始计时
  {
    time_count++;         // 每1ms自动增一
    if(time_count==1000)  // 1s
    {
      float Speed = 0;
      /* 读取编码器1数值 */
      CaptureNumber = ( int32_t )__HAL_TIM_GET_COUNTER(&htimx_Encoder1)+ENCODER1_OverflowCNT*65536;
      printf("Input1:%d \n",CaptureNumber);
      // 4 : 使用定时器编码器接口捕获AB相的上升沿和下降沿，一个脉冲*4.
      // 11：编码器线数(转速一圈输出脉冲数)
      // 270：电机减数比，内部电机转动圈数与电机输出轴转动圈数比，即减速齿轮比
      Speed = (float)CaptureNumber/ENCODER1_PPR;
      printf("电机1实际转动速度%0.2f r/s \n",Speed);
      ENCODER1_OverflowCNT = 0;
      __HAL_TIM_SET_COUNTER(&htimx_Encoder1,0);

      /* ------------------------------------------------------------- */
      CaptureNumber = ( int32_t )__HAL_TIM_GET_COUNTER(&htimx_Encoder2)+ENCODER2_OverflowCNT*65536;
      printf("Input2:%d \n",CaptureNumber);
      // 4 : 使用定时器编码器接口捕获AB相的上升沿和下降沿，一个脉冲*4.
      // 11：编码器线数(转速一圈输出脉冲数)
      // 270：电机减数比，内部电机转动圈数与电机输出轴转动圈数比，即减速齿轮比
      Speed = (float)CaptureNumber/ENCODER2_PPR;
      printf("电机2实际转动速度%0.2f r/s \n",Speed);
      ENCODER2_OverflowCNT = 0;
      __HAL_TIM_SET_COUNTER(&htimx_Encoder2,0);

      time_count=0;
    }
  }
}


/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
