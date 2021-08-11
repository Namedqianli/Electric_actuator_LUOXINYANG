/**
  ******************************************************************************
  * 文件名程: main.c
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2018-01-13
  * 功    能: 有刷直流电机位置闭环控制_位置式PID
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
#include "key/bsp_key.h"
#include "encoder/bsp_encoder.h"
#include "usart/bsp_usartx.h"
#include "DCMotor/bsp_BDCMotor.h"
#include "led/bsp_led.h"
#include "nrf24l01/nrf24l01.h"
/* 私有类型定义 --------------------------------------------------------------*/
typedef struct
{
  __IO int32_t  SetPoint;                                 //设定目标 Desired Value
  __IO float    SumError;                                //误差累计
  __IO float    Proportion;                               //比例常数 Proportional Const
  __IO float    Integral;                                 //积分常数 Integral Const
  __IO float    Derivative;                               //微分常数 Derivative Const
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

/* 私有宏定义 ----------------------------------------------------------------*/

/*************************************/
// 定义PID相关宏
// 这三个参数设定对电机运行影响非常大
// PID参数跟采样时间息息相关
/*************************************/
#define  SPD_P_DATA      1.025f        // P参数
#define  SPD_I_DATA      0.215f        // I参数
#define  SPD_D_DATA      0.1f         // D参数
#define  TARGET_LOC    11880        // 目标速度    1 r

/* 私有变量 ------------------------------------------------------------------*/
__IO uint8_t  Start_flag = 0;       // PID 开始标志
__IO uint32_t Motor_Dir = CW;             // 电机方向
__IO int32_t Loc_Pulse;              // 编码器捕获值 Pulse
uint8_t Bed_status = 0;
uint8_t Is_running = 0;

/* 扩展变量 ------------------------------------------------------------------ */
extern __IO uint32_t uwTick;

/* PID结构体 */
PID_TypeDef  sPID;               // PID参数结构体

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
void PID_ParamInit(void) ;
int32_t LocPIDCalc(int32_t NextPoint);
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
	uint8_t nrf_rx_buf[16] = {0};
	
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();
  /* 串口初始化 */
  MX_USARTx_Init();
	/* NRF24L01初始化 */
	NRF24L01_Init();
	NRF24L01_RX_Mode();
  /* LED初始化 */
  LED_GPIO_Init();
  /* 按键初始化 */
  KEY_GPIO_Init();
  /* 编码器初始化及使能编码器模式 */
  ENCODER_TIMx_Init();
	/* 高级控制定时器初始化并配置PWM输出功能 */
  BDCMOTOR_TIMx_Init();
  /* 设定占空比 */
  PWM_Duty = 0;
  SetMotorSpeed(PWM_Duty);  // 0%
  /* PID 参数初始化 */
  PID_ParamInit();
  /* 无限循环 */
  while (1)
  {
//    /* 停止按钮 */
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
//      HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR,TIM_CHANNEL_1);         // 停止输出
//    }
//    if(KEY3_StateRead()==KEY_DOWN)//加速
//    {
//      sPID.SetPoint += 11880; // +1 r
//    }
//    if(KEY4_StateRead()==KEY_DOWN)//减速
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
  * 函数功能: 系统滴答定时器中断回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 每发生一次滴答定时器中断进入该回调函数一次
  */
void HAL_SYSTICK_Callback(void)
{
  int32_t tmpPWM_Duty = 0;
  /* 速度环周期100ms */
  if(uwTick % 100 == 0)
  {
    Loc_Pulse = (OverflowCount*CNT_MAX) + (int32_t)__HAL_TIM_GET_COUNTER(&htimx_Encoder);

    /* 计算PID结果 */
    if(Start_flag == 1)
    {
      /* 限定速度 */
      PWM_Duty = LocPIDCalc(Loc_Pulse);
      if(PWM_Duty >= BDCMOTOR_DUTY_FULL/2)
        PWM_Duty = BDCMOTOR_DUTY_FULL/2;
      if(PWM_Duty <= -BDCMOTOR_DUTY_FULL/2)
        PWM_Duty = -BDCMOTOR_DUTY_FULL/2;

      /* 判断当前运动方向 */
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
      /* 输出PWM */
      SetMotorSpeed( tmpPWM_Duty );
    }
    printf(" Loc: %d (Pulse) = %.3f (r) \n",Loc_Pulse, (float)Loc_Pulse/11880.0f);
  }
}
/******************** PID 控制设计 ***************************/
/**
  * 函数功能: PID参数初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void PID_ParamInit()
{
    sPID.LastError = 0;               // Error[-1]
    sPID.PrevError = 0;               // Error[-2]
    sPID.Proportion = SPD_P_DATA; // 比例常数 Proportional Const
    sPID.Integral = SPD_I_DATA;   // 积分常数  Integral Const
    sPID.Derivative = SPD_D_DATA; // 微分常数 Derivative Const
    sPID.SetPoint = TARGET_LOC;     // 设定目标Desired Value
}
/**
  * 函数名称：位置闭环PID控制设计
  * 输入参数：当前控制量
  * 返 回 值：目标控制量
  * 说    明：无
  */
int32_t LocPIDCalc(int32_t NextPoint)
{
  int32_t iError,dError;
  iError = sPID.SetPoint - NextPoint; //偏差

  if( (iError<50) && (iError>-50) )
    iError = 0;

  /* 限定积分区域 */
  if((iError<400 )&& (iError>-400))
  {
    sPID.SumError += iError; //积分
    /* 设定积分上限 */
    if(sPID.SumError >= (TARGET_LOC*10))
       sPID.SumError  = (TARGET_LOC*10);
    if(sPID.SumError <= -(TARGET_LOC*10))
      sPID.SumError = -(TARGET_LOC*10);
  }

  dError = iError - sPID.LastError; //微分
  sPID.LastError = iError;
  return (int32_t)( (sPID.Proportion * (float)iError) //比例项
                    + (sPID.Integral * (float)sPID.SumError) //积分项
                    + (sPID.Derivative * (float)dError) ); //微分项
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
