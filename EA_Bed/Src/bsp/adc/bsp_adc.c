/**
  ******************************************************************************
  * 文件名程: bsp_adc.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: 板载ADC电压采集底层驱动程序
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
#include "adc/bsp_adc.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
ADC_HandleTypeDef hadcx;
DMA_HandleTypeDef hdma_adcx;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
int32_t AD_16Hex_Calibration(void);
int32_t ADC_Calibration(void);
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: AD转换初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void MX_ADCx_Init(void)
{ 
  ADC_ChannelConfTypeDef sConfig;
  ADC_AnalogWDGConfTypeDef AWDGConfig;
 
  /* 外设时钟使能 */
  ADCx_RCC_CLK_ENABLE();
 
  hadcx.Instance = ADCx;
  hadcx.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadcx.Init.Resolution = ADC_RESOLUTION_12B;
  hadcx.Init.ScanConvMode = DISABLE;
  hadcx.Init.ContinuousConvMode = ENABLE;
  hadcx.Init.DiscontinuousConvMode = DISABLE;
  hadcx.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadcx.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadcx.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadcx.Init.NbrOfConversion = 1;
  hadcx.Init.DMAContinuousRequests = ENABLE;
  hadcx.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadcx);
  
  /* 配置电流采样通道 */
  sConfig.Channel = ADC_CURRENT_CHANNEL;
  sConfig.Offset = 0;
  sConfig.Rank = 0x01;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadcx,&sConfig);

  /* 配置总线电压采集 */
  /* 模拟看门狗配置 */
  AWDGConfig.Channel = ADC_CHANNEL_6;
  AWDGConfig.HighThreshold = 0x0F00;    // 0x0F00; 对应60V,3840
  AWDGConfig.LowThreshold = 0x02D0;     // 0x02D0; 对应12V,720
  AWDGConfig.ITMode = ENABLE;
  AWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AWDGConfig.WatchdogNumber = 0; // not use
  HAL_ADC_AnalogWDGConfig(&hadcx,&AWDGConfig);
  
  sConfig.Channel = ADC_VOLT_CHANNEL;
  sConfig.Offset = 0;
  sConfig.Rank = 0x02;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadcx,&sConfig);
  
  HAL_NVIC_SetPriority(ADC_OVP_IRQx, 0, 1);
  HAL_NVIC_EnableIRQ(ADC_OVP_IRQx); 

	/* 初始化ADC_DMA */
	MX_DMA_Init();
 
}

void MX_DMA_Init(void) 
{ 
  /* 使能外设时钟 */
  DMAx_RCC_CLK_ENABLE();
  hdma_adcx.Instance = DMAx_Stream_x;
  hdma_adcx.Init.Channel = DMAx_CHANNEL_x;
  hdma_adcx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adcx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adcx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adcx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adcx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adcx.Init.Mode = DMA_CIRCULAR;
  hdma_adcx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_adcx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_adcx);
  
  __HAL_LINKDMA(&hadcx,DMA_Handle,hdma_adcx);
  
  /* 外设中断优先级配置和使能中断 */
  HAL_NVIC_SetPriority(ADCx_DMA_IRQx, 1, 1);
  HAL_NVIC_EnableIRQ(ADCx_DMA_IRQx); 
}

/**
  * 函数功能: ADC外设初始化配置
  * 输入参数: hadc：AD外设句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADCx)
  {
    /* AD转换通道引脚时钟使能 */
    ADC_CUR_GPIO_ClK_ENABLE();
    
    /* AD转换通道引脚初始化 */
    GPIO_InitStruct.Pin = ADC_CUR_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_CUR_GPIO, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = ADC_VOLT_GPIO_PIN;
    HAL_GPIO_Init(ADC_VOLT_GPIO, &GPIO_InitStruct);
  }
}

/**
  * 函数功能: ADC外设反初始化配置
  * 输入参数: hadc：AD外设句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADCx)
  {
    /* 禁用ADC外设时钟 */
    ADCx_RCC_CLK_DISABLE();
  
    /* AD转换通道引脚反初始化 */
    HAL_GPIO_DeInit(ADC_CUR_GPIO, ADC_CUR_GPIO_PIN);
    HAL_GPIO_DeInit(ADC_VOLT_GPIO, ADC_VOLT_GPIO_PIN);

    /* 禁用外设中断 */
    HAL_DMA_DeInit(hadc->DMA_Handle);
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
  }
} 


/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

