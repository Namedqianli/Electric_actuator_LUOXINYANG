/**
  ******************************************************************************
  * �ļ�����: bsp_adc.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: ����ADC��ѹ�ɼ��ײ���������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "adc/bsp_adc.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
ADC_HandleTypeDef hadcx;
DMA_HandleTypeDef hdma_adcx;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
int32_t AD_16Hex_Calibration(void);
int32_t ADC_Calibration(void);
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ADת����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void MX_ADCx_Init(void)
{ 
  ADC_ChannelConfTypeDef sConfig;
  ADC_AnalogWDGConfTypeDef AWDGConfig;
 
  /* ����ʱ��ʹ�� */
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
  
  /* ���õ�������ͨ�� */
  sConfig.Channel = ADC_CURRENT_CHANNEL;
  sConfig.Offset = 0;
  sConfig.Rank = 0x01;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadcx,&sConfig);

  /* �������ߵ�ѹ�ɼ� */
  /* ģ�⿴�Ź����� */
  AWDGConfig.Channel = ADC_CHANNEL_6;
  AWDGConfig.HighThreshold = 0x0F00;    // 0x0F00; ��Ӧ60V,3840
  AWDGConfig.LowThreshold = 0x02D0;     // 0x02D0; ��Ӧ12V,720
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

	/* ��ʼ��ADC_DMA */
	MX_DMA_Init();
 
}

void MX_DMA_Init(void) 
{ 
  /* ʹ������ʱ�� */
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
  
  /* �����ж����ȼ����ú�ʹ���ж� */
  HAL_NVIC_SetPriority(ADCx_DMA_IRQx, 1, 1);
  HAL_NVIC_EnableIRQ(ADCx_DMA_IRQx); 
}

/**
  * ��������: ADC�����ʼ������
  * �������: hadc��AD����������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADCx)
  {
    /* ADת��ͨ������ʱ��ʹ�� */
    ADC_CUR_GPIO_ClK_ENABLE();
    
    /* ADת��ͨ�����ų�ʼ�� */
    GPIO_InitStruct.Pin = ADC_CUR_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_CUR_GPIO, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = ADC_VOLT_GPIO_PIN;
    HAL_GPIO_Init(ADC_VOLT_GPIO, &GPIO_InitStruct);
  }
}

/**
  * ��������: ADC���跴��ʼ������
  * �������: hadc��AD����������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADCx)
  {
    /* ����ADC����ʱ�� */
    ADCx_RCC_CLK_DISABLE();
  
    /* ADת��ͨ�����ŷ���ʼ�� */
    HAL_GPIO_DeInit(ADC_CUR_GPIO, ADC_CUR_GPIO_PIN);
    HAL_GPIO_DeInit(ADC_VOLT_GPIO, ADC_VOLT_GPIO_PIN);

    /* ���������ж� */
    HAL_DMA_DeInit(hadc->DMA_Handle);
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
  }
} 


/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

