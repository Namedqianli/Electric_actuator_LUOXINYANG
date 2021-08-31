/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "nrf24l01/bsp_nrf24l01.h"
#include "actuator/bsp_actuator.h"
#include "key/bsp_key.h"
#include "global_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum 
{
	SIGNAL_BACK_UP				= 0x01,				//Ãß±≥
	SIGNAL_BACK_DOWN			= 0x02,				//Ωµ±≥
	SIGNAL_FOOT_UP				= 0x03,				//ÃßÕ»
	SIGNAL_FOOT_DOWN			= 0x04,				//ΩµÕ»
	SIGNAL_ROTATE_CHAIR		= 0x05,				//≤‡◊¯œ¬¥≤£¨–˝◊™≥…“Œ
	SIGNAL_ROTATE_BED			= 0x06,				//…œ¥≤∆ΩÃ…
	SIGNAL_PART_MOVE			= 0x07,				//∑÷¿Î“∆∂Ø
	SIGNAL_RESET					= 0x08,				//“ªº¸∆ΩÃ…	
	SIGNAL_MAX						= 0x09,
} receive_signal_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOCK()					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
#define UNLOCK()				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__IO uint16_t time_count				= 0;
uint8_t recv_signal 						= 0;
__IO uint8_t now_state 					= 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	__IO uint8_t pre_recv_signal		= SIGNAL_MAX;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	NRF24L01_Init();
	NRF24L01_RX_Mode();
	while(NRF24L01_Check()) {
		printf("check error!\n");
	}
	printf("successed\n");
	ACTUATOR_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(NRF24L01_RxPacket(&recv_signal) != 1) {
			//Ω” ’µΩ–≈∫≈
			#if DEBUG_INFO
				printf("pre: %d recv: %d\n", pre_recv_signal, recv_signal);
			#endif
			#if TYPE == TYPE_BED
				switch (recv_signal) {
	//					case SIGNAL_BACK_UP:
	//						time_count = 0;
	//						ActuatorStart(ACTUATOR1, DIR_PUSH);
	//						break;
	//					case SIGNAL_BACK_DOWN:
	//						time_count = 0;
	//						ActuatorStart(ACTUATOR1, DIR_BACK);
	//						break;
					case SIGNAL_RESET:
						//goto 0
						ActuatorMoveToMin(ACTUATOR1);
						break;
					case SIGNAL_ROTATE_CHAIR:
						//actuator move xmm
						ActuatorMoveMmSync(ACTUATOR1, 20);
						break;
					case SIGNAL_ROTATE_BED:
						ActuatorMoveMmSync(ACTUATOR1, -20);
						break;
					case SIGNAL_PART_MOVE:
						//actuator move xmm
						//ulock
						UNLOCK();
						break;
					default:
						break;
				}
				now_state = recv_signal;
			#elif TYPE == TYPE_CHAIR
				switch (recv_signal) {
					case SIGNAL_BACK_UP:
						//Ãß±≥
						#if DEBUG_INFO
							printf("Start Moto1 pre:%d recv:%d\n", pre_recv_signal, recv_signal);
						#endif
						time_count = 0;
						if(pre_recv_signal == SIGNAL_BACK_UP) {
							break;
						}
						ActuatorStart(ACTUATOR1, DIR_PUSH);
						break;
					case SIGNAL_BACK_DOWN:
						//Ωµ±≥
						time_count = 0;
						if(pre_recv_signal == SIGNAL_BACK_DOWN) {
							break;
						}
						ActuatorStart(ACTUATOR1, DIR_BACK);
						break;
					case SIGNAL_FOOT_UP:
						//ÃßÕ»
						time_count = 0;
						if(pre_recv_signal == SIGNAL_FOOT_UP) {
							break;
						}
						ActuatorStart(ACTUATOR2, DIR_PUSH);
						break;
					case SIGNAL_FOOT_DOWN:
						//ΩµÕ»
						time_count = 0;
						if(pre_recv_signal == SIGNAL_FOOT_DOWN) {
							break;
						}
						ActuatorStart(ACTUATOR2, DIR_BACK);
						break;
					case SIGNAL_RESET:
						//goto 0
						break;
					case SIGNAL_ROTATE_CHAIR:
						/* –˝◊™≥…“Œ */
						//actuator move xmm
						if(now_state != SIGNAL_ROTATE_CHAIR && now_state != SIGNAL_PART_MOVE) {
							//Ãß±≥75
							ActuatorMoveMmSync(ACTUATOR1, 10);
							//ÃßÕ»15
							ActuatorMoveMmSync(ACTUATOR2, 10);
							//–˝◊™90£®ƒÊ ±’Î£©
							ActuatorMoveMmSync(ACTUATOR3, 10);
							//¥ÛÕ»0£¨–°Õ»80
							ActuatorMoveMmSync(ACTUATOR2, -10);
							now_state = SIGNAL_ROTATE_CHAIR;
						}
						break;
					case SIGNAL_ROTATE_BED:
						/* ∆ΩÃ…£®∏¥‘≠£© */
						//«˙Õ» 0, π¥ÛÕ»∞Â°¢Ω≈Ã§∞Â…œ…˝÷¡15°„
						ActuatorMoveMmSync(ACTUATOR2, 15);
						//–˝◊™90£®À≥ ±’Î£©
						ActuatorMoveMmSync(ACTUATOR3, -10);
						//∑÷¿ÎÕ∆∏À£¨œÚ¥≤∆Ω“∆
						// π¥ÛÕ»∞Â°¢Ω≈Ã§∞ÂÃß∏¥‘≠÷¡0°„
						ActuatorMoveMmSync(ACTUATOR2, -15);
						break;
					case SIGNAL_PART_MOVE:
						if(now_state == SIGNAL_ROTATE_CHAIR && now_state == SIGNAL_MAX) {
							//–˝◊™≥…“Œ
							//Ãß±≥75
							ActuatorMoveMmSync(ACTUATOR1, 10);
							//ÃßÕ»15
							ActuatorMoveMmSync(ACTUATOR2, 10);
							//–˝◊™90£®ƒÊ ±’Î£©
							ActuatorMoveMmSync(ACTUATOR3, 10);
							//¥ÛÕ»0£¨–°Õ»80
							ActuatorMoveMmSync(ACTUATOR2, -10);
							//∑÷¿Î
							//ActuatorMoveMmSync();
							now_state = SIGNAL_PART_MOVE;
						}
						break;
					default:
						break;
				}
				#if DEBUG_INFO
					printf("pre1:%d\n", pre_recv_signal);
				#endif
				pre_recv_signal = recv_signal;
				#if DEBUG_INFO
					printf("pre2:%d\n", pre_recv_signal);
				#endif
			#endif
		}
		#if TYPE == TYPE_BED
			if(now_state == SIGNAL_BACK_UP || now_state == SIGNAL_BACK_UP) {
				HAL_Delay(1);
				time_count++;
				if(time_count == 100) {
					time_count = 0;
					ActuatorStop(ACTUATOR1);
				}
			}
		#elif TYPE == TYPE_CHAIR
			if(pre_recv_signal == SIGNAL_BACK_UP || pre_recv_signal == SIGNAL_BACK_DOWN ||
					pre_recv_signal == SIGNAL_FOOT_UP || pre_recv_signal == SIGNAL_FOOT_DOWN) {
				HAL_Delay(1);
				time_count++;
				if(time_count == 100) {
					#if DEBUG_INFO
						printf("Stop Running!\n");
					#endif
					pre_recv_signal = SIGNAL_MAX;
					time_count = 0;
					ActuatorStop(ACTUATOR1);
					ActuatorStop(ACTUATOR2);
				}
			}
		#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
