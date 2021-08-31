#include "actuator/bsp_actuator.h"

#include <stdio.h>

#include "tim.h"
#include "gpio.h"
#include "global_config.h"

/* 私有变量 ------------------------------------------------------------------*/
__IO actuator_state_t actuator[3];

///**
//  * 函数功能: 系统滴答定时器中断回调函数
//  * 输入参数: 无
//  * 返 回 值: 无
//  * 说    明: 每发生一次滴答定时器中断进入该回调函数一次
//  */
//void HAL_SYSTICK_Callback(void)
//{
//	if(1 == actuator1_flag) {
//		uint32_t CaptureNumber = ( int32_t )__HAL_TIM_GET_COUNTER(&htimx_Encoder1);
//		if(CaptureNumber <= actuator1_target_pulse) {
//			HAL_TIM_PWM_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
//      HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
//			__HAL_TIM_SET_COUNTER(&htimx_Encoder1,0);
//			actuator1_flag = 0;
//		}
//	}
//	if(1 == actuator2_flag) {
//		uint32_t CaptureNumber = ( int32_t )__HAL_TIM_GET_COUNTER(&htimx_Encoder2);
//		if(CaptureNumber <= actuator2_target_pulse) {
//			HAL_TIM_PWM_Stop(&htimx_BDCMOTOR2,TIM_CHANNEL_1);
//      HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR2,TIM_CHANNEL_1);
//			__HAL_TIM_SET_COUNTER(&htimx_Encoder2,0);
//			actuator2_flag = 0;
//		}
//	}
//}

void SetActuatorDir(uint8_t index, uint8_t dir)
{
	uint8_t gpio_output = 0;
	
	StopRunning(index);
	actuator[index].dir = dir;
	HAL_Delay(100);
	if(dir == DIR_PUSH) {
		gpio_output = GPIO_PIN_RESET;
	} else {
		gpio_output = GPIO_PIN_SET;
	}
	switch (index) {
		case ACTUATOR1:
			HAL_GPIO_WritePin(ACTUATOR1_LINE1_GPIO_PORT, ACTUATOR1_LINE1_GPIO_PIN, gpio_output);
			HAL_GPIO_WritePin(ACTUATOR1_LINE2_GPIO_PORT, ACTUATOR1_LINE2_GPIO_PIN, gpio_output);
			break;
		case ACTUATOR2:
			HAL_GPIO_WritePin(ACTUATOR2_LINE1_GPIO_PORT, ACTUATOR2_LINE1_GPIO_PIN, gpio_output);
			HAL_GPIO_WritePin(ACTUATOR2_LINE2_GPIO_PORT, ACTUATOR2_LINE2_GPIO_PIN, gpio_output);
			break;
		case ACTUATOR3:
			HAL_GPIO_WritePin(ACTUATOR3_LINE1_GPIO_PORT, ACTUATOR3_LINE1_GPIO_PIN, gpio_output);
			HAL_GPIO_WritePin(ACTUATOR3_LINE2_GPIO_PORT, ACTUATOR3_LINE2_GPIO_PIN, gpio_output);
			break;
	}
	HAL_Delay(100);
}
void StartRunning(uint8_t index)
{
	actuator[index].is_running = true;
	switch (index) {
		case ACTUATOR1:
			HAL_GPIO_WritePin(ACTUATOR1_EN_GPIO_PORT, ACTUATOR1_EN_GPIO_PIN, GPIO_PIN_SET);
			break;
		case ACTUATOR2:
			HAL_GPIO_WritePin(ACTUATOR2_EN_GPIO_PORT, ACTUATOR2_EN_GPIO_PIN, GPIO_PIN_SET);
			break;
		case ACTUATOR3:
			HAL_GPIO_WritePin(ACTUATOR3_EN_GPIO_PORT, ACTUATOR3_EN_GPIO_PIN, GPIO_PIN_SET);
			break;
	}
}

void StopRunning(uint8_t index)
{
	actuator[index].is_running = false;
	switch (index) {
		case ACTUATOR1:
			HAL_GPIO_WritePin(ACTUATOR1_EN_GPIO_PORT, ACTUATOR1_EN_GPIO_PIN, GPIO_PIN_RESET);
			break;
		case ACTUATOR2:
			HAL_GPIO_WritePin(ACTUATOR2_EN_GPIO_PORT, ACTUATOR2_EN_GPIO_PIN, GPIO_PIN_RESET);
			break;
		case ACTUATOR3:
			HAL_GPIO_WritePin(ACTUATOR3_EN_GPIO_PORT, ACTUATOR3_EN_GPIO_PIN, GPIO_PIN_RESET);
			break;
	}
}

void ACTUATOR_Init()
{
	memset((void *)actuator, 0, sizeof(actuator));
	#if TYPE == TYPE_BED
		actuator[0].total_pulse = ACTUATOR_MAX_PULSE_150328;
		actuator[0].resolution = 22.3;
	#elif TYPE == TYPE_CHAIR
		actuator[0].total_pulse = ACTUATOR_MAX_PULSE_100278;
		actuator[0].resolution = 22.3;
		actuator[1].total_pulse = ACTUATOR_MAX_PULSE_100278;
		actuator[1].resolution = 22.3;
		actuator[2].total_pulse = ACTUATOR_MAX_PULSE_300499;
		actuator[2].resolution = 15.0;
	#endif
	actuator[0].handler = &htim1;
	actuator[1].handler = &htim4;
	actuator[2].handler = &htim8;
}

/**
  * 函数功能: 推杆移动xMM
  * 输入参数: 移动的距离
  * 返 回 值: 无
  * 说    明: 同步
  */
void ActuatorMoveMmSync(uint8_t index, int32_t length)
{
	if(length > 0) {
		actuator[index].dir = DIR_PUSH;			//SET dir
		//calculator pulse
		actuator[index].target_pulse = (uint32_t)(actuator[index].resolution*length);
	} else {
		actuator[index].dir = DIR_BACK;			//SET dir
		//calculator pulse
		actuator[index].target_pulse = (uint32_t)(actuator[index].resolution*length*-1);
	}
	actuator[index].target_pulse *= 4;
	#if DEBUG_INFO
		printf("TARGET PULSE: %d\n", actuator[index].target_pulse);
	#endif
	//修正,防止运动超出范围
	if(actuator[index].dir == DIR_PUSH) {
		actuator[index].target_pulse = 
			(actuator[index].target_pulse + actuator[index].now_pulse) < actuator[index].total_pulse ? actuator[index].target_pulse : 
				actuator[index].now_pulse < actuator[index].total_pulse ? 
					(actuator[index].total_pulse - actuator[index].now_pulse) : 0;
	} else if(actuator[index].dir == DIR_BACK) {
		actuator[index].target_pulse = 
			(int32_t)(actuator[index].now_pulse - actuator[index].target_pulse) >= 0 ? actuator[index].target_pulse : actuator[index].now_pulse;
	}
	#if DEBUG_INFO
		printf("AFTER PULSE: %d\n", actuator[index].target_pulse);
	#endif
	//电机运行
	if(actuator[index].dir == DIR_PUSH) {
		__HAL_TIM_SET_COUNTER(actuator[index].handler,0xFF00);
		while(__HAL_TIM_GET_COUNTER(actuator[index].handler) != 0xFF00);
		SetActuatorDir(index, DIR_PUSH);
		StartRunning(index);
		while(__HAL_TIM_GET_COUNTER(actuator[index].handler) > (0xFF00 - actuator[index].target_pulse)) {
			#if DEBUG_INFO
				printf("moto: %d target count: %d count: %d\n", 
					index,(0xFF00 - actuator[index].target_pulse), __HAL_TIM_GET_COUNTER(actuator[index].handler));
			#endif
			uint32_t pulse = __HAL_TIM_GET_COUNTER(actuator[index].handler);
			HAL_Delay(100);
			if(pulse == __HAL_TIM_GET_COUNTER(actuator[index].handler)) {
				break;
			}			
		}
		StopRunning(index);
		actuator[index].now_pulse += (0xFF00 - __HAL_TIM_GET_COUNTER(actuator[index].handler));
	} else if(actuator[index].dir == DIR_BACK && actuator[index].target_pulse) {
		__HAL_TIM_SET_COUNTER(actuator[index].handler,1000);
		while(__HAL_TIM_GET_COUNTER(actuator[index].handler) != 1000);
		SetActuatorDir(index, DIR_BACK);
		StartRunning(index);
		while(__HAL_TIM_GET_COUNTER(actuator[index].handler) < actuator[index].target_pulse + 1000) {
			#if DEBUG_INFO
				printf("moto: %d target_count: %d count: %d\n", 
					index, actuator[index].target_pulse, __HAL_TIM_GET_COUNTER(actuator[index].handler));
			#endif
			uint32_t pulse = __HAL_TIM_GET_COUNTER(actuator[index].handler);
			HAL_Delay(100);
			if(pulse == __HAL_TIM_GET_COUNTER(actuator[index].handler)) {
				break;
			}
		}
		StopRunning(index);
		actuator[index].now_pulse -= __HAL_TIM_GET_COUNTER(actuator[index].handler);
	}
	#if DEBUG_INFO
		printf("NOW PULSE: %d\n", actuator[index].now_pulse);
	#endif
}

//unused function
void ActuatorMoveMmAsync(uint8_t index, int32_t length);

void ActuatorStart(uint8_t index, uint8_t dir)
{
	switch(dir) {
		case DIR_PUSH:
			__HAL_TIM_SET_COUNTER(actuator[index].handler,0xFF00);
			break;
		case DIR_BACK:
			__HAL_TIM_SET_COUNTER(actuator[index].handler,1000);
			break;
		default:
			break;
	}
	SetActuatorDir(index, dir);
	StartRunning(index);
}

void ActuatorStop(uint8_t index)
{
	StopRunning(index);
	switch(actuator[index].dir) {
		case DIR_PUSH:
			actuator[index].total_pulse += (0xFF00 - __HAL_TIM_GET_COUNTER(actuator[index].handler));
			break;
		case DIR_BACK:
			actuator[index].total_pulse -= (__HAL_TIM_GET_COUNTER(actuator[index].handler) - 1000);
			break;
		default:
			break;
	}
}

void ActuatorMoveToMin(uint8_t index)
{
	uint32_t pulse = 0;
	
	SetActuatorDir(index, DIR_BACK);
	StartRunning(index);
	while(1) {
		pulse = __HAL_TIM_GET_COUNTER(actuator[index].handler);
		HAL_Delay(500);
		if(pulse == __HAL_TIM_GET_COUNTER(actuator[index].handler)) {
			break;
		}			
	}
	actuator[index].now_pulse = 0;
	StopRunning(index);
}
