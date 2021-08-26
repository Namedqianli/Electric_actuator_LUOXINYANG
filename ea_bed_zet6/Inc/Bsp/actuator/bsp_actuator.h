#ifndef __BSP_ACTUATOR_H__
#define __BSP_ACTUATOR_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "tim.h"
/* 私有类型定义 --------------------------------------------------------------*/
typedef struct 
{
	uint32_t now_pulse;
	uint32_t total_pulse;
	uint32_t is_running;
	uint32_t dir;
	uint32_t target_pulse;
	float resolution;
	TIM_HandleTypeDef *handler;
} actuator_state_t;

typedef enum
{
	ACTUATOR1 = 0,
	ACTUATOR2 = 1,
	ACTUATOR3 = 2,
} actuator_device_t;

typedef enum
{
	DIR_PUSH,
	DIR_BACK,
} actuator_dir_t;
/* 私有宏定义 ----------------------------------------------------------------*/
#define ACTUATOR_MAX_LENGTH_300499						300	//mm
#define ACTUATOR_MAX_LENGTH_150328						150	//mm
#define ACTUATOR_MAX_LENGTH_100278						100	//mm

#define ACTUATOR_MAX_PULSE_300499							(4500*4)
#define ACTUATOR_MAX_PULSE_150328							(3343*4)
#define ACTUATOR_MAX_PULSE_100278							(2229*4)

#define ACTUATOR1_EN_GPIO_PORT								GPIOB
#define ACTUATOR1_EN_GPIO_PIN									GPIO_PIN_0
#define ACTUATOR1_LINE1_GPIO_PORT							GPIOF
#define ACTUATOR1_LINE1_GPIO_PIN							GPIO_PIN_12
#define ACTUATOR1_LINE2_GPIO_PORT							GPIOF
#define ACTUATOR1_LINE2_GPIO_PIN							GPIO_PIN_13

#define ACTUATOR2_EN_GPIO_PORT								GPIOF
#define ACTUATOR2_EN_GPIO_PIN									GPIO_PIN_14
#define ACTUATOR2_LINE1_GPIO_PORT							GPIOF
#define ACTUATOR2_LINE1_GPIO_PIN							GPIO_PIN_15
#define ACTUATOR2_LINE2_GPIO_PORT							GPIOD
#define ACTUATOR2_LINE2_GPIO_PIN							GPIO_PIN_14

#define ACTUATOR3_EN_GPIO_PORT								GPIOD
#define ACTUATOR3_EN_GPIO_PIN									GPIO_PIN_15
#define ACTUATOR3_LINE1_GPIO_PORT							GPIOF
#define ACTUATOR3_LINE1_GPIO_PIN							GPIO_PIN_0
#define ACTUATOR3_LINE2_GPIO_PORT							GPIOF
#define ACTUATOR3_LINE2_GPIO_PIN							GPIO_PIN_1
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
void SetActuatorDir(uint8_t index, uint8_t dir);
void StartRunning(uint8_t index);
void StopRunning(uint8_t index);
/* 函数体 --------------------------------------------------------------------*/
void ACTUATOR_Init();
void ActuatorMoveMmSync(uint8_t index, int32_t length);
void ActuatorMoveMmAsync(uint8_t index, int32_t length);
void ActuatorStart(uint8_t index, uint8_t dir);
void ActuatorStop(uint8_t index);

#endif
