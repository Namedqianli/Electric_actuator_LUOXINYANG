#ifndef __BSP_ACTUATOR_H__
#define __BSP_ACTUATOR_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "encoder/bsp_encoder.h"
#include "DCMotor/bsp_BDCMotor.h"
/* 私有类型定义 --------------------------------------------------------------*/
typedef struct 
{
	uint32_t total_plus;
	uint32_t is_running;
	uint32_t target_plus;
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
#define ACTUATOR_MAX_LENGTH		100	//mm
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
void ActuatorMoveMmSync(uint8_t index, int32_t length);
void ActuatorMoveMmAsync(uint8_t index, int32_t length);
void ActuatorStart(uint8_t index, uint8_t dir);
void ActuatorStop(uint8_t index);

#endif
