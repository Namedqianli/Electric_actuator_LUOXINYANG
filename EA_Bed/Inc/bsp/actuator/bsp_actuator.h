#ifndef __BSP_ACTUATOR_H__
#define __BSP_ACTUATOR_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "encoder/bsp_encoder.h"
#include "DCMotor/bsp_BDCMotor.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
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
/* ˽�к궨�� ----------------------------------------------------------------*/
#define ACTUATOR_MAX_LENGTH		100	//mm
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
void ActuatorMoveMmSync(uint8_t index, int32_t length);
void ActuatorMoveMmAsync(uint8_t index, int32_t length);
void ActuatorStart(uint8_t index, uint8_t dir);
void ActuatorStop(uint8_t index);

#endif
