#include "actuator/bsp_actuator.h"

/* ˽�б��� ------------------------------------------------------------------*/
__IO actuator_state_t actuator[3];

///**
//  * ��������: ϵͳ�δ�ʱ���жϻص�����
//  * �������: ��
//  * �� �� ֵ: ��
//  * ˵    ��: ÿ����һ�εδ�ʱ���жϽ���ûص�����һ��
//  */
//void HAL_SYSTICK_Callback(void)
//{
//	if(1 == actuator1_flag) {
//		uint32_t CaptureNumber = ( int32_t )__HAL_TIM_GET_COUNTER(&htimx_Encoder1);
//		if(CaptureNumber <= actuator1_target_plus) {
//			HAL_TIM_PWM_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
//      HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
//			__HAL_TIM_SET_COUNTER(&htimx_Encoder1,0);
//			actuator1_flag = 0;
//		}
//	}
//	if(1 == actuator2_flag) {
//		uint32_t CaptureNumber = ( int32_t )__HAL_TIM_GET_COUNTER(&htimx_Encoder2);
//		if(CaptureNumber <= actuator2_target_plus) {
//			HAL_TIM_PWM_Stop(&htimx_BDCMOTOR2,TIM_CHANNEL_1);
//      HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR2,TIM_CHANNEL_1);
//			__HAL_TIM_SET_COUNTER(&htimx_Encoder2,0);
//			actuator2_flag = 0;
//		}
//	}
//}

void ActuatorInit()
{
	#if TYPE == TYPE_BED
		memset((void *)actuator, 0, sizeof(actuator));
	#elif TYPE == TYPE_CHAIR
		
	#endif
}

/**
  * ��������: �Ƹ��ƶ�xMM
  * �������: �ƶ��ľ���
  * �� �� ֵ: ��
  * ˵    ��: ͬ��
  */
void ActuatorMoveMmSync(uint8_t index, int32_t length)
{
	__HAL_TIM_SET_COUNTER(&htimx_Encoder1,0);
	if(length > 0) {
		actuator[index].target_plus = (uint32_t)(22.3*length);
		SetMotor1Dir(1);
		SetMotor1Speed(BDCMOTOR1_DUTY_FULL);
	} else {
		actuator[index].target_plus = (uint32_t)(22.3*length) * -1;
		SetMotor1Dir(0);
		SetMotor1Speed(BDCMOTOR1_DUTY_FULL);
	}
	actuator[index].target_plus *= 2;
	while(__HAL_TIM_GET_COUNTER(&htimx_Encoder1) < actuator[index].target_plus);
	HAL_TIM_PWM_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR1,TIM_CHANNEL_1);
	__HAL_TIM_SET_COUNTER(&htimx_Encoder1,0);
}


void ActuatorMoveMmAsync(uint8_t index, int32_t length);
void ActuatorStart(uint8_t index, uint8_t dir);
void ActuatorStop(uint8_t index);
