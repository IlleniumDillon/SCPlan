/*
 * driver_motor.c
 *
 *  Created on: Sep 15, 2024
 *      Author: 14769
 */

#include "driver_motor.h"

#include <math.h>

#define PI (3.14159265358979323846)

void em_init(EncoderMotorDev* dev)
{
	if(dev->reverse)
	{
		uint32_t temp = dev->hoChannel;
		dev->hoChannel = dev->loChannel;
		dev->loChannel = temp;
	}
	dev->mileage_m = 0;
	HAL_TIM_PWM_Start(dev->oModule,dev->hoChannel);
	HAL_TIM_PWM_Start(dev->oModule,dev->loChannel);

	dev->iModule->Instance->CNT = 0x7FFF;
	HAL_TIM_Base_Start(dev->iModule);
}
void em_setPWM(EncoderMotorDev* dev, float duty)
{
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(dev->oModule);
	if(duty > 0)
	{
		__HAL_TIM_SET_COMPARE(dev->oModule, dev->hoChannel, arr);
		__HAL_TIM_SET_COMPARE(dev->oModule, dev->loChannel, arr * (1 - duty));
	}
	else
	{
		__HAL_TIM_SET_COMPARE(dev->oModule, dev->hoChannel, arr * (1 + duty));
		__HAL_TIM_SET_COMPARE(dev->oModule, dev->loChannel, arr);
	}
}
float em_getSpeed(EncoderMotorDev* dev)
{
	int cnt;
	cnt = dev->iModule->Instance->CNT - 0x7FFF;
	dev->iModule->Instance->CNT = 0x7FFF;
	if(dev->reverse)
	{
		cnt = -cnt;
	}
	float arc = (float)cnt / dev->lines * 2 * PI / dev->reduction;
	float delta = arc * dev->radius_m;
	dev->mileage_m += delta;
	return delta / dev->sampleTime_s;
}

void em_setSpeedTarget(EncoderMotorDev* dev, float speed)
{
	pid_setTarget(&dev->ctrl, speed);
}

void em_update(EncoderMotorDev* dev)
{
	em_setPWM(dev, pid_update(&dev->ctrl, em_getSpeed(dev)));
}
