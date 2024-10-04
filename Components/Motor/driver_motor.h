/*
 * driver_motor.h
 *
 *  Created on: Sep 15, 2024
 *      Author: 14769
 */

#ifndef MOTOR_DRIVER_MOTOR_H_
#define MOTOR_DRIVER_MOTOR_H_

#include "tim.h"
#include "main.h"

#include "driver_motor_ctrl.h"

typedef struct
{
	TIM_HandleTypeDef* oModule;
	uint32_t hoChannel;
	uint32_t loChannel;
	TIM_HandleTypeDef* iModule;
	float pwm_freq;

	uint32_t lines;
	float reduction;
	float radius_m;
	float sampleTime_s;
	float mileage_m;

	uint8_t reverse;

	PID ctrl;
}EncoderMotorDev;

#define TIM_ARRMAX (4294967295)
#define TIM_SRCFREQ (275000000)

void em_init(EncoderMotorDev* dev);
void em_setPWM(EncoderMotorDev* dev, float duty);
float em_getSpeed(EncoderMotorDev* dev);
void em_setSpeedTarget(EncoderMotorDev* dev, float speed);
void em_update(EncoderMotorDev* dev);

#endif /* MOTOR_DRIVER_MOTOR_H_ */
