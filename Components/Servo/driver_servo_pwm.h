/*
 * driver_servo_pwm.h
 *
 *  Created on: Sep 15, 2024
 *      Author: 14769
 */

#ifndef SERVO_DRIVER_SERVO_PWM_H_
#define SERVO_DRIVER_SERVO_PWM_H_

#include "tim.h"
#include "main.h"

typedef struct pwmservo_handle
{
	TIM_HandleTypeDef* port;
	uint32_t channel;
	uint32_t timclk_freq;
	uint32_t pwm_freq;
	float min_pwidth_us;
	float max_pwidth_us;
	float min_degree;
	float max_degree;

	float tic_per_us;
	float pwidth_per_degree;
}PWMServo_handle;

void pwmservo_init(PWMServo_handle* p);

void pwmservo_set_pwidth(PWMServo_handle* p, float us);

void pwmservo_set_degree(PWMServo_handle* p, float degree);

#endif /* SERVO_DRIVER_SERVO_PWM_H_ */
