/*
 * driver_servo_pwm.c
 *
 *  Created on: Sep 15, 2024
 *      Author: 14769
 */

#include "driver_servo_pwm.h"

void pwmservo_init(PWMServo_handle* p)
{
	p->pwidth_per_degree = (p->max_pwidth_us - p->min_pwidth_us) / (p->max_degree - p->min_degree);

	uint32_t prescale = 0, autoreload = 0;
	do
	{
		prescale++;
		autoreload = p->timclk_freq / prescale / p->pwm_freq;
	}while (autoreload > 65536);
	prescale--;
	autoreload--;

	__HAL_TIM_SET_PRESCALER(p->port, prescale);
	__HAL_TIM_SET_AUTORELOAD(p->port, autoreload);
	p->tic_per_us = 1.0 / (1000000.0 / p->timclk_freq * (prescale + 1));

	pwmservo_set_degree(p, 0);

	HAL_TIM_Base_Start(p->port);
	HAL_TIM_PWM_Start(p->port, p->channel);
}

void pwmservo_set_pwidth(PWMServo_handle* p, float us)
{
	if (us < p->min_pwidth_us) us = p->min_pwidth_us;
	if (us > p->max_pwidth_us) us = p->max_pwidth_us;
	uint32_t compare = us * p->tic_per_us;
	__HAL_TIM_SET_COMPARE(p->port, p->channel, compare);
}

void pwmservo_set_degree(PWMServo_handle* p, float degree)
{
	if (degree > p->max_degree) degree = p->max_degree;
	if (degree > p->min_degree) degree = p->min_degree;
	float pwidth = p->min_pwidth_us + degree * p->pwidth_per_degree;
	pwmservo_set_pwidth(p, pwidth);
}
