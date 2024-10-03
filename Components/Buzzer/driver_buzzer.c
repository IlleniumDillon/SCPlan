/*
 * driver_buzzer.c
 *
 *  Created on: Sep 15, 2024
 *      Author: 14769
 */

#include "driver_buzzer.h"

typedef struct buzzer_handle
{
	const float* tone;
	size_t tone_size;
	size_t cur_size;
	uint32_t timer_clock_freq;
}Buzzer_handle;

Buzzer_handle buzzerHandle;

void _set_pwm_freq(float freq)
{
//	HAL_TIM_PWM_Stop(BUZZER_TIMER, BUZZER_CH);
	if (freq < 10)
	{
		__HAL_TIM_SET_AUTORELOAD(BUZZER_TIMER, 4294967295);
		__HAL_TIM_SET_COMPARE(BUZZER_TIMER, BUZZER_CH, 0);
		// __HAL_TIM_URS_ENABLE(BUZZER_TIMER);
		__HAL_TIM_SET_COUNTER(BUZZER_TIMER, 0);
		SET_BIT(BUZZER_TIMER->Instance->EGR, TIM_EGR_UG);
	}
	else
	{
		uint32_t autoreload, compare;
		autoreload = buzzerHandle.timer_clock_freq / freq;
		compare = autoreload / 2;
		__HAL_TIM_SET_AUTORELOAD(BUZZER_TIMER, autoreload);
		__HAL_TIM_SET_COMPARE(BUZZER_TIMER, BUZZER_CH, compare);
		__HAL_TIM_SET_COUNTER(BUZZER_TIMER, 0);
		SET_BIT(BUZZER_TIMER->Instance->EGR, TIM_EGR_UG);
	}
//	HAL_TIM_PWM_Start(BUZZER_TIMER, BUZZER_CH);
}

void buzzer_init()
{
	buzzerHandle.timer_clock_freq = 275000000;
	buzzer_on();
	HAL_TIM_Base_Start(BUZZER_TIMER);
}

void buzzer_on()
{
	_set_pwm_freq(0);
	buzzerHandle.cur_size = 0;
	buzzerHandle.tone = NULL;
	buzzerHandle.tone_size = 0;
	HAL_TIM_PWM_Start(BUZZER_TIMER, BUZZER_CH);
}

void buzzer_off()
{
	_set_pwm_freq(0);
	buzzerHandle.cur_size = 0;
	buzzerHandle.tone = NULL;
	buzzerHandle.tone_size = 0;
	HAL_TIM_PWM_Stop(BUZZER_TIMER, BUZZER_CH);
}

void buzzer_play(Buzzer_tone tone)
{
	buzzerHandle.tone = tone.music;
	buzzerHandle.tone_size = tone.size;
	buzzerHandle.cur_size = 0;
}

void buzzer_update()
{
	if (buzzerHandle.cur_size >= buzzerHandle.tone_size)
	{
		_set_pwm_freq(0);
		buzzerHandle.cur_size = 0;
		buzzerHandle.tone = NULL;
		buzzerHandle.tone_size = 0;
	}
	else
	{
		_set_pwm_freq(buzzerHandle.tone[buzzerHandle.cur_size]);
		buzzerHandle.cur_size++;
	}
}
