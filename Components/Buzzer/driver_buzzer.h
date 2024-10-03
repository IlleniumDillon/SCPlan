/*
 * driver_buzzer.h
 *
 *  Created on: Sep 15, 2024
 *      Author: 14769
 */

#ifndef BUZZER_DRIVER_BUZZER_H_
#define BUZZER_DRIVER_BUZZER_H_

#include "tim.h"
#include "main.h"

#include "sound.h"

#define BUZZER_TIMER (&htim24)
#define BUZZER_CH (TIM_CHANNEL_4)

// extern Buzzer_handle buzzerHandle;

void buzzer_init();

void buzzer_on();

void buzzer_off();

void buzzer_play(Buzzer_tone tone);

void buzzer_update();

void _set_pwm_freq(float freq);

#endif /* BUZZER_DRIVER_BUZZER_H_ */
