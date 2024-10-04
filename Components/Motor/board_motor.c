/*
 * board_motor.c
 *
 *  Created on: Sep 17, 2024
 *      Author: 14769
 */

#include "board_motor.h"

EncoderMotorDev leftWheel =
		{
				.oModule = &htim23,
				.hoChannel = TIM_CHANNEL_1,
				.loChannel = TIM_CHANNEL_2,
				.iModule = &htim3,
				.pwm_freq = 20000,
				.lines = 11 * 2,
				.reduction = 56,
				.radius_m = 0.0282,
				.reverse = 1,
				.sampleTime_s = 0.02,
		};
EncoderMotorDev rightWheel =
		{
				.oModule = &htim23,
				.hoChannel = TIM_CHANNEL_3,
				.loChannel = TIM_CHANNEL_4,
				.iModule = &htim4,
				.pwm_freq = 20000,
				.lines = 11 * 2,
				.reduction = 56,
				.radius_m = 0.0282,
				.reverse = 0,
				.sampleTime_s = 0.02,
		};
