/*
 * board_servo.c
 *
 *  Created on: Sep 15, 2024
 *      Author: 14769
 */

#include "board_servo.h"

PWMServo_handle tripod_handle =
{
		.port = &htim16,
		.channel = TIM_CHANNEL_1,
		.timclk_freq = 275000000,
		/// https://category.yahboom.net/collections/a-servo/products/sg90-servo
		.pwm_freq = 50,
		.min_degree = -90,
		.max_degree = 90,
		.min_pwidth_us = 500,
		.max_pwidth_us = 2500
};

BusServo_dev_handle devs[2] =
{
		{
				.id = 7
		},
		{
				.id = 8
		}
};

BusServo_bus_handle arm_handle =
{
		.num = 2,
		.pdev = devs,
		.port = &huart1
};
