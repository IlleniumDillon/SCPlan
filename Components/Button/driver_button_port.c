/*
 * driver_button_port.c
 *
 *  Created on: Sep 17, 2024
 *      Author: 14769
 */

#include <Button/driver_button_port.h>
#include "gpio.h"
#include "main.h"

flex_button_t user_button[USER_BUTTON_MAX];

uint8_t btn_read(void *arg)
{
	uint8_t value = 0;

	flex_button_t *btn = (flex_button_t *)arg;

	switch (btn->id)
	{
	case USER_BUTTON_UP:
		value = HAL_GPIO_ReadPin(SW_A_GPIO_Port, SW_A_Pin);
		break;
	case USER_BUTTON_DOWN:
		value = HAL_GPIO_ReadPin(SW_B_GPIO_Port, SW_B_Pin);
		break;
	case USER_BUTTON_LEFT:
		value = HAL_GPIO_ReadPin(SW_C_GPIO_Port, SW_C_Pin);
		break;
	case USER_BUTTON_RIGHT:
		value = HAL_GPIO_ReadPin(SW_D_GPIO_Port, SW_D_Pin);
		break;
	case USER_BUTTON_OK:
		value = HAL_GPIO_ReadPin(SW_CEN_GPIO_Port, SW_CEN_Pin);
		break;
	}

	return value;
}

