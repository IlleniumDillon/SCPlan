/*
 * driver_electromagnet.h
 *
 *  Created on: Sep 15, 2024
 *      Author: 14769
 */

#ifndef ELECTROMAGNET_DRIVER_ELECTROMAGNET_H_
#define ELECTROMAGNET_DRIVER_ELECTROMAGNET_H_

#include "gpio.h"
#include "main.h"

static inline void emag_enable();
static inline void emag_disable();


static inline void emag_enable()
{
	HAL_GPIO_WritePin(MAG_SWITCH_GPIO_Port, MAG_SWITCH_Pin, GPIO_PIN_SET);
}
static inline void emag_disable()
{
	HAL_GPIO_WritePin(MAG_SWITCH_GPIO_Port, MAG_SWITCH_Pin, GPIO_PIN_RESET);
}

#endif /* ELECTROMAGNET_DRIVER_ELECTROMAGNET_H_ */
