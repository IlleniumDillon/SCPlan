/*
 * driver_button.h
 *
 *  Created on: Sep 17, 2024
 *      Author: 14769
 */

#ifndef BUTTON_DRIVER_BUTTON_H_
#define BUTTON_DRIVER_BUTTON_H_

#include "gpio.h"
#include "main.h"

#include "flexible_button.h"

#include "driver_button_port.h"

void button_init(flex_button_response_callback cb);

void button_scan();

#endif /* BUTTON_DRIVER_BUTTON_H_ */
