/*
 * board_button.h
 *
 *  Created on: Sep 17, 2024
 *      Author: 14769
 */

#ifndef BUTTON_DRIVER_BUTTON_PORT_H_
#define BUTTON_DRIVER_BUTTON_PORT_H_

#include "flexible_button.h"

#define ENUM_TO_STR(e) (#e)

typedef enum
{
    USER_BUTTON_UP = 0,
    USER_BUTTON_DOWN,
    USER_BUTTON_LEFT,
    USER_BUTTON_RIGHT,
    USER_BUTTON_OK,
    USER_BUTTON_MAX
} user_button_t;

extern flex_button_t user_button[USER_BUTTON_MAX];

uint8_t btn_read(void *arg);

#endif /* BUTTON_DRIVER_BUTTON_PORT_H_ */
