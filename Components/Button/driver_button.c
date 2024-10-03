/*
 * driver_button.c
 *
 *  Created on: Sep 17, 2024
 *      Author: 14769
 */

#include "driver_button.h"
#include "driver_button_port.h"

#include <stdlib.h>
#include <string.h>

void button_init(flex_button_response_callback cb)
{
	memset(&user_button[0], 0x0, sizeof(user_button));

	for (uint8_t i = 0; i < USER_BUTTON_MAX; i ++)
	{
		user_button[i].id = i;
		user_button[i].usr_button_read = btn_read;
		user_button[i].cb = cb;
		user_button[i].pressed_logic_level = 0;
		user_button[i].short_press_start_tick = FLEX_MS_TO_SCAN_CNT(1500);
		user_button[i].long_press_start_tick = FLEX_MS_TO_SCAN_CNT(3000);
		user_button[i].long_hold_start_tick = FLEX_MS_TO_SCAN_CNT(4500);

		flex_button_register(&user_button[i]);
	}
}

void button_scan()
{
	flex_button_scan();
}
