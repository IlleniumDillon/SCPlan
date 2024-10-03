/*
 * app_ctrl.c
 *
 *  Created on: Sep 11, 2024
 *      Author: 14769
 */

#include "apps.h"
#include "app_ctrl.h"

float wheelSpeeds[2] = {0, 0};

void app_ctrl(void *argument)
{
    (void) argument;
    DataIn dataIn = {0};
    for(;;)
    {
        if(osMessageQueueGet(queue_comInHandle, &dataIn, NULL, 0) == osOK)
        {
        	pwmservo_set_degree(&tripod_handle, dataIn.tripodAngle);
        	if (dataIn.magEnable)
        	{
        		emag_enable();
        	}
        	else
        	{
        		emag_disable();
        	}
        	// get speed target

        }

        // motor control code here
        em_update(&leftWheel);
        em_update(&rightWheel);

        osDelay(20);
    }
}
