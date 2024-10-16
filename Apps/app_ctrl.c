/*
 * app_ctrl.c
 *
 *  Created on: Sep 11, 2024
 *      Author: 14769
 */

#include "apps.h"
#include "app_ctrl.h"

float wheelSpeeds[2] = {0, 0};
float currentOutput[2] = {0,0};
float currentTarget[2] = {0, 0};
float wheelbase_m = 0.18393;
float fgamma = 1;

void app_ctrl(void *argument)
{
    (void) argument;
    pid_init(&leftWheel.ctrl, 4, 10, 0, 0.02);
    pid_init(&rightWheel.ctrl, 4, 10, 0, 0.02);
    pid_setLimit(&leftWheel.ctrl, 1.0, -1.0, 0.7, -0.7);
    pid_setLimit(&rightWheel.ctrl, 1.0, -1.0, 0.7, -0.7);
    osDelay(1000);
    int count = 0;
    SpeedTarget speedTarget = {0};
    EmagTarget emagTarget = {0};
    TripodTarget tripodTarget = {0};
    for(;;)
    {
        if(osMessageQueueGet(queue_speedTargetHandle, &speedTarget, NULL, 0) == osOK)
        {
        	// get speed target
        	float dummy_wheelbase = wheelbase_m * fgamma;
        	em_setSpeedTarget(&leftWheel, speedTarget.linearVel - speedTarget.angularVel * dummy_wheelbase / 2);
        	em_setSpeedTarget(&rightWheel, speedTarget.linearVel + speedTarget.angularVel * dummy_wheelbase / 2);
        }
        if(osMessageQueueGet(queue_emagTargetHandle, &emagTarget, NULL, 0) == osOK)
        {
        	if (emagTarget)
			{
				emag_enable();
			}
			else
			{
				emag_disable();
			}
        }
        if(osMessageQueueGet(queue_tripodTargetHandle, &tripodTarget, NULL, 0) == osOK)
        {
        	pwmservo_set_degree(&tripod_handle, tripodTarget);
        }


        // motor control code here
        em_update(&leftWheel);
        em_update(&rightWheel);
//        pid_update(&leftWheel.ctrl, em_getSpeed(&leftWheel));
//        pid_update(&rightWheel.ctrl, em_getSpeed(&rightWheel));

//        if (osMutexAcquire(mtx_wheelSpeedsHandle, 0) == osOK)
//        {
//        	wheelSpeeds[0] = leftWheel.ctrl.feedback;
//        	wheelSpeeds[1] = rightWheel.ctrl.feedback;
//        	currentOutput[0] = leftWheel.ctrl.out;
//        	currentOutput[1] = rightWheel.ctrl.out;
//        	currentTarget[0] = leftWheel.ctrl.target;
//        	currentTarget[1] = rightWheel.ctrl.target;
//        	osMutexRelease(mtx_wheelSpeedsHandle);
//        }

        osDelay(20);
    }
}
