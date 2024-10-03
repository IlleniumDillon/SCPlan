/*
 * app_arm.c
 *
 *  Created on: Oct 3, 2024
 *      Author: 14769
 */

#include "apps.h"
#include "app_arm.h"

uint16_t armAngles[2] = {0, 0};

void app_arm(void *argument)
{
	(void) argument;
	ArmTarget armTarget = {0};
	for(;;)
	{
		if (osMessageQueueGet(queue_armTargetHandle, &armTarget, NULL, 0) == osOK)
		{
			busservo_setSync(&arm_handle,
					arm_handle.pdev[0].id, armTarget.armAngles[0], 50,
					arm_handle.pdev[0].id, armTarget.armAngles[1], 50);
		}
		else
		{
			for (int i = 0; i < arm_handle.num; i++)
			{
				busservo_getPosition(&arm_handle, arm_handle.pdev[i].id);
				if (osMutexAcquire(mtx_armAnglesHandle, 0) == osOK)
				{
					armAngles[i] = arm_handle.pdev[i].current_position;
				}
			}
		}
		osDelay(50);
	}
}
