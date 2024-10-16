/*
 * app_communicate.c
 *
 *  Created on: Sep 11, 2024
 *      Author: 14769
 */

#include "apps.h"
#include "app_communicate.h"

//#include "usbd_cdc_if.h"
#include "driver_usb.h"

void usb_rx_ch1_cb(uint8_t* pdata, size_t size)
{
	if (size == sizeof(ComDataIn_Reg))
	{
		ComDataIn_Reg* p = (ComDataIn_Reg*) pdata;
		SpeedTarget data;
		data.linearVel = drift16_to_float(p->linearVel);
		data.angularVel = drift16_to_float(p->angularVel);
		osMessageQueuePut(queue_speedTargetHandle, &data, 0, 0);
	}
}
void usb_rx_ch2_cb(uint8_t* pdata, size_t size)
{
	if (size == sizeof(ComDataIn_Bur))
	{
		ComDataIn_Bur* p = (ComDataIn_Bur*)pdata;
		switch(p->flag)
		{
		case COM_FLAG_ARM:
		{
			ArmTarget data;
			memcpy(&data, p->armAngles[0], sizeof(ArmTarget));
			osMessageQueuePut(queue_armTargetHandle, &data, 0, 0);
			break;
		}
		case COM_FLAG_EMAG:
		{
			EmagTarget data;
			memcpy(&data, &p->emag, sizeof(EmagTarget));
			osMessageQueuePut(queue_emagTargetHandle, &data, 0, 0);
			break;
		}
		case COM_FLAG_TRIPOD:
		{
			TripodTarget data;
			memcpy(&data, &p->tripodAngle, sizeof(TripodTarget));
			osMessageQueuePut(queue_tripodTargetHandle, &data, 0, 0);
			break;
		}
		}
	}
}

void app_communicate(void *argument)
{
    (void) argument;
    uint8_t comCount = 0;
    driver_usb_init();
    /// cb
    drvier_usb_rx_cbs[USB_COMDATA_ITF] = usb_rx_ch1_cb;
	drvier_usb_rx_cbs[USB_COMCMD_ITF] = usb_rx_ch2_cb;
    ComDataOut_Reg comDataOut;
    for(;;)
    {
        if (osMutexAcquire(mtx_batVoltageHandle, 0) == osOK)
        {
            comDataOut.voltage = drift8_from_float(batVoltage);
            osMutexRelease(mtx_batVoltageHandle);
        }
        if (osMutexAcquire(mtx_imuRotationsHandle, 0) == osOK)
        {
        	comDataOut.imuQuat[0].memory = imuRotations[0];
        	comDataOut.imuQuat[1].memory = imuRotations[1];
        	comDataOut.imuQuat[2].memory = imuRotations[2];
        	comDataOut.imuQuat[3].memory = imuRotations[3];
            osMutexRelease(mtx_imuRotationsHandle);
        }
        if (osMutexAcquire(mtx_wheelSpeedsHandle, 0) == osOK)
        {
        	comDataOut.wheelSpeeds[0] = drift16_from_float(wheelSpeeds[0]);
        	comDataOut.wheelSpeeds[1] = drift16_from_float(wheelSpeeds[1]);
            osMutexRelease(mtx_wheelSpeedsHandle);
        }
        if (osMutexAcquire(mtx_armAnglesHandle, 0) == osOK)
        {
        	comDataOut.armAngles[0] = armAngles[0];
        	comDataOut.armAngles[1] = armAngles[1];
            osMutexRelease(mtx_armAnglesHandle);
        }
        // CDC_Transmit_HS((uint8_t *) &comDataOut, sizeof(comDataOut));
        driver_usb_task();

        if (comCount > 50)
        {
        	driver_usb_write(USB_COMDATA_ITF, (uint8_t*)&comDataOut, sizeof(comDataOut));
        	comCount = 0;
        }

        // osDelay(1);
        comCount++;
    }
}
