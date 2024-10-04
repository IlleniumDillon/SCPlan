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
	if (size == sizeof(ComDataIn))
	{
		ComDataIn* idata = (ComDataIn*) pdata;
		DataIn data;
		data.angularVel = drift16_to_float(idata->angularVel);
		data.armAngles[0] = idata->armAngles[0];
		data.armAngles[1] = idata->armAngles[1];
		data.linearVel = drift16_to_float(idata->linearVel);
		data.magEnable = idata->magEnable;
		data.tripodAngle = drift16_to_float(idata->tripodAngle);

		osMessageQueuePut(queue_comInHandle, &data, 0, 0);
		osMessageQueuePut(queue_armTargetHandle, &data.armAngles, 0, 0);
	}
}
void usb_rx_ch2_cb(uint8_t* pdata, size_t size)
{
	static DataIn que = {0};
	char head[16] = {0};
	float data = 0;
	int ret = firewater_read(pdata, size, head, 1, &data);
	if (strcmp(head, "sdeg") == 0)
	{
		que.tripodAngle = data;
		osMessageQueuePut(queue_comInHandle, &que, 0, 0);
	}
	else if (strcmp(head, "emag") == 0)
	{
		que.magEnable = data > 0.5 ? 1 : 0;
		osMessageQueuePut(queue_comInHandle, &que, 0, 0);
	}
	else if (strcmp(head, "lpwm") == 0)
	{
		que.angularVel = data;
		osMessageQueuePut(queue_comInHandle, &que, 0, 0);
	}
	else if (strcmp(head, "rpwm") == 0)
	{
		que.linearVel = data;
		osMessageQueuePut(queue_comInHandle, &que, 0, 0);
	}
	else if (strcmp(head, "arm1") == 0)
	{
		que.armAngles[0] = (uint16_t)data;
		osMessageQueuePut(queue_armTargetHandle, &que.armAngles, 0, 0);
	}
	else if (strcmp(head, "arm2") == 0)
	{
		que.armAngles[1] = (uint16_t)data;
		osMessageQueuePut(queue_armTargetHandle, &que.armAngles, 0, 0);
	}
}

void app_communicate(void *argument)
{
    (void) argument;
    uint8_t comCount = 0;
    driver_usb_init();
    /// TODO: cb
    ComDataOut comDataOut;
    uint8_t vofaTx[64] = {0};
    for(;;)
    {
//        if (osMutexAcquire(mtx_batVoltageHandle, 0) == osOK)
//        {
//            comDataOut.voltage = batVoltage;
//            osMutexRelease(mtx_batVoltageHandle);
//        }
//        if (osMutexAcquire(mtx_imuRotationsHandle, 0) == osOK)
//        {
//            memcpy(comDataOut.imuQuat, imuRotations, sizeof(comDataOut.imuQuat));
//            osMutexRelease(mtx_imuRotationsHandle);
//        }
//        if (osMutexAcquire(mtx_wheelSpeedsHandle, 0) == osOK)
//        {
//            memcpy(comDataOut.wheelSpeeds, wheelSpeeds, sizeof(comDataOut.wheelSpeeds));
//            osMutexRelease(mtx_wheelSpeedsHandle);
//        }
//        if (osMutexAcquire(mtx_armAnglesHandle, 0) == osOK)
//        {
//            memcpy(comDataOut.armAngles, armAngles, sizeof(comDataOut.armAngles));
//            osMutexRelease(mtx_armAnglesHandle);
//        }
        // CDC_Transmit_HS((uint8_t *) &comDataOut, sizeof(comDataOut));
        driver_usb_task();
        drvier_usb_rx_cbs[USB_COMDATA_ITF] = usb_rx_ch1_cb;
        drvier_usb_rx_cbs[USB_COMCMD_ITF] = usb_rx_ch2_cb;
        if (comCount > 200)
        {
        	float data[] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0};
        	int size = firewater_write(vofaTx, 64, "test", 10, data);
        	// driver_usb_write(USB_COMDATA_ITF, (uint8_t*)&comDataOut, sizeof(comDataOut));
        	driver_usb_write(USB_COMDATA_ITF, vofaTx, size);
        	comCount = 0;
        }

        // osDelay(1);
        comCount++;
    }
}
