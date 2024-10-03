/*
 * app_imuFX.c
 *
 *  Created on: Sep 11, 2024
 *      Author: 14769
 */

#include "apps.h"
#include "app_imuFX.h"

#include "driver_icm20948.h"
#include "Invn/Devices/Drivers/ICM20948/Icm20948.h"
#include "Invn/Devices/Drivers/ICM20948/Icm20948MPUFifoControl.h"
#include "Invn/Devices/Drivers/Ak0991x/Ak0991x.h"

float imuRotations[4];
float imuAccels[3];
float imuGyros[3];

static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
	INV_SENSOR_TYPE_ACCELEROMETER,
	INV_SENSOR_TYPE_GYROSCOPE,
	INV_SENSOR_TYPE_RAW_ACCELEROMETER,
	INV_SENSOR_TYPE_RAW_GYROSCOPE,
	INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
	INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
	INV_SENSOR_TYPE_BAC,
	INV_SENSOR_TYPE_STEP_DETECTOR,
	INV_SENSOR_TYPE_STEP_COUNTER,
	INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
	INV_SENSOR_TYPE_ROTATION_VECTOR,
	INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
	INV_SENSOR_TYPE_MAGNETOMETER,
	INV_SENSOR_TYPE_SMD,
	INV_SENSOR_TYPE_PICK_UP_GESTURE,
	INV_SENSOR_TYPE_TILT_DETECTOR,
	INV_SENSOR_TYPE_GRAVITY,
	INV_SENSOR_TYPE_LINEAR_ACCELERATION,
	INV_SENSOR_TYPE_ORIENTATION,
	INV_SENSOR_TYPE_B2S
};

void build_sensor_event_data(void * context, uint8_t sensortype, uint64_t timestamp, const void * data, const void *arg)
{
    (void) context;
    (void) timestamp;
    (void) arg;
    uint8_t sensor_id = convert_to_generic_ids[sensortype];

    switch(sensor_id)
    {
        case INV_SENSOR_TYPE_ACCELEROMETER:
        {
            osMutexAcquire(mtx_imuRotationsHandle, osWaitForever);
            memcpy(imuAccels, data, sizeof(imuAccels));
            osMutexRelease(mtx_imuRotationsHandle);
            break;
        }
        case INV_SENSOR_TYPE_GYROSCOPE:
        {
            osMutexAcquire(mtx_imuRotationsHandle, osWaitForever);
            memcpy(imuGyros, data, sizeof(imuGyros));
            osMutexRelease(mtx_imuRotationsHandle);
            break;
        }
        case INV_SENSOR_TYPE_ROTATION_VECTOR:
        {
            osMutexAcquire(mtx_imuRotationsHandle, osWaitForever);
            memcpy(imuRotations, data, sizeof(imuRotations));
            osMutexRelease(mtx_imuRotationsHandle);
            break;
        }
    }
}

void app_imuFX(void *argument)
{
    (void) argument;

    for(;;)
    {
        inv_icm20948_poll_sensor(&icm_device, (void *)0, build_sensor_event_data);
        osDelay(10);
    }
}
