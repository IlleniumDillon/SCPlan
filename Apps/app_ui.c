/*
 * app_ui.c
 *
 *  Created on: Sep 11, 2024
 *      Author: 14769
 */

#include "apps.h"
#include "app_ui.h"

#include "driver_ssd1306_advance.h"

#include <math.h>

void toEulerAngles(float* q, float* e)
{
    float sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    float cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    e[0] = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
    if (fabs(sinp) >= 1)
    {
        e[1] = copysign(M_PI / 2, sinp);
    }
    else
    {
        e[1] = asin(sinp);
    }

    float siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    e[2] = atan2(siny_cosp, cosy_cosp);
}

void app_ui(void *argument)
{
    (void) argument;
    char line[20];
	float voltage = 0;
	float imuq[4];
    float imue[3];
	int rst = 0;

	ssd1306_advance_clear();

    for(;;)
    {
        if (osMutexAcquire(mtx_batVoltageHandle, 0) == osOK)
        {
            voltage = batVoltage;
            osMutexRelease(mtx_batVoltageHandle);
        }
        if (osMutexAcquire(mtx_imuRotationsHandle, 0) == osOK)
        {
            memcpy(imuq, imuRotations, sizeof(imuq));
            osMutexRelease(mtx_imuRotationsHandle);
            toEulerAngles(imuq, imue);
        }
        rst = sprintf(line,"voltage: % .3fv  ",voltage);
        ssd1306_advance_string(0, 0, line, rst, 1, SSD1306_FONT_12);
        rst = sprintf(line,"x: % .5f   ",imue[0]);
        ssd1306_advance_string(0, 1*16, line, rst, 1, SSD1306_FONT_12);
        rst = sprintf(line,"y: % .5f   ",imue[1]);
        ssd1306_advance_string(0, 1*16+12, line, rst, 1, SSD1306_FONT_12);
        rst = sprintf(line,"z: % .5f   ",imue[2]);
        ssd1306_advance_string(0, 1*16+2*12, line, rst, 1, SSD1306_FONT_12);

        osDelay(20);
    }
}
