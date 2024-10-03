/*
 * app_bat_monitor.c
 *
 *  Created on: Sep 11, 2024
 *      Author: 14769
 */

#include "apps.h"
#include "app_bat_monitor.h"

#include "adc.h"

float batVoltage;

void btn_evt_cb(void *arg)
{
	flex_button_t *btn = (flex_button_t *)arg;
	if (flex_button_event_read(&user_button[USER_BUTTON_UP]) == FLEX_BTN_PRESS_DOWN)
	{
		emag_enable();
		buzzer_play(tone_mag_on);
	}
	if (flex_button_event_read(&user_button[USER_BUTTON_UP]) == FLEX_BTN_PRESS_CLICK)
	{
		emag_disable();
		buzzer_play(tone_mag_off);
	}
	if (flex_button_event_read(&user_button[USER_BUTTON_DOWN]) == FLEX_BTN_PRESS_DOWN)
	{
		buzzer_play(tone_switch);
	}
	if (flex_button_event_read(&user_button[USER_BUTTON_LEFT]) == FLEX_BTN_PRESS_DOWN)
	{
		buzzer_play(tone_switch);
	}
	if (flex_button_event_read(&user_button[USER_BUTTON_RIGHT]) == FLEX_BTN_PRESS_DOWN)
	{
		buzzer_play(tone_switch);
	}
	if (flex_button_event_read(&user_button[USER_BUTTON_OK]) == FLEX_BTN_PRESS_DOWN)
	{
		buzzer_play(tone_switch);
	}
}

void app_batMonitor(void *argument)
{
    (void) argument;

    button_init(btn_evt_cb);

    uint32_t uVoltage = 0;
    float fVoltage = 0.0;
    // HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &uVoltage, 1);
    uint8_t adcCount = 0;
    uint8_t buzCount = 0;

    for(;;)
    {
    	if (adcCount > 10)
    	{
    		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
			{
				uVoltage = HAL_ADC_GetValue(&hadc1);
			}

			fVoltage = (float) uVoltage * 0.00020294189453125;
			if (osMutexAcquire(mtx_batVoltageHandle, 0) == osOK)
			{
				batVoltage = fVoltage;
				osMutexRelease(mtx_batVoltageHandle);
			}
			adcCount = 0;
    	}


        button_scan();

        if (buzCount > 2)
        {
        	buzzer_update();
        	buzCount = 0;
        }


        osDelay(20);
        adcCount++;
        buzCount++;

    }
}
