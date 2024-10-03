/*
 * app_bat_monitor.h
 *
 *  Created on: Sep 11, 2024
 *      Author: 14769
 */

#ifndef APP_BAT_MONITOR_H_
#define APP_BAT_MONITOR_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

void app_batMonitor(void *argument);

#endif /* APP_BAT_MONITOR_H_ */
