/*
 * app_arm.h
 *
 *  Created on: Oct 3, 2024
 *      Author: 14769
 */

#ifndef APP_ARM_H_
#define APP_ARM_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

void app_arm(void *argument);

#endif /* APP_ARM_H_ */
