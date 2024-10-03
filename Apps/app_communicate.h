/*
 * app_communicate.h
 *
 *  Created on: Sep 11, 2024
 *      Author: 14769
 */

#ifndef APP_COMMUNICATE_H_
#define APP_COMMUNICATE_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

void app_communicate(void *argument);

#endif /* APP_COMMUNICATE_H_ */
