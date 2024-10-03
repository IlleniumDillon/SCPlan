/*
 * apps.h
 *
 *  Created on: Sep 11, 2024
 *      Author: 14769
 */

#ifndef APPS_H_
#define APPS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "app_bat_monitor.h"
#include "app_communicate.h"
#include "app_ctrl.h"
#include "app_imuFX.h"
#include "app_ui.h"
#include "app_arm.h"

#include "driver_buzzer.h"
#include "driver_electromagnet.h"
#include "driver_icm20948.h"
#include "driver_motor.h"
#include "driver_servo_bus.h"
#include "driver_servo_pwm.h"
#include "driver_ssd1306_advance.h"
#include "driver_button.h"
#include "driver_usb.h"

#include "board_motor.h"
#include "board_servo.h"

#include "protocol.h"

#include "adc.h"

//typedef struct __attribute__((aligned(1))) comdatain
//{
//    float linearVel;
//    float angularVel;
//    float armAngles[2];
//    float tripodAngle;
//    uint8_t magEnable;
//}ComDataIn;
//
//typedef struct __attribute__((aligned(1))) comdataout
//{
//    float voltage;
//    float imuQuat[4];
//    float wheelSpeeds[2];
//    float armAngles[2];
//}ComDataOut;

extern float batVoltage;
extern float imuRotations[4];
extern float imuAccels[3];
extern float imuGyros[3];
extern float wheelSpeeds[2];
extern uint16_t armAngles[2];

extern osMutexId_t mtx_batVoltageHandle;
extern osMutexId_t mtx_imuRotationsHandle;
extern osMutexId_t mtx_wheelSpeedsHandle;
extern osMutexId_t mtx_armAnglesHandle;

extern osMessageQueueId_t queue_comInHandle;
extern osMessageQueueId_t queue_armTargetHandle;

extern osThreadId_t thread_batMonitorHandle;
extern osThreadId_t thread_communicateHandle;
extern osThreadId_t thread_ctrlHandle;
extern osThreadId_t thread_imuFXHandle;
extern osThreadId_t thread_uiHandle;
extern osThreadId_t thread_armHandle;

void board_init(void);

void apps_init(void);

void apps_start(void);

#endif /* APPS_H_ */
