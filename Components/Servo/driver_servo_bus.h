/*
 * driver_servo_bus.h
 *
 *  Created on: Sep 15, 2024
 *      Author: 14769
 */

#ifndef SERVO_DRIVER_SERVO_BUS_H_
#define SERVO_DRIVER_SERVO_BUS_H_

#include "usart.h"
#include "main.h"

#include "stdint.h"
#include "stdarg.h"
#include "string.h"

typedef struct busservo_dev_handle
{
	uint8_t id;
	uint8_t status;
	uint16_t target_position;
	uint16_t target_last;
	uint16_t current_position;
}BusServo_dev_handle;

typedef struct busseervo_bus_handle
{
	UART_HandleTypeDef* port;
	BusServo_dev_handle* pdev;
	uint8_t num;
	uint8_t txbuffer[64];
	uint8_t rxbuffer[64];
}BusServo_bus_handle;

typedef enum
{
	BUS_CMD_PING = 0X01,
	BUS_CMD_RD	= 0X02,
	BUS_CMD_WD = 0X03,
	BUS_CMD_WR	= 0X04,
	BUS_CMD_ACT	= 0X05,
	BUS_CMD_REST	= 0X06,
	BUS_CMD_SYNC	= 0X83,
}ServoCmd;

typedef struct
{
	uint8_t OVERVOLTAGE	:	1;
	uint8_t _reserved0	:	1;
	uint8_t OVERHEAT	:	1;
	uint8_t _reserved1	:	2;
	uint8_t OVERLOAD	:	1;
	uint8_t _reserved2	:	2;
}ServoStatus;

#define TX_HEADER	(0xFFFF)	///发送帧字头
#define RX_HEADER	(0xFFF5)	///接收帧字头

void busservo_bus_init(BusServo_bus_handle* p);

void busservo_setAsync(BusServo_bus_handle* p, uint8_t id, uint16_t degree, uint16_t time_ms);

void busservo_action(BusServo_bus_handle* p);

void busservo_setSync(BusServo_bus_handle* p, ...);

void busservo_getPosition(BusServo_bus_handle* p, uint8_t id);

#endif /* SERVO_DRIVER_SERVO_BUS_H_ */
