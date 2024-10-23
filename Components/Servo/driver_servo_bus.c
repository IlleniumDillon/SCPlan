/*
 * driver_servo_bus.c
 *
 *  Created on: Sep 15, 2024
 *      Author: 14769
 */

#include "driver_servo_bus.h"

#include "cmsis_os.h"

#define HEADER1	(0)
#define HEADER2	(1)
#define IDNUM	(2)
#define PACKLEN	(3)
#define CMD_STATUS	(4)
#define PAYLOAD	(5)
#define CHECKBYTE(EXP)	((EXP) + 3)

uint8_t arg[64];

void __getCheckByte(uint8_t* busTxBuffer)
{
	uint8_t checkByte = busTxBuffer[IDNUM] +
						busTxBuffer[PACKLEN] +
						busTxBuffer[CMD_STATUS];
	for(uint8_t i = 0; i < busTxBuffer[PACKLEN] - 2; i++)
	{
		checkByte += busTxBuffer[PAYLOAD + i];
	}
	checkByte = ~checkByte;
	checkByte &= 0xFF;
	busTxBuffer[CHECKBYTE(busTxBuffer[PACKLEN])] = checkByte;
}

uint8_t __checkCheckByte(uint8_t* busRxBuffer)
{
	uint8_t checkByte = busRxBuffer[IDNUM] +
			busRxBuffer[PACKLEN] +
			busRxBuffer[CMD_STATUS];
	for(uint8_t i = 0; i < busRxBuffer[PACKLEN] - 2; i++)
	{
		checkByte += busRxBuffer[PAYLOAD + i];
	}
	checkByte = ~checkByte;
	checkByte &= 0xFF;
	uint8_t rst =  busRxBuffer[CHECKBYTE(busRxBuffer[PACKLEN])] == checkByte ? 1 : 0;
	return rst;
}

void __dataWrite(BusServo_bus_handle* p, uint8_t id, uint8_t dataLen, ServoCmd cmd, void* arg)
{
	uint8_t txSize = dataLen + 4;
	uint8_t argSize = dataLen - 2;

	p->txbuffer[HEADER1] = 0XFF;
	p->txbuffer[HEADER2] = 0XFF;
	p->txbuffer[IDNUM] = id;
	p->txbuffer[PACKLEN] = dataLen;
	p->txbuffer[CMD_STATUS] = cmd;

	memcpy(&p->txbuffer[PAYLOAD], arg, argSize);

	__getCheckByte(p->txbuffer);

	while (HAL_UART_GetState(p->port) != HAL_UART_STATE_READY)
	{
		osDelay(1);
	}
	HAL_UART_Transmit(p->port, p->txbuffer, txSize, 0xFF);
}

void __dataRead(BusServo_bus_handle* p)
{
	while (HAL_UART_GetState(p->port) != HAL_UART_STATE_READY)
	{
		osDelay(1);
	}
	HAL_UART_Receive(p->port, p->rxbuffer, 64, 0xFF);

	if(p->rxbuffer[HEADER1] != 0xFF || p->rxbuffer[HEADER2] != 0xF5) return;
	if (!__checkCheckByte(p->rxbuffer)) return;

	int16_t indx = -1;
	for(uint8_t i ;i < p->num; i++)
	{
		if(p->pdev[i].id == p->rxbuffer[IDNUM])
		{
			indx = i;
			break;
		}
	}
	if (indx < 0) return;

	p->pdev[indx].status = p->rxbuffer[CMD_STATUS];
	if(p->rxbuffer[PACKLEN] == 4)
	{
		p->pdev[indx].current_position = (p->rxbuffer[PAYLOAD] << 8) + p->rxbuffer[PAYLOAD + 1];
	}
}

void busservo_bus_init(BusServo_bus_handle* p)
{
	for(uint8_t i = 0; i < p->num; i++)
	{
		__dataWrite(p, p->pdev[i].id, 2, BUS_CMD_PING, NULL);
		__dataRead(p);
	}
}

void busservo_setAsync(BusServo_bus_handle* p, uint8_t id, uint16_t degree, uint16_t time_ms)
{
	arg[0] = 0x2a;
	arg[1] = degree>>8;
	arg[2] = degree&0Xff;
	arg[3] = time_ms>>8;
	arg[4] = time_ms&0xff;
	__dataWrite(p, id, 7, BUS_CMD_WR, arg);
}

void busservo_action(BusServo_bus_handle* p)
{
	__dataWrite(p, 0xfe, 2, BUS_CMD_ACT, NULL);
}

void busservo_setSync(BusServo_bus_handle* p, ...)
{
	uint8_t argSize = 2;
	arg[0] = 0x2a;
	arg[1] = 0x04;
	va_list ap;
	va_start(ap,p);
	for(uint8_t i = 0; i < p->num; i++)
	{
		int id = va_arg(ap, int);
		int degree = va_arg(ap, int);
		int time_ms = va_arg(ap, int);
		arg[argSize] = (uint8_t) id;				argSize++;
		arg[argSize] = (uint8_t) (degree>>8);		argSize++;
		arg[argSize] = (uint8_t) (degree & 0xff);	argSize++;
		arg[argSize] = (uint8_t) (time_ms >> 8);	argSize++;
		arg[argSize] = (uint8_t) (time_ms & 0xff);	argSize++;
	}
	va_end(ap);

	__dataWrite(p, 0xfe, 4 + 5 * p->num, BUS_CMD_SYNC, arg);
}

void busservo_getPosition(BusServo_bus_handle* p, uint8_t id)
{
	arg[0] = 0x38;
	arg[1] = 0x02;
	__dataWrite(p, id, 4, BUS_CMD_RD, arg);
	__dataRead(p);
}
