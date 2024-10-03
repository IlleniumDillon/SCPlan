/*
 * protocol.h
 *
 *  Created on: Sep 29, 2024
 *      Author: 14769
 */

#ifndef PROTOCOL_PROTOCOL_H_
#define PROTOCOL_PROTOCOL_H_

#include "fpn.h"
#include "vofa+.h"

typedef struct __attribute__((aligned(1))) comdatain
{
	drift16 linearVel;
	drift16 angularVel;
	uint16_t armAngles[2];
	drift16 tripodAngle;
    uint8_t magEnable;
}ComDataIn;

typedef struct
{
	float linearVel;
	float angularVel;
	uint16_t armAngles[2];
	float tripodAngle;
	uint8_t magEnable;
}DataIn;

typedef struct __attribute__((aligned(1))) comdataout
{
	drift8 voltage;
	drift32 imuQuat[3];
	drift16 wheelSpeeds[2];
	uint16_t armAngles[2];
}ComDataOut;


typedef struct
{
	uint16_t armAngles[2];
}ArmTarget;

#endif /* PROTOCOL_PROTOCOL_H_ */
