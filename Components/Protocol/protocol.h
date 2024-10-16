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

typedef struct __attribute__((aligned(1))) comdatain_Reg
{
	drift16 linearVel;
	drift16 angularVel;
}ComDataIn_Reg;

typedef enum
{
	COM_FLAG_ARM,
	COM_FLAG_EMAG,
	COM_FLAG_TRIPOD
}ComDataIn_Bur_Flag;

typedef struct __attribute__((aligned(1))) comdatain_Bur
{
	ComDataIn_Bur_Flag flag;
	union
	{
		uint16_t armAngles[2];
		uint8_t emag;
		uint16_t tripodAngle;
	};
}ComDataIn_Bur;

typedef struct
{
	float linearVel;
	float angularVel;
}SpeedTarget;

typedef struct
{
	uint16_t armAngles[2];
}ArmTarget;

typedef uint8_t EmagTarget;

typedef uint16_t TripodTarget;


typedef struct __attribute__((aligned(1))) comdataout_reg
{
	drift8 voltage;
	drift32 imuQuat[4];
	drift16 wheelSpeeds[2];
	uint16_t armAngles[2];
}ComDataOut_Reg;




#endif /* PROTOCOL_PROTOCOL_H_ */
