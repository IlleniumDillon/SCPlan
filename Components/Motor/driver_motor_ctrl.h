/*
 * driver_motor_ctrl.h
 *
 *  Created on: Oct 3, 2024
 *      Author: 14769
 */

#ifndef MOTOR_DRIVER_MOTOR_CTRL_H_
#define MOTOR_DRIVER_MOTOR_CTRL_H_

typedef struct
{
	float target;
	float feedback;
	float out;

	float p,i,d,dt;
	float error,last_error,sum_error;
	float outlim_h, outlim_l;
	float ilim_h,ilim_l;
}PID;

void pid_init(PID* pctrl, float p, float i, float d, float dt);

void pid_setLimit(PID* pctrl, float outlim_h, float outlim_l, float ilim_h, float ilim_l);

static inline void pid_setTarget(PID* pctrl, float target)
{
	pctrl->target = target;
}

float pid_update(PID* pctrl, float feedback);

#endif /* MOTOR_DRIVER_MOTOR_CTRL_H_ */
