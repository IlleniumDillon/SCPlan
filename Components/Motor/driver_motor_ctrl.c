/*
 * driver_motor_ctrl.c
 *
 *  Created on: Oct 3, 2024
 *      Author: 14769
 */

#include "driver_motor_ctrl.h"

#include <math.h>

void pid_init(PID* pctrl, float p, float i, float d, float dt)
{
	pctrl->target = 0;
	pctrl->feedback = 0;
	pctrl->out = 0;
	pctrl->p = p;
	pctrl->i = i;
	pctrl->d = d;
	pctrl->dt = dt;
	pctrl->error = 0;
	pctrl->last_error = 0;
	pctrl->sum_error = 0;
	pctrl->outlim_h = INFINITY;
	pctrl->outlim_l = -INFINITY;
	pctrl->ilim_h = INFINITY;
	pctrl->ilim_l = -INFINITY;
}

void pid_setLimit(PID* pctrl, float outlim_h, float outlim_l, float ilim_h, float ilim_l)
{
	pctrl->outlim_h = outlim_h;
	pctrl->outlim_l = outlim_l;
	pctrl->ilim_h = ilim_h;
	pctrl->ilim_l = ilim_l;
}

float pid_update(PID* pctrl, float feedback)
{
	pctrl->feedback = feedback;

	pctrl->error = pctrl->target - pctrl->feedback;

	pctrl->sum_error += pctrl->error * pctrl->dt;
	pctrl->sum_error = pctrl->sum_error > pctrl->ilim_h ? pctrl->ilim_h : pctrl->sum_error;
	pctrl->sum_error = pctrl->sum_error < pctrl->ilim_l ? pctrl->ilim_l : pctrl->sum_error;

	pctrl->out = pctrl->error * pctrl->p +
			pctrl->sum_error * pctrl->i +
			(pctrl->error - pctrl->last_error) / pctrl->dt * pctrl->d;
	pctrl->out = pctrl->out > pctrl->outlim_h ? pctrl->outlim_h : pctrl->out;
	pctrl->out = pctrl->out < pctrl->outlim_l ? pctrl->outlim_l : pctrl->out;

	pctrl->last_error = pctrl->error;

	return pctrl->out;
}
