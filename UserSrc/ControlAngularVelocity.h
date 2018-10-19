/*
 * ControlAngularVelocity.h
 *
 *  Created on: 2018/10/20
 *      Author: shuuichi
 */

#ifndef CONTROLANGULARVELOCITY_H_
#define CONTROLANGULARVELOCITY_H_

#include "typedef.h"

void CAV_Init(void);

void CAV_SetTargetUpAccel(float32_t upAccel);
void CAV_SetTargetDownAccel(float32_t downAccel);
void CAV_SetTarget(float32_t target);

void CAV_ControlTask(void);


#endif /* CONTROLANGULARVELOCITY_H_ */
