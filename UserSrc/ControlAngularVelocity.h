/*
 * ControlAngularVelocity.h
 *
 *  Created on: 2018/10/20
 *      Author: shuuichi
 */

#ifndef CONTROLANGULARVELOCITY_H_
#define CONTROLANGULARVELOCITY_H_

#include "typedef.h"
#include "Log.h"

void CAV_Init(void);

void CAV_StartDriveMotor(void);
void CAV_StopDriveMotor(void);

void CAV_SetTargetUpAccel(float32_t upAccel);
void CAV_SetTargetDownAccel(float32_t downAccel);
void CAV_SetTarget(float32_t target);

void CAV_ControlTask(void);

float32_t CAV_GetTarget(void);
float32_t CAV_GetVelocity(void);
float32_t CAV_GetErrorNow(void);
float32_t CAV_GetAngle(void);
float32_t CAV_GetRadius(void);
float32_t CAV_GetVirtualThetaDeg(void);

void CAV_ClearAngle(void);


LOG_StrControlVelocityHeader *CAV_GetLogHeader(void);
LOG_StrControlVelocityArray  *CAV_GetLogArray(void);

#endif /* CONTROLANGULARVELOCITY_H_ */
