/*
 * ControlVelocity.h
 *
 *  Created on: 2018/10/18
 *      Author: shuuichi
 */

#ifndef CONTROLVELOCITY_H_
#define CONTROLVELOCITY_H_

#include "typedef.h"
#include "Log.h"

void CVL_Init(void);

void CVL_StartDriveMotor(void);
void CVL_StopDriveMotor(void);

void CVL_SetTargetUpAccel(float32_t upAccel);
void CVL_SetTargetDownAccel(float32_t downAccel);
void CVL_SetTarget(float32_t target);

void CVL_ControlTask(void);

float32_t CVL_GetTarget(void);
float32_t CVL_GetVelocity(void);
float32_t CVL_GetErrorNow(void);
float32_t CVL_GetDistance(void);
float32_t CVL_GetEncoderDiff(void);

LOG_StrControlVelocityHeader *CVL_GetLogHeader(void);
LOG_StrControlVelocityArray  *CVL_GetLogArray(void);

#endif /* CONTROLVELOCITY_H_ */
