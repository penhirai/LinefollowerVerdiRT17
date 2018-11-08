/*
 * ControlSensorAngle.h
 *
 *  Created on: 2018/10/14
 *      Author: shuuichi
 */

#ifndef CONTROLSENSORANGLE_H_
#define CONTROLSENSORANGLE_H_

#include "typedef.h"
#include "Log.h"

void CSA_Init(void);

void CSA_StartSensorMotor(void);
void CSA_StopSensorMotor(void);
void CSA_StartSensorTask(void);

void CSA_ControlSensorTask(void);

float32_t CSA_GetSensorTheta(void);


LOG_StrControlSensorHeader *CSA_GetLogHeader(void);
LOG_StrControlSensorArray  *CSA_GetLogArray(void);

#endif /* CONTROLSENSORANGLE_H_ */
