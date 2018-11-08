/*
 * FunctionTimer.h
 *
 *  Created on: 2018/07/02
 *      Author: hirai
 */

#ifndef FUNCTIONTIMER_H_
#define FUNCTIONTIMER_H_

#include "typedef.h"
#include "Log.h"


void FTR_Init(void);

void FTR_StartBuzzerTimer(void);
void FTR_SetBuzzerDuty(float32_t duty);

void FTR_StartRightEncoderTimer(void);
uint16_t FTR_GetRightEncoderCount(void);
void FTR_StartLeftEncoderTimer(void);
uint16_t FTR_GetLeftEncoderCount(void);

void FTR_StartRightMotorTimer(void);
void FTR_SetTransitionRightMotorDuty(float32_t duty);
void FTR_SetAngularRightMotorDuty(float32_t duty);
float32_t FTR_GetRightMotorDuty(void);

void FTR_StartLeftMotorTimer(void);
void FTR_SetTransitionLeftMotorDuty(float32_t duty);
void FTR_SetAngularLeftMotorDuty(float32_t duty);
float32_t FTR_GetLeftMotorDuty(void);

void FTR_StartSensorMotorTimer(void);
void FTR_SetSensorMotorDuty(float32_t duty, float32_t offset);
float32_t FTR_GetSensorMotorDuty(void);


LOG_StrControlVelocityDutyArray *FTR_GetVelocityLogDutyArray(void);
LOG_StrControlSensorDutyArray   *FTR_GetSensorLogDutyArray(void);

#endif /* FUNCTIONTIMER_H_ */
