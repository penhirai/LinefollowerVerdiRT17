/*
 * FunctionTimer.h
 *
 *  Created on: 2018/07/02
 *      Author: hirai
 */

#ifndef FUNCTIONTIMER_H_
#define FUNCTIONTIMER_H_

#include "typedef.h"


void FTR_Init(void);

void FTR_StartBuzzerTimer(void);
void FTR_SetBuzzerDuty(float32_t duty);

void FTR_StartRightEncoderTimer(void);
uint16_t FTR_GetRightEncoderCount(void);
void FTR_StartLeftEncoderTimer(void);
uint16_t FTR_GetLeftEncoderCount(void);

void FTR_StartRightMotorTimer(void);
void FTR_SetRightMotorDuty(float32_t duty);
void FTR_AddRightMotorDuty(float32_t duty);
uint16_t FTR_GetRightMotorDuty(void);

void FTR_StartLeftMotorTimer(void);
void FTR_SetLeftMotorDuty(float32_t duty);
void FTR_AddLeftMotorDuty(float32_t duty);
uint16_t FTR_GetLeftMotorDuty(void);

void FTR_StartSensorMotorTimer(void);
void FTR_SetSensorMotorDuty(float32_t duty);
uint16_t FTR_GetSensorMotorDuty(void);

#endif /* FUNCTIONTIMER_H_ */
