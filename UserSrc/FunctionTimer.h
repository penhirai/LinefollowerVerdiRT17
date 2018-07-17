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
void FTR_SetBuzzerDuty(uint16_t duty);

void FTR_StartRightEncoderTimer(void);
uint16_t FTR_GetRightEncoderCount(void);
void FTR_StartLeftEncoderTimer(void);
uint16_t FTR_GetLeftEncoderCount(void);

void FTR_StartRightMotorTimer(void);
void FTR_SetRightMotorDuty(uint16_t duty);
uint16_t FTR_GetRightMotorDuty(void);

void FTR_StartLeftMotorTimer(void);
void FTR_SetLeftMotorDuty(uint16_t duty);
uint16_t FTR_GetLeftMotorDuty(void);

void FTR_StartSensorMotorTimer(void);
void FTR_SetSensorMotorDuty(uint16_t duty);
uint16_t FTR_GetSensorMotorDuty(void);

#endif /* FUNCTIONTIMER_H_ */
