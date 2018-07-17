/*
 * FunctionTimer.c
 *
 *  Created on: 2018/07/02
 *      Author: hirai
 */

#include "FunctionTimer.h"

typedef struct strDuty
{
	uint16_t Duty;
	uint16_t TgrxMax;
	uint16_t DivideValue;
}StrDuty;

static StrDuty st_BuzzerDuty;
static StrDuty st_RightMotorDuty;
static StrDuty st_LeftMotorDuty;
static StrDuty st_SteerMotorDuty;

void FTR_Init(void)
{
	st_BuzzerDuty.Duty = 0;
	st_BuzzerDuty.TgrxMax = R_MTU0_GetTGRA();
	st_BuzzerDuty.DivideValue = 100;

	st_RightMotorDuty.Duty = 0;
	st_RightMotorDuty.TgrxMax = R_MTU3_GetTGRB();
	st_RightMotorDuty.DivideValue = 100;

	st_LeftMotorDuty.Duty = 0;
	st_LeftMotorDuty.TgrxMax = R_MTU3_GetTGRD();
	st_LeftMotorDuty.DivideValue = 100;

	st_SteerMotorDuty.Duty = 0;
	st_SteerMotorDuty.TgrxMax = R_MTU4_GetTGRB();
	st_SteerMotorDuty.DivideValue = 100;
}


void FTR_StartBuzzerTimer(void)
{
	R_MTU3_C0_Start();
}

void FTR_SetBuzzerDuty(uint16_t duty)
{
	uint32_t temp;

	temp = st_BuzzerDuty.TgrxMax * duty;
	temp /= st_BuzzerDuty.DivideValue;

	if(temp > st_BuzzerDuty.TgrxMax)
	{
		temp = st_BuzzerDuty.TgrxMax;
	}

	st_BuzzerDuty.Duty = temp;
	R_MTU0_SetTGRA(temp);
}


void FTR_StartRightEncoderTimer(void)
{
	R_MTU3_C1_Start();
}

uint16_t FTR_GetRightEncoderCount(void)
{
	return R_MTU1_GetTcnt();
}


void FTR_StartLeftEncoderTimer(void)
{
	R_MTU3_C2_Start();
}

uint16_t FTR_GetLeftEncoderCount(void)
{
	return R_MTU2_GetTcnt();
}


void FTR_StartRightMotorTimer(void)
{
	R_MTU3_C3_Start();
}

void FTR_SetRightMotorDuty(uint16_t duty)
{
	uint32_t temp;

	temp = st_RightMotorDuty.TgrxMax * duty;
	temp /= st_RightMotorDuty.DivideValue;

	if(temp > st_RightMotorDuty.TgrxMax)
	{
		temp = st_RightMotorDuty.TgrxMax;
	}

	st_RightMotorDuty.Duty = temp;
	R_MTU3_SetTGRA(temp);
}

uint16_t FTR_GetRightMotorDuty(void)
{
	return st_RightMotorDuty.Duty;
}


void FTR_StartLeftMotorTimer(void)
{
	R_MTU3_C3_Start();
}

void FTR_SetLeftMotorDuty(uint16_t duty)
{
	uint32_t temp;

	temp = st_LeftMotorDuty.TgrxMax * duty;
	temp /= st_LeftMotorDuty.DivideValue;

	if(temp > st_LeftMotorDuty.TgrxMax)
	{
		temp = st_LeftMotorDuty.TgrxMax;
	}

	st_LeftMotorDuty.Duty = temp;
	R_MTU3_SetTGRC(temp);
}

uint16_t FTR_GetLeftMotorDuty(void)
{
	return st_LeftMotorDuty.Duty;
}


void FTR_StartSensorMotorTimer(void)
{
	R_MTU3_C4_Start();
}

void FTR_SetSensorMotorDuty(uint16_t duty)
{
	uint32_t temp;

	temp = st_SteerMotorDuty.TgrxMax * duty;
	temp /= st_SteerMotorDuty.DivideValue;

	if(temp > st_SteerMotorDuty.TgrxMax)
	{
		temp = st_SteerMotorDuty.TgrxMax;
	}

	st_SteerMotorDuty.Duty = temp;
	R_MTU4_SetTGRA(temp);
}

uint16_t FTR_GetSensorMotorDuty(void)
{
	return st_SteerMotorDuty.Duty;
}
