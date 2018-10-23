/*
 * FunctionTimer.c
 *
 *  Created on: 2018/07/02
 *      Author: hirai
 */

#include "FunctionTimer.h"
#include <r_cg_port.h>

typedef struct strDuty
{
	float32_t Duty;
	uint16_t TgrxMax;
	float32_t DivideValue;
}StrDuty;

static StrDuty st_BuzzerDuty;
static StrDuty st_RightMotorDuty;
static StrDuty st_LeftMotorDuty;
static StrDuty st_SensorMotorDuty;

static void st_SetLeftDriveCw(void);
static void st_SetLeftDriveCcw(void);
static void st_SetRightDriveCw(void);
static void st_SetRightDriveCcw(void);
static void st_SetSensorDriveCw(void);
static void st_SetSensorDriveCcw(void);


void FTR_Init(void)
{
	st_BuzzerDuty.Duty = 0.0;
	R_MTU0_SetTGRC(st_BuzzerDuty.Duty);
	st_BuzzerDuty.TgrxMax = R_MTU0_GetTGRD();
	st_BuzzerDuty.DivideValue = 1.0/100.0;

	st_RightMotorDuty.Duty = 0.0;
	R_MTU4_SetTGRA(st_RightMotorDuty.Duty);
	st_RightMotorDuty.TgrxMax = R_MTU4_GetTGRB();
	st_RightMotorDuty.DivideValue = 1.0/100.0;

	st_LeftMotorDuty.Duty = 0.0;
	R_MTU4_SetTGRC(st_LeftMotorDuty.Duty);
	st_LeftMotorDuty.TgrxMax = R_MTU4_GetTGRD();
	st_LeftMotorDuty.DivideValue = 1.0/100.0;

	st_SensorMotorDuty.Duty = 0.0;
	R_MTU3_SetTGRC(st_SensorMotorDuty.Duty);
	st_SensorMotorDuty.TgrxMax = R_MTU3_GetTGRD();
	st_SensorMotorDuty.DivideValue = 1.0/100.0;
}


void FTR_StartBuzzerTimer(void)
{
	R_MTU3_C0_Start();
}

void FTR_SetBuzzerDuty(float32_t duty)
{
	float32_t temp;
	uint16_t tempDuty;

	temp = (float32_t)st_BuzzerDuty.TgrxMax * duty;
	temp *= st_BuzzerDuty.DivideValue;

	tempDuty = (uint16_t)temp;
	if(tempDuty > st_BuzzerDuty.TgrxMax)
	{
		tempDuty = st_BuzzerDuty.TgrxMax;
	}

	st_BuzzerDuty.Duty = tempDuty;
	R_MTU0_SetTGRC(tempDuty);
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
	R_MTU3_C4_Start();
}

void FTR_SetRightMotorDuty(float32_t duty)
{
	float32_t temp;
	uint16_t tempDuty;

	st_RightMotorDuty.Duty = duty;

	if(duty >= 0.0)
	{
		st_SetRightDriveCw();
	}
	else
	{
		duty *= -1.0;
		st_SetRightDriveCcw();
	}

	if(duty > 100.0)
	{
		duty = 100.0;
	}

	temp = (float32_t)st_RightMotorDuty.TgrxMax * duty;
	temp *= st_RightMotorDuty.DivideValue;

	tempDuty = (uint16_t)temp;
	if(tempDuty >= st_RightMotorDuty.TgrxMax)
	{
		tempDuty = st_RightMotorDuty.TgrxMax - 1;
	}

	R_MTU4_SetTGRA(tempDuty);
}

void FTR_AddRightMotorDuty(float32_t duty)
{
	st_RightMotorDuty.Duty += duty;

	FTR_SetRightMotorDuty(st_RightMotorDuty.Duty);
}

float32_t FTR_GetRightMotorDuty(void)
{
	return st_RightMotorDuty.Duty;
}


void FTR_StartLeftMotorTimer(void)
{
	R_MTU3_C4_Start();
}

void FTR_SetLeftMotorDuty(float32_t duty)
{
	float32_t temp;
	uint16_t tempDuty;

	st_LeftMotorDuty.Duty = duty;

	if(duty >= 0.0)
	{
		st_SetLeftDriveCw();
	}
	else
	{
		duty *= -1.0;
		st_SetLeftDriveCcw();
	}

	if(duty > 100.0)
	{
		duty = 100.0;
	}

	temp = (float32_t)st_LeftMotorDuty.TgrxMax * duty;
	temp *= st_LeftMotorDuty.DivideValue;

	tempDuty = (uint16_t)temp;
	if(tempDuty >= st_LeftMotorDuty.TgrxMax)
	{
		tempDuty = st_LeftMotorDuty.TgrxMax - 1;
	}

	R_MTU4_SetTGRC(tempDuty);
}

void FTR_AddLeftMotorDuty(float32_t duty)
{
	st_LeftMotorDuty.Duty += duty;

	FTR_SetLeftMotorDuty(st_LeftMotorDuty.Duty);
}

float32_t FTR_GetLeftMotorDuty(void)
{
	return st_LeftMotorDuty.Duty;
}


void FTR_StartSensorMotorTimer(void)
{
	R_MTU3_C3_Start();
}

void FTR_SetSensorMotorDuty(float32_t duty)
{
	float32_t temp;
	uint16_t tempDuty;

	if(duty >= 0.0)
	{
		st_SetSensorDriveCw();
	}
	else
	{
		duty *= -1.0;
		st_SetSensorDriveCcw();
	}

	if(duty > 100.0)
	{
		duty = 100.0;
	}

	temp = (float32_t)st_SensorMotorDuty.TgrxMax * duty;
	temp *= st_SensorMotorDuty.DivideValue;

	tempDuty = (uint16_t)temp;
	if(tempDuty >= st_SensorMotorDuty.TgrxMax)
	{
		tempDuty = st_SensorMotorDuty.TgrxMax - 1;
	}

	st_SensorMotorDuty.Duty = tempDuty;
	R_MTU3_SetTGRC(tempDuty);
}

float32_t FTR_GetSensorMotorDuty(void)
{
	return st_SensorMotorDuty.Duty;
}


static void st_SetLeftDriveCw(void)
{
	R_PORT_EnmPort state = R_PORT_LOW;

	R_PORT_SetPD0(state);
	R_PORT_SetPD1(state);

	state = R_PORT_HIGH;
	for(volatile i = 0; i < 10; ++i) ;

	R_PORT_SetPD0(state);
}


static void st_SetLeftDriveCcw(void)
{
	R_PORT_EnmPort state = R_PORT_LOW;

	R_PORT_SetPD0(state);
	R_PORT_SetPD1(state);

	state = R_PORT_HIGH;
	for(volatile i = 0; i < 10; ++i) ;

	R_PORT_SetPD1(state);
}


static void st_SetRightDriveCw(void)
{
	R_PORT_EnmPort state = R_PORT_LOW;

	R_PORT_SetPE0(state);
	R_PORT_SetPE3(state);

	state = R_PORT_HIGH;
	for(volatile i = 0; i < 10; ++i) ;

	R_PORT_SetPE3(state);
}


static void st_SetRightDriveCcw(void)
{
	R_PORT_EnmPort state = R_PORT_LOW;

	R_PORT_SetPE0(state);
	R_PORT_SetPE3(state);

	state = R_PORT_HIGH;
	for(volatile i = 0; i < 10; ++i) ;

	R_PORT_SetPE0(state);
}

static void st_SetSensorDriveCw(void)
{
	R_PORT_EnmPort state = R_PORT_LOW;

	R_PORT_SetPE4(state);
	R_PORT_SetPB0(state);

	state = R_PORT_HIGH;
	for(volatile i = 0; i < 10; ++i) ;

	R_PORT_SetPE4(state);
}


static void st_SetSensorDriveCcw(void)
{
	R_PORT_EnmPort state = R_PORT_LOW;

	R_PORT_SetPE4(state);
	R_PORT_SetPB0(state);

	state = R_PORT_HIGH;
	for(volatile i = 0; i < 10; ++i) ;

	R_PORT_SetPB0(state);
}
