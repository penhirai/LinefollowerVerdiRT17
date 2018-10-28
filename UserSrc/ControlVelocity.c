/*
 * ControlVelocity.c
 *
 *  Created on: 2018/10/18
 *      Author: shuuichi
 */

#include "ControlVelocity.h"
#include "FunctionTimer.h"
#include "Setting.h"
#include "ControlStructure.h"
#include <r_cg_port.h>

typedef struct strEncoderFactor
{
	uint16_t Now;
	uint16_t Past;
	int32_t Diff;
	int32_t Sum;
}StrEncoderFactor;

typedef struct strEncoder
{
	StrEncoderFactor Left;
	StrEncoderFactor Right;
	float32_t Average;
	float32_t Diff;
	float32_t VecDiff;
	float32_t SumAverage;
	float32_t Velocity;
	float32_t Distance;
}StrEncoder;

typedef enum enmTargetState
{
	UP = 0,
	DOWN,
	NEUTRAL
}EnmTargetState;

typedef struct strController
{
	float32_t Target;				// [m/s]
	float32_t InstantTarget;		// [m/s]
	float32_t TargetStepAbs;		// [m/s]
	EnmTargetState TargetState;
	float32_t TargetUpAccel;		// [m/ss]
	float32_t TargetDownAccel;		// [m/ss]
	CST_StrError Error;
	CST_StrGain Gain;
}StrController;

 StrEncoder st_Encoder;
static StrController st_Controller;


static void st_CalcEncoder(void);
static void st_UpdateTarget(void);
static void st_CalcError(void);
static void st_CalcGain(void);
static void st_SetMotorDuty(void);

void CVL_Init(void)
{
	st_Encoder.Left.Now   = 0;
	st_Encoder.Left.Past  = 0;
	st_Encoder.Left.Diff  = 0;
	st_Encoder.Left.Sum   = 0;
	st_Encoder.Right.Now  = 0;
	st_Encoder.Right.Past = 0;
	st_Encoder.Right.Diff = 0;
	st_Encoder.Right.Sum  = 0;
	st_Encoder.Average    = 0.0;
	st_Encoder.Diff       = 0.0;
	st_Encoder.VecDiff    = 0.0;
	st_Encoder.SumAverage = 0.0;
	st_Encoder.Velocity   = 0.0;
	st_Encoder.Distance   = 0.0;

	st_Controller.Target        = 0.0;
	st_Controller.InstantTarget = 0.0;
	st_Controller.TargetStepAbs = 0.0;
	st_Controller.TargetState   = NEUTRAL;
	st_Controller.TargetUpAccel   = 5.0;
	st_Controller.TargetDownAccel = 5.0;
	st_Controller.Error.Now  = 0.0;
	st_Controller.Error.Past = 0.0;
	st_Controller.Error.Sum  = 0.0;
	st_Controller.Error.Factor.FF = 0.0;
	st_Controller.Error.Factor.P  = 0.0;
	st_Controller.Error.Factor.I  = 0.0;
	st_Controller.Error.Factor.D  = 0.0;

	st_Controller.Gain.Scale      = 1.0;
	st_Controller.Gain.Factor.FF  = 15.0;  // 無負荷時計測データの傾き
	st_Controller.Gain.Factor.P   = 2.0;
	st_Controller.Gain.Factor.I   = 0.1;
	st_Controller.Gain.Factor.D   = 0.0;

	CVL_SetTargetUpAccel(st_Controller.TargetUpAccel);
	CVL_SetTargetDownAccel(st_Controller.TargetDownAccel);
	CVL_SetTarget(st_Controller.Target);

	FTR_StartLeftEncoderTimer();
	FTR_StartRightEncoderTimer();

	FTR_StartLeftMotorTimer();
	FTR_StartRightMotorTimer();
}


void CVL_StartDriveMotor(void)
{
	R_PORT_EnmPort state = R_PORT_HIGH;

	R_PORT_SetPB2(state);

	st_Controller.Error.Factor.I = 0.0;

	FTR_SetTransitionLeftMotorDuty(0.0);
	FTR_SetTransitionRightMotorDuty(0.0);
}


void CVL_StopDriveMotor(void)
{
	R_PORT_EnmPort state = R_PORT_LOW;

	R_PORT_SetPB2(state);
}


void CVL_SetTargetUpAccel(float32_t upAccel)
{
	st_Controller.TargetUpAccel = upAccel;
}


void CVL_SetTargetDownAccel(float32_t downAccel)
{
	st_Controller.TargetDownAccel = downAccel;
}


void CVL_SetTarget(float32_t target)
{
	st_Controller.Target = target;

	if(st_Controller.Target > st_Controller.InstantTarget)
	{
		st_Controller.TargetState = UP;
		st_Controller.TargetStepAbs = st_Controller.TargetUpAccel * PERIOD_INTERRUPT;
	}
	else
	{
		st_Controller.TargetState = DOWN;
		st_Controller.TargetStepAbs = -st_Controller.TargetDownAccel * PERIOD_INTERRUPT;
	}
}


void CVL_ControlTask(void)
{
	st_CalcEncoder();

	st_UpdateTarget();

	st_CalcError();
	//st_CalcGain();

	st_SetMotorDuty();
}


float32_t CVL_GetTarget(void)
{
	return st_Controller.InstantTarget;
}


float32_t CVL_GetVelocity(void)
{
	return st_Encoder.Velocity;
}


float32_t CVL_GetErrorNow(void)
{
	return st_Controller.Error.Now;
}


float32_t CVL_GetDistance(void)
{
	return st_Encoder.Distance;
}


float32_t CVL_GetEncoderDiff(void)
{
	return st_Encoder.VecDiff;
}


static void st_CalcEncoder(void)
{
	int32_t leftDiff;
	int32_t leftDiffUpBorder;
	int32_t leftDiffDownBorder;
	int32_t rightDiff;
	int32_t rightDiffUpBorder;
	int32_t rightDiffDownBorder;
	float32_t kTemp;

	st_Encoder.Left.Now  = FTR_GetLeftEncoderCount();
	st_Encoder.Right.Now = FTR_GetRightEncoderCount();

	leftDiff  = (int32_t)st_Encoder.Left.Now  - (int32_t)st_Encoder.Left.Past;
	rightDiff = (int32_t)st_Encoder.Right.Now - (int32_t)st_Encoder.Right.Past;
	leftDiffDownBorder  = leftDiff  & 0xFFFF0000;
	rightDiffDownBorder = rightDiff & 0xFFFF0000;
	leftDiffUpBorder  = leftDiff  & 0xFFFF;
	rightDiffUpBorder = rightDiff & 0xFFFF;
	//leftDiff3 = leftDiff & 0xFFFF0000;
	//rightDiff3 = rightDiff & 0xFFFF0000;

	if(leftDiffUpBorder > 30000)
	{
		if(leftDiffDownBorder == 0)
		{
			leftDiff = leftDiffUpBorder - 0xFFFF;
		}
	}
	else
	{
		leftDiff = leftDiffUpBorder;
	}
	if(rightDiffUpBorder > 30000)
	{
		if(rightDiffDownBorder == 0)
		{
			rightDiff = rightDiffUpBorder - 0xFFFF;
		}
	}
	else
	{
		rightDiff = rightDiffUpBorder;
	}
	st_Encoder.Left.Diff  = leftDiff;
	st_Encoder.Right.Diff = rightDiff;

	st_Encoder.Left.Sum  += leftDiff;
	st_Encoder.Right.Sum += rightDiff;

	st_Encoder.Average    = (float32_t)(st_Encoder.Left.Diff + st_Encoder.Right.Diff) * 0.5;
	st_Encoder.SumAverage = (float32_t)(st_Encoder.Left.Sum  + st_Encoder.Right.Sum)  * 0.5;
	st_Encoder.Diff       = (float32_t)(st_Encoder.Right.Diff - st_Encoder.Left.Diff);

	kTemp = (ENCODER_PULSE_MAX_INV) * (PI * D_TIRE) * GEAR_RATIO_INV;
	st_Encoder.Velocity = st_Encoder.Average    * kTemp * PERIOD_INTERRUPT_INV;
	//st_Encoder.Velocity = st_Encoder.Average;
	st_Encoder.Distance = st_Encoder.SumAverage * kTemp;
	st_Encoder.VecDiff = st_Encoder.Diff * kTemp * PERIOD_INTERRUPT_INV;

	st_Encoder.Left.Past  = st_Encoder.Left.Now;
	st_Encoder.Right.Past = st_Encoder.Right.Now;
}


static void st_UpdateTarget(void)
{
	st_Controller.InstantTarget += st_Controller.TargetStepAbs;

	if(st_Controller.TargetState == UP)
	{
		if(st_Controller.InstantTarget >= st_Controller.Target)
		{
			st_Controller.InstantTarget = st_Controller.Target;
		}
	}
	else
	{
		if(st_Controller.InstantTarget <= st_Controller.Target)
		{
			st_Controller.InstantTarget = st_Controller.Target;
		}
	}
}


static void st_CalcError(void)
{
	CST_StrFactor factorBuff;
	float32_t sum;

	st_Controller.Error.Now = st_Controller.InstantTarget - st_Encoder.Velocity;

	st_Controller.Error.Factor.FF = st_Controller.Error.Now;
	st_Controller.Error.Factor.P  = st_Controller.Error.Now;
	st_Controller.Error.Factor.I += st_Controller.Error.Now;
	st_Controller.Error.Factor.D  = st_Controller.Error.Now - st_Controller.Error.Past;

	factorBuff.FF = st_Controller.Gain.Factor.FF * st_Controller.Error.Factor.FF;
	factorBuff.P  = st_Controller.Gain.Factor.P  * st_Controller.Error.Factor.P;
	factorBuff.I  = st_Controller.Gain.Factor.I  * st_Controller.Error.Factor.I;
	factorBuff.D  = st_Controller.Gain.Factor.D  * st_Controller.Error.Factor.D;
	sum = factorBuff.P + factorBuff.I + factorBuff.D;
	sum *= st_Controller.Gain.Scale;

	st_Controller.Error.Sum = factorBuff.FF + sum;

	st_Controller.Error.Past = st_Controller.Error.Now;
}


static void st_CalcGain(void)
{
	st_Controller.Error.Sum;
}


static void st_SetMotorDuty(void)
{
	FTR_SetTransitionLeftMotorDuty(st_Controller.Error.Sum);
	FTR_SetTransitionRightMotorDuty(st_Controller.Error.Sum);
}
