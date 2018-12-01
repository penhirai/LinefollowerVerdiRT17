/*
 * ControlAngularVelocity.c
 *
 *  Created on: 2018/10/20
 *      Author: shuuichi
 */

#include "ControlAngularVelocity.h"
#include "FunctionTimer.h"
#include "Sensor.h"
#include "Setting.h"
#include "ControlStructure.h"
#include "ControlVelocity.h"
#include <mathf.h>
#include "ControlSensorAngle.h"
#include <r_cg_port.h>

//typedef struct strEncoderFactor
//{
//	uint16_t Now;
//	uint16_t Past;
//	uint16_t Diff;
//}StrEncoderFactor;
//
//typedef struct strEncoder
//{
//	StrEncoderFactor Left;
//	StrEncoderFactor Right;
//	float32_t Average;
//	float32_t Velocity;
//}StrEncoder;

typedef struct strVirtualGeometry
{
	float32_t Theta;
	float32_t ThetaDeg;
	float32_t OmegaRatio;
	float32_t Radius;
	float32_t Velocity;
}StrVirtualGeometry;

typedef enum enmTargetState
{
	UP = 0,
	DOWN,
	NEUTRAL
}EnmTargetState;

typedef struct strController
{
	float32_t Target;				// [deg/s]
	float32_t InstantTarget;		// [deg/s]
	float32_t TargetStepAbs;		// [deg/s]
	float32_t TargetUpAccel;		// [deg/ss]
	float32_t TargetDownAccel;		// [deg/ss]
	CST_StrError Error;
	CST_StrGain Gain;
	EnmTargetState TargetState;
}StrController;

static SSR_StrSensorData st_SensorData;
static float32_t st_BodyOmega;
static float32_t st_BodyAngle;
//static StrEncoder st_Encoder;
//static float32_t st_Theta;
//static float32_t st_Radius;
//static float32_t st_Velocity;
static StrVirtualGeometry st_VirtualGeometry;
static StrController st_Controller;

static LOG_StrControlVelocityHeader st_LogHeader;
static LOG_StrControlVelocityArray  st_LogArray;


static void st_CalcTarget(void);
static void st_CalcGyro(void);
static void st_UpdateTarget(void);
static void st_CalcError(void);
static void st_CalcGain(void);
static void st_SetMotorDuty(void);

void CAV_Init(void)
{
//	st_Encoder.Left.Now   = 0;
//	st_Encoder.Left.Past  = 0;
//	st_Encoder.Left.Diff  = 0;
//	st_Encoder.Right.Now  = 0;
//	st_Encoder.Right.Past = 0;
//	st_Encoder.Right.Diff = 0;
//	st_Encoder.Average    = 0.0;
//	st_Encoder.Velocity   = 0.0;

	st_Controller.Target        = 0.0;
	st_Controller.InstantTarget = 0.0;
	st_Controller.TargetStepAbs = 0.0;
	st_Controller.TargetState   = NEUTRAL;
	st_Controller.TargetUpAccel   = 30000.0;
	st_Controller.TargetDownAccel = 30000.0;
	st_Controller.Error.Now  = 0.0;
	st_Controller.Error.Past = 0.0;
	st_Controller.Error.Sum  = 0.0;
	st_Controller.Error.Factor.FF = 0.0;
	st_Controller.Error.Factor.P  = 0.0;
	st_Controller.Error.Factor.I  = 0.0;
	st_Controller.Error.Factor.D  = 0.0;

	st_Controller.Gain.Scale      = 1.0;
	st_Controller.Gain.Factor.FF  = 1.0 / 1200.0;	// 30 / 1200 実働を見てざっくり
	st_Controller.Gain.Factor.P   = 0.08;
	st_Controller.Gain.Factor.I   = 0.008;
	st_Controller.Gain.Factor.D   = 0.0;

	//st_SensorData = SSR_GetSensorStructure();
	//st_BodyOmega = SSR_GetGyroData();
	st_BodyOmega = 0.0;
	st_BodyAngle = 0.0;


	st_VirtualGeometry.Theta = 0.0;
	st_VirtualGeometry.ThetaDeg = 0.0;
	st_VirtualGeometry.Radius = 0.0;
	st_VirtualGeometry.OmegaRatio = 2.0;
	st_VirtualGeometry.Velocity = 0.0;

	CAV_SetTargetUpAccel(st_Controller.TargetUpAccel);
	CAV_SetTargetDownAccel(st_Controller.TargetDownAccel);
	CAV_SetTarget(st_Controller.Target);
}


void CAV_StartDriveMotor(void)
{
	R_PORT_EnmPort state = R_PORT_HIGH;

	R_PORT_SetPB2(state);

	st_Controller.Error.Factor.I = 0.0;

	FTR_SetTransitionLeftMotorDuty(0.0);
	FTR_SetTransitionRightMotorDuty(0.0);
}


void CAV_StopDriveMotor(void)
{
	R_PORT_EnmPort state = R_PORT_LOW;

	R_PORT_SetPB2(state);
}


void CAV_SetTargetUpAccel(float32_t upAccel)
{
	st_Controller.TargetUpAccel = upAccel;
}


void CAV_SetTargetDownAccel(float32_t downAccel)
{
	st_Controller.TargetDownAccel = downAccel;
}


void CAV_SetTarget(float32_t target)
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


void CAV_ControlTask(void)
{
	st_CalcTarget();

	st_CalcGyro();

	st_UpdateTarget();

	st_CalcError();
//	st_CalcGain();

	st_SetMotorDuty();
}

float32_t CAV_GetTarget(void)
{
	return st_Controller.InstantTarget;
}


float32_t CAV_GetVelocity(void)
{
	return st_BodyOmega;
}


float32_t CAV_GetErrorNow(void)
{
	return st_Controller.Error.Now;
}


float32_t CAV_GetAngle(void)
{
	return st_BodyAngle;
}


float32_t CAV_GetRadius(void)
{
	return st_VirtualGeometry.Radius;
}


float32_t CAV_GetVirtualThetaDeg(void)
{
	return st_VirtualGeometry.ThetaDeg;
}


void CAV_ClearAngle(void)
{
	st_BodyAngle = 0.0;
}


LOG_StrControlVelocityHeader *CAV_GetLogHeader(void)
{
	st_LogHeader.Gain = st_Controller.Gain;

	return &st_LogHeader;
}


LOG_StrControlVelocityArray  *CAV_GetLogArray(void)
{
	float32_t memsOmega;

	st_SensorData = SSR_GetSensorData();
	memsOmega = + K_GYRO * (float32_t)st_SensorData.Gyro;	// [deg/s]

	st_LogArray.EncoderVelocity = st_BodyOmega;
	st_LogArray.EncoderDistance = st_BodyAngle;
	st_LogArray.MemsVelocity    = memsOmega;
	st_LogArray.TargetInstance  = st_Controller.InstantTarget;
	st_LogArray.Error    = st_Controller.Error.Factor;
	st_LogArray.ErrorSum = st_Controller.Error.Sum;

	return &st_LogArray;
}


static void st_CalcTarget(void)
{
	float32_t target;
	float32_t radius;

	st_VirtualGeometry.ThetaDeg = CSA_GetSensorTheta();
	st_VirtualGeometry.Theta = K_ANGULAR_VELOCITY * st_VirtualGeometry.ThetaDeg;
	if(st_VirtualGeometry.Theta == 0.0)
	{
		radius = 100.0;
	}
	else
	{
		radius = 0.5 * LENGTH_SENSOR / sinf(0.5 * st_VirtualGeometry.Theta); // sin(theta/2)
	}
	st_VirtualGeometry.Radius = radius;
//	if(radius < 0.0)
//	{
//		//radius *= -1.0;
//	}

	st_VirtualGeometry.Velocity = CVL_GetVelocity();
	target = st_VirtualGeometry.Velocity / radius;		// [rad/s]
	target *= K_ANGULAR_VELOCITY_INV;		// [deg/s]
	target *= st_VirtualGeometry.OmegaRatio;
	CAV_SetTarget(target);
}


static void st_CalcGyro(void)
{
	float32_t encDiff;

	encDiff = CVL_GetEncoderDiff();
	st_BodyOmega = K_ANGULAR_VELOCITY_INV * encDiff * (16.667);	 //トレッド(0.12) の逆数
	/*
	st_SensorData = SSR_GetSensorData();
	st_BodyOmega = + K_GYRO * (float32_t)st_SensorData.Gyro;	// [deg/s]
	st_BodyOmega += 10.0;
	if((-3.0 < st_BodyOmega) && (st_BodyOmega < 3.0))
	{
		st_BodyOmega = 0.0;
	}
	*/
	st_BodyAngle += st_BodyOmega * PERIOD_INTERRUPT;
}
/*
static void st_CalcEncoder(void)
{
	st_Encoder.Left.Now  = FTR_GetLeftEncoderCount();
	st_Encoder.Right.Now = FTR_GetRightEncoderCount();

	st_Encoder.Left.Diff  = st_Encoder.Left.Now  - st_Encoder.Left.Past;
	st_Encoder.Right.Diff = st_Encoder.Right.Now - st_Encoder.Right.Past;

	st_Encoder.Average = (st_Encoder.Left.Diff + st_Encoder.Right.Diff) * 0.5;

	st_Encoder.Velocity = st_Encoder.Average * (ENCODER_PULSE_MAX_INV) * (PI * D_TIRE) * (PERIOD_INTERRUPT_INV);

	st_Encoder.Left.Past  = st_Encoder.Left.Now;
	st_Encoder.Right.Past = st_Encoder.Right.Now;
}
*/


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

	st_Controller.Error.Now = st_Controller.InstantTarget - st_BodyOmega;

	st_Controller.Error.Factor.FF = st_Controller.InstantTarget;
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
	FTR_SetAngularLeftMotorDuty(-st_Controller.Error.Sum);
	FTR_SetAngularRightMotorDuty(st_Controller.Error.Sum);
}
