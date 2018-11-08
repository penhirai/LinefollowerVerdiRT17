/*
 * Log.c
 *
 *  Created on: 2018/06/23
 *      Author: shuuichi
 */

#include "Log.h"
#include "ControlSensorAngle.h"
#include "ControlVelocity.h"
#include "ControlAngularVelocity.h"
#include "FunctionTimer.h"
#include "SciFifo.h"

#define SEND_BUF_SIZE 150

static uint8_t st_SendBuf[SEND_BUF_SIZE];
static uint8_t st_BufSize;

#define LOG_CONTROL_MAX	2500
#define LOG_COURCE_MAX	6500


typedef struct strControlVelocityFactor
{
	LOG_StrControlVelocityHeader Header;
	LOG_StrControlVelocityArray  Array[LOG_CONTROL_MAX];
}LOG_StrControlVelocityFactor;

typedef struct strControlVelocity
{
	LOG_StrControlVelocityFactor    Transition;
	LOG_StrControlVelocityFactor    Angular;
	LOG_StrControlVelocityDutyArray MotorArray[LOG_CONTROL_MAX];
}LOG_StrControlVelocity;


typedef struct strControlSensorFactor
{
	LOG_StrControlSensorHeader Header;
	LOG_StrControlSensorArray  Array[LOG_CONTROL_MAX];
}LOG_StrControlSensorFactor;

typedef struct strControlSensor
{
	LOG_StrControlSensorFactor Sensor;
	LOG_StrControlSensorDutyArray MotorArray[LOG_CONTROL_MAX];
}LOG_StrControlSensor;

typedef struct strControlLog
{
	LOG_StrControlVelocity ControlVelocity;
	LOG_StrControlSensor   ControlSensor;
	uint32_t Index;
}LOG_StrControlLog;


typedef struct strCourcelog
{
	LOG_StrCourceLogArray Array[LOG_COURCE_MAX];
	uint32_t Index;
}LOG_StrCourceLog;

static LOG_StrControlLog st_ControlLog;
static LOG_StrCourceLog  st_CourceLog;


void LOG_Init(void)
{
	// 制御系ログ
	LOG_InitControl();

	// コースログ
	LOG_InitCource();
}


void LOG_InitControl(void)
{
	// 並進系ヘッダー
	st_ControlLog.ControlVelocity.Transition.Header = *CVL_GetLogHeader();

	// 回転系ヘッダー
	st_ControlLog.ControlVelocity.Angular.Header    = *CAV_GetLogHeader();

	// センサ系ヘッダー
	st_ControlLog.ControlSensor.Sensor.Header       = *CSA_GetLogHeader();


	for(int32_t i = 0; i < LOG_CONTROL_MAX; ++i)
	{
		// 並進系
		st_ControlLog.ControlVelocity.Transition.Array[i].EncoderVelocity = 0.0;
		st_ControlLog.ControlVelocity.Transition.Array[i].EncoderDistance = 0.0;
		st_ControlLog.ControlVelocity.Transition.Array[i].MemsVelocity = 0.0;
		st_ControlLog.ControlVelocity.Transition.Array[i].MemsDistance = 0.0;
		st_ControlLog.ControlVelocity.Transition.Array[i].TargetInstance  = 0.0;
		st_ControlLog.ControlVelocity.Transition.Array[i].Error.FF = 0.0;
		st_ControlLog.ControlVelocity.Transition.Array[i].Error.P  = 0.0;
		st_ControlLog.ControlVelocity.Transition.Array[i].Error.I  = 0.0;
		st_ControlLog.ControlVelocity.Transition.Array[i].Error.D  = 0.0;
		st_ControlLog.ControlVelocity.Transition.Array[i].ErrorSum = 0.0;

		// 角速度系
		st_ControlLog.ControlVelocity.Angular.Array[i].EncoderVelocity = 0.0;
		st_ControlLog.ControlVelocity.Angular.Array[i].EncoderDistance = 0.0;
		st_ControlLog.ControlVelocity.Angular.Array[i].MemsVelocity = 0.0;
		st_ControlLog.ControlVelocity.Angular.Array[i].MemsDistance = 0.0;
		st_ControlLog.ControlVelocity.Angular.Array[i].TargetInstance  = 0.0;
		st_ControlLog.ControlVelocity.Angular.Array[i].Error.FF = 0.0;
		st_ControlLog.ControlVelocity.Angular.Array[i].Error.P  = 0.0;
		st_ControlLog.ControlVelocity.Angular.Array[i].Error.I  = 0.0;
		st_ControlLog.ControlVelocity.Angular.Array[i].Error.D  = 0.0;
		st_ControlLog.ControlVelocity.Angular.Array[i].ErrorSum = 0.0;

		// 制御系Duty
		st_ControlLog.ControlVelocity.MotorArray[i].LeftDuty  = 0.0;
		st_ControlLog.ControlVelocity.MotorArray[i].RightDuty = 0.0;

		// センサ系
		st_ControlLog.ControlSensor.Sensor.Array[i].SensorAngle     = 0.0;
		st_ControlLog.ControlSensor.Sensor.Array[i].LeftLineSensor  = 0.0;
		st_ControlLog.ControlSensor.Sensor.Array[i].RightLineSensor = 0.0;
		st_ControlLog.ControlSensor.Sensor.Array[i].DiffLineSensor  = 0.0;
		st_ControlLog.ControlSensor.Sensor.Array[i].Target          = 0.0;
		st_ControlLog.ControlSensor.Sensor.Array[i].ErrorP          = 0.0;
		st_ControlLog.ControlSensor.Sensor.Array[i].ErrorSum        = 0.0;

		// センサ系Duty
		st_ControlLog.ControlSensor.MotorArray[i].SensorDuty = 0.0;
	}

	st_CourceLog.Index = 0;
}


void LOG_InitCource(void)
{
	for(int32_t i = 0; i < LOG_COURCE_MAX; ++i)
	{
		st_CourceLog.Array[i].SensorAngle     = 0.0;
		st_CourceLog.Array[i].Velocity        = 0.0;
		st_CourceLog.Array[i].AngularVelocity = 0.0;
		st_CourceLog.Array[i].Distance        = 0.0;
		st_CourceLog.Array[i].MarkerKind      = SSR_COURCE_MARKER_NON;
		st_CourceLog.Array[i].IsChangeFlag    = LOG_CHANGE_FALSE;
	}

	st_CourceLog.Index = 0;
}


void LOG_RecordControl(void)
{
	LOG_StrControlVelocityArray *velocityArray;
	LOG_StrControlVelocityArray *angularArray;
	LOG_StrControlVelocityDutyArray *velocityDutyArray;
	LOG_StrControlSensorArray *sensorArray;
	LOG_StrControlSensorDutyArray *sensorDutyArray;
	uint32_t index;

	velocityArray = CVL_GetLogArray();
	angularArray  = CAV_GetLogArray();
	velocityDutyArray = FTR_GetVelocityLogDutyArray();
	sensorArray = CSA_GetLogArray();
	sensorDutyArray = FTR_GetSensorLogDutyArray();
	index = st_ControlLog.Index;

	st_ControlLog.ControlVelocity.Transition.Array[index] = *velocityArray;
	st_ControlLog.ControlVelocity.Angular.Array[index]    = *angularArray;
	st_ControlLog.ControlVelocity.MotorArray[index]       = *velocityDutyArray;
	st_ControlLog.ControlSensor.Sensor.Array[index]       = *sensorArray;
	st_ControlLog.ControlSensor.MotorArray[index]         = *sensorDutyArray;

	++st_ControlLog.Index;
	if(st_CourceLog.Index >= LOG_CONTROL_MAX)
	{
		st_ControlLog.Index = LOG_CONTROL_MAX - 1;
	}
}


void LOG_RecordCource(SSR_EnmCourceMarkerKind kind)
{
	LOG_StrCourceLogArray array;
	uint32_t index;

	array.SensorAngle = CSA_GetSensorTheta();
	array.Velocity    = CVL_GetVelocity();
	array.Distance    = CVL_GetDistance();
	array.AngularVelocity = CAV_GetVelocity();
	array.MarkerKind  = kind;
	index = st_CourceLog.Index;

	st_CourceLog.Array[index] = array;

	++st_CourceLog.Index;
	if(st_CourceLog.Index >= LOG_COURCE_MAX)
	{
		st_CourceLog.Index = LOG_COURCE_MAX - 1;
	}
}


void LOG_PrintControlRecord(void)
{
	// header
	st_BufSize = sprintf(st_SendBuf, "T FF, %.2f, T P, %.2f, T I, %.2f, T D %.2f, T S, %.2f, A FF, %.2f, A P, %.2f, A I, %.2f, A D, %.2f, A S, %.2f \r\n"
										, st_ControlLog.ControlVelocity.Transition.Header.Gain.Factor.FF
										, st_ControlLog.ControlVelocity.Transition.Header.Gain.Factor.P
										, st_ControlLog.ControlVelocity.Transition.Header.Gain.Factor.I
										, st_ControlLog.ControlVelocity.Transition.Header.Gain.Factor.D
										, st_ControlLog.ControlVelocity.Transition.Header.Gain.Scale
										, st_ControlLog.ControlVelocity.Angular.Header.Gain.Factor.FF
										, st_ControlLog.ControlVelocity.Angular.Header.Gain.Factor.P
										, st_ControlLog.ControlVelocity.Angular.Header.Gain.Factor.I
										, st_ControlLog.ControlVelocity.Angular.Header.Gain.Factor.D
										, st_ControlLog.ControlVelocity.Angular.Header.Gain.Scale);
	SCF_WriteData(st_SendBuf, st_BufSize);

	for(uint32_t i = 0; i < LOG_CONTROL_MAX; ++i)
	{
		st_BufSize = sprintf(st_SendBuf, "[%d], [V], E V, %.2f, E D, %.2f, M V, %.2f, M D, %.2f, T I, %.2f, E F, %.2f, E P, %.2f, E I, %.2f, E D, %.2f, E S, %.2f, "
											, i
											, st_ControlLog.ControlVelocity.Transition.Array[i].EncoderVelocity
											, st_ControlLog.ControlVelocity.Transition.Array[i].EncoderDistance
											, st_ControlLog.ControlVelocity.Transition.Array[i].MemsVelocity
											, st_ControlLog.ControlVelocity.Transition.Array[i].MemsDistance
											, st_ControlLog.ControlVelocity.Transition.Array[i].TargetInstance
											, st_ControlLog.ControlVelocity.Transition.Array[i].Error.FF
											, st_ControlLog.ControlVelocity.Transition.Array[i].Error.P
											, st_ControlLog.ControlVelocity.Transition.Array[i].Error.I
											, st_ControlLog.ControlVelocity.Transition.Array[i].Error.D
											, st_ControlLog.ControlVelocity.Transition.Array[i].ErrorSum);
		SCF_WriteData(st_SendBuf, st_BufSize);

		st_BufSize = sprintf(st_SendBuf, "[A], E V, %.2f, E D, %.2f, M V, %.2f, M D, %.2f, T I, %.2f, E F, %.2f, E P, %.2f, E I, %.2f, E D, %.2f, E S, %.2f, "
											, st_ControlLog.ControlVelocity.Angular.Array[i].EncoderVelocity
											, st_ControlLog.ControlVelocity.Angular.Array[i].EncoderDistance
											, st_ControlLog.ControlVelocity.Angular.Array[i].MemsVelocity
											, st_ControlLog.ControlVelocity.Angular.Array[i].MemsDistance
											, st_ControlLog.ControlVelocity.Angular.Array[i].TargetInstance
											, st_ControlLog.ControlVelocity.Angular.Array[i].Error.FF
											, st_ControlLog.ControlVelocity.Angular.Array[i].Error.P
											, st_ControlLog.ControlVelocity.Angular.Array[i].Error.I
											, st_ControlLog.ControlVelocity.Angular.Array[i].Error.D
											, st_ControlLog.ControlVelocity.Angular.Array[i].ErrorSum);
		SCF_WriteData(st_SendBuf, st_BufSize);

		st_BufSize = sprintf(st_SendBuf, "[D], L D, %.2f, R D, %.2f, "
											, st_ControlLog.ControlVelocity.MotorArray[i].LeftDuty
											, st_ControlLog.ControlVelocity.MotorArray[i].RightDuty);

		st_BufSize = sprintf(st_SendBuf, "[S], L L, %.2f, R L, %.2f, Df, %.2f, S A, %.2f, T G, %.2f, E P, %.2f, E S, %.2f, "
											, st_ControlLog.ControlSensor.Sensor.Array[i].LeftLineSensor
											, st_ControlLog.ControlSensor.Sensor.Array[i].RightLineSensor
											, st_ControlLog.ControlSensor.Sensor.Array[i].DiffLineSensor
											, st_ControlLog.ControlSensor.Sensor.Array[i].SensorAngle
											, st_ControlLog.ControlSensor.Sensor.Array[i].Target
											, st_ControlLog.ControlSensor.Sensor.Array[i].ErrorP
											, st_ControlLog.ControlSensor.Sensor.Array[i].ErrorSum);
		SCF_WriteData(st_SendBuf, st_BufSize);

		st_BufSize = sprintf(st_SendBuf, "[S D], D, %.2f \r\n"
											, st_ControlLog.ControlSensor.MotorArray[i].SensorDuty);
		SCF_WriteData(st_SendBuf, st_BufSize);
	}
}


void LOG_PrintCourceRecord(void)
{

}
