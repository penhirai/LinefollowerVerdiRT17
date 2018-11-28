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

#define SEND_BUF_SIZE 200

static uint8_t st_SendBuf[SEND_BUF_SIZE];
static uint8_t st_BufSize;

#define LOG_CONTROL_MAX	1500
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
		st_ControlLog.ControlSensor.Sensor.Array[i].Error.P         = 0.0;
		st_ControlLog.ControlSensor.Sensor.Array[i].Error.D         = 0.0;
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
		st_CourceLog.Array[i].Radius          = 0.0;
		st_CourceLog.Array[i].TargetVelocity  = 0.0;
		st_CourceLog.Array[i].s_n             = 0.0;
		st_CourceLog.Array[i].v_n             = 0.0;
		st_CourceLog.Array[i].a_n             = 0.0;
		st_CourceLog.Array[i].MarkerKind      = SSR_COURCE_MARKER_NON;
		//st_CourceLog.Array[i].IsChangeFlag    = LOG_CHANGE_FALSE;
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


void LOG_RecordCource(SSR_EnmCourceMarkerKind kind, float32_t distance)
{
	LOG_StrCourceLogArray array;
	uint32_t index;

	array.SensorAngle = CSA_GetSensorTheta();
	array.Velocity    = CVL_GetVelocity();
	array.Distance    = distance;
	array.AngularVelocity = CAV_GetVelocity();
	array.Radius      = 0.0;
	array.TargetVelocity = 0.0;
	array.s_n         = 0.0;
	array.v_n         = 0.0;
	array.a_n         = 0.0;
	array.MarkerKind  = kind;
	//array.IsChangeFlag = LOG_CHANGE_FALSE;
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
	// 並進系，回転系，センサ系 header
	st_BufSize = sprintf(st_SendBuf, "\r\n T FF Gain, T P Gain, T I Gain, T D Gain, T Scale Gain, A FF Gain, A P Gain, A I Gain, A D Gain, A Scale Gain, S P Gain, S D Gain \r\n");
	SCF_WriteData(st_SendBuf, st_BufSize);

	st_BufSize = sprintf(st_SendBuf, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f \r\n"
										, st_ControlLog.ControlVelocity.Transition.Header.Gain.Factor.FF
										, st_ControlLog.ControlVelocity.Transition.Header.Gain.Factor.P
										, st_ControlLog.ControlVelocity.Transition.Header.Gain.Factor.I
										, st_ControlLog.ControlVelocity.Transition.Header.Gain.Factor.D
										, st_ControlLog.ControlVelocity.Transition.Header.Gain.Scale
										, st_ControlLog.ControlVelocity.Angular.Header.Gain.Factor.FF
										, st_ControlLog.ControlVelocity.Angular.Header.Gain.Factor.P
										, st_ControlLog.ControlVelocity.Angular.Header.Gain.Factor.I
										, st_ControlLog.ControlVelocity.Angular.Header.Gain.Factor.D
										, st_ControlLog.ControlVelocity.Angular.Header.Gain.Scale
										, st_ControlLog.ControlSensor.Sensor.Header.Gain.Factor.P
										, st_ControlLog.ControlSensor.Sensor.Header.Gain.Factor.D);
	SCF_WriteData(st_SendBuf, st_BufSize);


	// title
	// 並進系
	st_BufSize = sprintf(st_SendBuf, "index, V EncoderVelocity[m/s], V EncoderDistance[m], V MemsVelocity[m/s], V MemsDistance[m], V TargetInstance[m/s], V Error FF, V Error P, V Error I, V Error D, V Error Sum, ");
	SCF_WriteData(st_SendBuf, st_BufSize);
	// 角速度系
	st_BufSize = sprintf(st_SendBuf, "A EncoderVelocity[deg/s], A EncoderDistance[deg], A MemsVelocity[deg/s], A MemsDistance[deg], A TargetInstance[deg/s], A Error FF, A Error P, A Error I, A Error D, A Error Sum, ");
	SCF_WriteData(st_SendBuf, st_BufSize);
	// 速度系Duty
	st_BufSize = sprintf(st_SendBuf, "D LeftDuty, D RightDuty, ");
	SCF_WriteData(st_SendBuf, st_BufSize);
	// センサ系
	st_BufSize = sprintf(st_SendBuf, "S LeftLineSensor, S RightLineSensor, S DiffSenser, S SensorAngle[deg], S Target[deg], S Error P, S Error D, S Error Sum, ");
	SCF_WriteData(st_SendBuf, st_BufSize);
	// センサ系Duty
	st_BufSize = sprintf(st_SendBuf, "S Duty \r\n");
	SCF_WriteData(st_SendBuf, st_BufSize);

	for(uint32_t i = 0; i < st_ControlLog.Index; ++i)
	{
		st_BufSize = sprintf(st_SendBuf, "%d, %.2f, %.4f, %.2f, %.4f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, "
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

		st_BufSize = sprintf(st_SendBuf, "%.2f, %.4f, %.2f, %.4f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, "
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

		st_BufSize = sprintf(st_SendBuf, "%.2f, %.2f, "
											, st_ControlLog.ControlVelocity.MotorArray[i].LeftDuty
											, st_ControlLog.ControlVelocity.MotorArray[i].RightDuty);
		SCF_WriteData(st_SendBuf, st_BufSize);

		st_BufSize = sprintf(st_SendBuf, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, "
											, st_ControlLog.ControlSensor.Sensor.Array[i].LeftLineSensor
											, st_ControlLog.ControlSensor.Sensor.Array[i].RightLineSensor
											, st_ControlLog.ControlSensor.Sensor.Array[i].DiffLineSensor
											, st_ControlLog.ControlSensor.Sensor.Array[i].SensorAngle
											, st_ControlLog.ControlSensor.Sensor.Array[i].Target
											, st_ControlLog.ControlSensor.Sensor.Array[i].Error.P
											, st_ControlLog.ControlSensor.Sensor.Array[i].Error.D
											, st_ControlLog.ControlSensor.Sensor.Array[i].ErrorSum);
		SCF_WriteData(st_SendBuf, st_BufSize);

		st_BufSize = sprintf(st_SendBuf, "%.2f \r\n"
											, st_ControlLog.ControlSensor.MotorArray[i].SensorDuty);
		SCF_WriteData(st_SendBuf, st_BufSize);
	}
}


void LOG_PrintCourceRecord(void)
{
	st_BufSize = sprintf(st_SendBuf, "\r\n index, SensorAngle, Velocity, AngularVelocity, Distance, Radius, Target Velocity, s_n, v_n, a_n, MarkerKind \r\n");
	SCF_WriteData(st_SendBuf, st_BufSize);

	for(uint32_t i = 0; i < st_CourceLog.Index; ++i)
	{
		st_BufSize = sprintf(st_SendBuf, "%d, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %d \r\n"
											, i
											, st_CourceLog.Array[i].SensorAngle
											, st_CourceLog.Array[i].Velocity
											, st_CourceLog.Array[i].AngularVelocity
											, st_CourceLog.Array[i].Distance
											, st_CourceLog.Array[i].Radius
											, st_CourceLog.Array[i].TargetVelocity
											, st_CourceLog.Array[i].s_n
											, st_CourceLog.Array[i].v_n
											, st_CourceLog.Array[i].a_n
											, st_CourceLog.Array[i].MarkerKind);
//											, st_CourceLog.Array[i].IsChangeFlag);
		SCF_WriteData(st_SendBuf, st_BufSize);
	}
}


uint32_t LOG_GetCourceRecordIndex(void)
{
	return st_CourceLog.Index;
}

LOG_StrCourceLogArray *LOG_GetCourceRecord(void)
{
	return st_CourceLog.Array;
}
