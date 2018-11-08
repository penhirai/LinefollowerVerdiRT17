/*
 * Log.h
 *
 *  Created on: 2018/06/21
 *      Author: shuuichi
 */

#ifndef LOG_H_
#define LOG_H_

#include "typedef.h"
#include "ControlStructure.h"
#include "Sensor.h"


typedef struct strControlVelocityHeader
{
	CST_StrGain Gain;
}LOG_StrControlVelocityHeader;

typedef struct strControlVelocityArray
{
	float32_t EncoderVelocity;
	float32_t MemsVelocity;
	float32_t EncoderDistance;
	float32_t MemsDistance;
	float32_t TargetInstance;
	CST_StrFactor Error;
	float32_t ErrorSum;
}LOG_StrControlVelocityArray;

typedef struct strControlVelocityDutyArray
{
	float32_t LeftDuty;
	float32_t RightDuty;
}LOG_StrControlVelocityDutyArray;


typedef struct strControlSensorHeader
{
	CST_StrGain Gain;
}LOG_StrControlSensorHeader;

typedef struct strControlSensorArray
{
	float32_t LeftLineSensor;
	float32_t RightLineSensor;
	float32_t DiffLineSensor;
	float32_t SensorAngle;
	float32_t Target;
	float32_t ErrorP;
	float32_t ErrorSum;
}LOG_StrControlSensorArray;

typedef struct strControlSensorDutyArray
{
	float32_t SensorDuty;
}LOG_StrControlSensorDutyArray;


typedef enum enmIsChange
{
	LOG_CHANGE_FALSE = 0,
	LOG_CHANGE_TRUE
}LOG_EnmIsChange;

typedef struct strCourceLogArray
{
	float32_t SensorAngle;
	float32_t Velocity;
	float32_t AngularVelocity;
	float32_t Distance;
	SSR_EnmCourceMarkerKind MarkerKind;
	LOG_EnmIsChange IsChangeFlag;
}LOG_StrCourceLogArray;




void LOG_Init(void);
void LOG_InitControl(void);
void LOG_InitCource(void);


void LOG_RecordControl(void);
void LOG_RecordCource(SSR_EnmCourceMarkerKind kind);

void LOG_PrintControlRecord(void);
void LOG_PrintCourceRecord(void);

#endif /* LOG_H_ */
