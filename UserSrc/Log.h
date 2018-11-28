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
	CST_StrFactor Error;
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
	float32_t Radius;
	float32_t TargetVelocity;
	float32_t s_n;
	float32_t v_n;
	float32_t a_n;
	SSR_EnmCourceMarkerKind MarkerKind;
	//LOG_EnmIsChange IsChangeFlag;
}LOG_StrCourceLogArray;




void LOG_Init(void);
void LOG_InitControl(void);
void LOG_InitCource(void);


void LOG_RecordControl(void);
void LOG_RecordCource(SSR_EnmCourceMarkerKind kind, float32_t distance);

void LOG_PrintControlRecord(void);
void LOG_PrintCourceRecord(void);

uint32_t LOG_GetCourceRecordIndex(void);
LOG_StrCourceLogArray *LOG_GetCourceRecord(void);

#endif /* LOG_H_ */
