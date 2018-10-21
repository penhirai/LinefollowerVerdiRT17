/*
 * Sensor.h
 *
 *  Created on: 2018/07/12
 *      Author: shuuichi
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "typedef.h"
//#include "Log.h"

#define SSR_SENSOR_BUFF_SIZE 10

typedef struct strSensorData
{
	int16_t LeftMarker;
	int16_t LeftCenter;
	int16_t RightCenter;
	int16_t RightMarker;
	int16_t Potentio;
	int16_t Power;
	int16_t Gyro;
}SSR_StrSensorData;

typedef enum enmMarkerState
{
	SSR_LOW_STATE = 0,
	SSR_HIGH_STATE
}SSR_EnmMarkerState;

typedef enum enmMarkerKind
{
	SSR_LEFT_MARKER = 0,
	SSR_RIGHT_MARKER
}SSR_EnmMarkerKind;

typedef struct StrMarkerSensorState
{
	SSR_EnmMarkerState Left;
	SSR_EnmMarkerState Right;
}SSR_StrMarkerState;

typedef struct strSensorDataArray
{
	SSR_StrSensorData ArrayTemp[SSR_SENSOR_BUFF_SIZE];
	SSR_StrSensorData Result;
	SSR_StrMarkerState MarkerState;
	float32_t SensorTheta;
	float32_t BodyOmega;
	uint16_t Index;
}SSR_StrSensorDataArray;

void SSR_Init(void);

void SSR_TaskStartSensorGate(void);
void SSR_TaskStopSensorGate(void);
void SSR_TaskGetAnalogSensor(void);
void SSR_TaskStartReadGyro(void);

void SSR_TaskCalcSensor(void);

void SSR_TaskCalcFilter(void);

void SSR_TaskJudgeMarkerSensor(void);

void SSR_CalibSensor(void);

//SSR_StrSensorData *SSR_GetSensorStructure(void);
SSR_StrSensorData SSR_GetSensorData(void);
//float32_t *SSR_GetPotentioData(void);
float32_t SSR_GetPotentioData(void);
void      SSR_SetPotentioData(void);
float32_t SSR_GetGyroData(void);
void      SSR_SetGyroData(void);

SSR_EnmMarkerState SSR_GetMarkerState(SSR_EnmMarkerKind kind);

void SSR_PrintAllSensor(void);

#endif /* SENSOR_H_ */
