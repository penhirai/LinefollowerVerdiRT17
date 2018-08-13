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
	uint16_t LeftMarker;
	uint16_t LeftCenter;
	uint16_t RightCenter;
	uint16_t RightMarker;
	uint16_t Potentio;
	uint16_t Power;
	uint16_t Gyro;
}SSR_StrSensorData;

typedef struct strSensorDataArray
{
	SSR_StrSensorData ArrayTemp[SSR_SENSOR_BUFF_SIZE];
	SSR_StrSensorData Result;
	uint16_t Index;
}SSR_StrSensorDataArray;

void SSR_Init(void);

void SSR_TaskStartSensorGate(void);
void SSR_TaskStopsensorGate(void);
void SSR_GetAnalogSensor(void);
void SSR_TaskStartReadGyro(void);

void SSR_TaskCalcSensor(void);

#endif /* SENSOR_H_ */
