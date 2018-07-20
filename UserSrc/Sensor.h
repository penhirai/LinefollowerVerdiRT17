/*
 * Sensor.h
 *
 *  Created on: 2018/07/12
 *      Author: shuuichi
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "typedef.h"
#include "Log.h"

void SSR_Init(void);

LOG_StrSensorData *SSR_GetSensor(void);
void SSR_TaskStartReadGyro(void);

LOG_StrSensorData *SSR_TaskCalcSensor(void);

#endif /* SENSOR_H_ */
