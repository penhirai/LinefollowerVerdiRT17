/*
 * Log.h
 *
 *  Created on: 2018/06/21
 *      Author: shuuichi
 */

#ifndef LOG_H_
#define LOG_H_

#include "typedef.h"

#define CONTROL_LOG_MAX	1000
#define COURCE_LOG_MAX	1000

typedef struct strSensorData
{
	uint16_t LeftMarker;
	uint16_t LeftCenter;
	uint16_t RightCenter;
	uint16_t RightMarker;
	uint16_t Potentio;
	uint16_t Power;
	uint16_t Gyro;
}LOG_StrSensorData;

typedef struct strControlFactor
{
	float32_t ff;
	float32_t p;
	float32_t i;
	float32_t d;
}StrControlFactor;

typedef struct strControlUnit
{
	StrControlFactor Error;
	StrControlFactor Gain;
}StrControlUnit_t;

/*
typedef struct strTranslationControl
{
	StrControlUnit Unit;
}StrTranslationControl_t;

typedef struct strRotationControl
{
	StrControlUnit Unit;
}StrRotationControl_t;
*/

typedef struct strControlLog
{
//	StrTranslationControl_t	TranslationControl;
//	StrRotationControl_t	RotationControl;
	StrControlUnit_t TranslationControl;
	StrControlUnit_t RotationControl;
}StrControlLog_t;

typedef struct strControlLogArray
{
	StrControlLog_t ControlLog[CONTROL_LOG_MAX];
	uint32_t ArrayMax;
}StrControlLogArray;


typedef struct strCourceLog
{
	float32_t Distance;
}StrCourceLog_t;

typedef struct strCourceLogArray
{
	StrCourceLog_t CourceLog[COURCE_LOG_MAX];
	uint32_t ArrayMax;
}StrCourceLogArray_t;

#endif /* LOG_H_ */
