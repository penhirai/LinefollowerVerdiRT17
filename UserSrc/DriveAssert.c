/*
 * DriveAssert.c
 *
 *  Created on: 2018/10/18
 *      Author: shuuichi
 */

#include "DriveAssert.h"
#include "Sensor.h"
#include "Buzzer.h"
#include "ControlVelocity.h"

#define LINE_SENSOR_SHRESHOLD	(500)
#define LINE_ASSERT_COUNT		(100)	// 100 ms 連続LINE OUT でAssert する

typedef struct strLineAssert
{
	DAS_EnmAssertFlag Flag;
	uint32_t Count;
}StrLineAssert;

static DAS_EnmAssertFlag st_AssertFlag;
static SSR_StrSensorData st_SensorData;
static StrLineAssert st_LineAssert;

static void st_LineSensorAssert(void);


void DAS_Init(void)
{
	st_AssertFlag = DAS_NOT_ASSERT;

	st_LineAssert.Flag = DAS_NOT_ASSERT;
	st_LineAssert.Count = 0;
}


void DAS_AssertTask(void)
{
	st_LineSensorAssert();

	if(st_AssertFlag != DAS_ASSERTED)
	{
		if(st_LineAssert.Flag == DAS_ASSERTED)
		{
			st_AssertFlag = DAS_ASSERTED;
			CVL_StopDriveMotor();
			BZR_SetBeepCount(3);
		}
	}
}


DAS_EnmAssertFlag DAS_GetAssertFlag(void)
{
	return st_AssertFlag;
}


void DAS_ClearAssertFlag(void)
{
	st_AssertFlag = DAS_NOT_ASSERT;
	st_LineAssert.Flag = DAS_NOT_ASSERT;
	//BZR_SetBeepCount(3);
}


static void st_LineSensorAssert(void)
{
	st_SensorData = SSR_GetSensorData();

	if(st_LineAssert.Flag != DAS_ASSERTED)
	{
		if((st_SensorData.LeftCenter < LINE_SENSOR_SHRESHOLD) && (st_SensorData.RightCenter < LINE_SENSOR_SHRESHOLD))
		{
			++st_LineAssert.Count;
//			st_LineAssert.Flag = DAS_ASSERTED;
		}
		else
		{
			st_LineAssert.Count = 0;
		}

		if(st_LineAssert.Count > LINE_ASSERT_COUNT)
		{
			st_LineAssert.Flag = DAS_ASSERTED;
		}
	}
}
