/*
 * Tracer.c
 *
 *  Created on: 2018/10/15
 *      Author: shuuichi
 */

#include "Tracer.h"
#include "Sensor.h"
#include "TaskTimer.h"
#include "FunctionTimer.h"
#include "ControlSensorAngle.h"


void TRC_Init(void)
{

}

void TRC_StartSearchMode(void)
{
	SSR_CalibSensor();

	CSA_StartSensorMotor();
	TSK_Start(TSK_TASK2_CONTROL_SENSOR);
	FTR_StartSensorMotorTimer();

	while(1)
	{

	}
}


void TRC_StartDriveMode(void)
{
	SSR_CalibSensor();

	CSA_StartSensorMotor();
	TSK_Start(TSK_TASK2_CONTROL_SENSOR);
	FTR_StartSensorMotorTimer();

	while(1)
	{

	}
}
