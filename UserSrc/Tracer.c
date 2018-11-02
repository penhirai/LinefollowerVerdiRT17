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
#include "Switch.h"
#include "ControlVelocity.h"
#include "ControlAngularVelocity.h"
#include "Buzzer.h"
#include "DriveAssert.h"
#include "SciFifo.h"
#include <stdio.h>

#define SEND_BUF_SIZE 50

static SWT_EnmDecision st_Decision;
static uint8_t st_SendBuf[SEND_BUF_SIZE];
static uint8_t st_BufSize;
static SSR_StrSensorData st_SensorData;


void TRC_Init(void)
{

}

void TRC_StartSearchMode(void)
{
	float32_t vecTarget;
	float32_t velocity;
	float32_t vecError;
	float32_t angTarget;
	float32_t angular;
	float32_t angError;
	float32_t vecTargetTemp;
	float32_t theta;
	int32_t vecFlag = 0;
	int32_t threshold = 5;

	SSR_CalibSensor();

	for(volatile int32_t i = 0; i < 1000000; ++i){
		st_Decision = SWT_GetCenterDecision();
	}

	CSA_StartSensorMotor();
	CSA_StartSensorTask();

	while(1)
	{
		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			for(volatile int32_t i = 0; i < 10000000; ++i)	;

			for(int32_t i = 0; i < 3; ++i)
			{
				BZR_SetBeepCount(1);
				for(volatile int32_t i = 0; i < 10000000; ++i)	;
			}

			CVL_Init();
			CAV_Init();
			TSK_Start(TSK_TASK3_CONTROL_VELOCITY);
			TSK_Start(TSK_TASK4_CONTROL_ANGULAR);

			TSK_Start(TSK_TASKEACH1_ASSERT);
			DAS_Init();

			TSK_Start(TSK_TASK5_Judge_MARKER);

			CVL_StartDriveMotor();
			CAV_StartDriveMotor();
			vecTargetTemp = 1.0;
			CVL_SetTarget(vecTargetTemp);

			break;
		}
	}

	while(1)
	{
		SSR_StrMarkerData markerData;
		int32_t markerCount;
		float32_t leftDuty;
		float32_t rightDuty;
		SSR_EnmMarkerKind kind;
		SSR_EnmMarkerState leftMarker, rightMarker;

		vecTarget = CVL_GetTarget();
		velocity = CVL_GetVelocity();
		vecError = CVL_GetErrorNow();
		angTarget = CAV_GetTarget();
		angular = CAV_GetVelocity();
		angError = CAV_GetErrorNow();
		leftDuty  = FTR_GetLeftMotorDuty();
		rightDuty = FTR_GetRightMotorDuty();
		theta = CAV_GetVirtualThetaDeg();

		st_BufSize = sprintf(st_SendBuf, "theta:%.2f, vt:%.2f, v:%.2f \r\n", theta, vecTarget, velocity);
		SCF_WriteData(st_SendBuf, st_BufSize);

//		st_BufSize = sprintf(st_SendBuf, "vt:%.2f, v:%.2f, ve:%.2f   ",vecTarget, velocity, vecError);
//		SCF_WriteData(st_SendBuf, st_BufSize);

//		st_BufSize = sprintf(st_SendBuf, "at:%.2f, ang:%.2f, ae:%.2f  ", angTarget, angular, angError);
//		SCF_WriteData(st_SendBuf, st_BufSize);
//
//		st_BufSize = sprintf(st_SendBuf, "lD:%.2f, rD:%.2f \r\n", leftDuty, rightDuty);
//		SCF_WriteData(st_SendBuf, st_BufSize);

		theta = CAV_GetVirtualThetaDeg();
		if(theta < 0.0)
		{
			theta *= -1.0;
		}
		if((theta < threshold) && (vecFlag == 0))
		{
			CVL_SetTarget(1.1 * vecTargetTemp);
			vecFlag = 1;
		}
		if((theta >= threshold) && (vecFlag == 1))
		{
			CVL_SetTarget(vecTargetTemp);
			vecFlag = 0;
		}

		st_SensorData = SSR_GetSensorData();

//		st_BufSize = sprintf(st_SendBuf, "lm:%d, lc:%d, rc:%d, rm:%d  ", st_SensorData.LeftMarker, st_SensorData.LeftCenter, st_SensorData.RightCenter, st_SensorData.RightMarker);
//		SCF_WriteData(st_SendBuf, st_BufSize);

		kind = SSR_LEFT_MARKER;
		leftMarker = SSR_GetMarkerState(kind);
		kind = SSR_RIGHT_MARKER;
		rightMarker = SSR_GetMarkerState(kind);

		markerCount = 0;
		markerData = SSR_GetCourceMarker();

//		st_BufSize = sprintf(st_SendBuf, "l:%d, r:%d, marker:%d, dis:%.3f \r\n", leftMarker, rightMarker, markerData.MarkerKind, markerData.Distance);
//		SCF_WriteData(st_SendBuf, st_BufSize);

		if(markerData.MarkerKind == SSR_COURCE_MARKER_RIGHT)
		{
			++markerCount;
			BZR_SetBeepCount(3);
			while(1)
			{
				vecTarget = CVL_GetTarget();
				velocity = CVL_GetVelocity();
				st_BufSize = sprintf(st_SendBuf, "theta:%.2f, vt:%.2f, v:%.2f \r\n", theta, vecTarget, velocity);
				SCF_WriteData(st_SendBuf, st_BufSize);


				theta = CAV_GetVirtualThetaDeg();
				if(theta < 0.0)
				{
					theta *= -1.0;
				}
				if((theta < threshold) && (vecFlag == 0))
				{
					CVL_SetTarget(1.1 * vecTargetTemp);
					vecFlag = 1;
				}
				if((theta >= threshold) && (vecFlag == 1))
				{
					CVL_SetTarget(vecTargetTemp);
					vecFlag = 0;
				}

				/*
				vecTarget = CVL_GetTarget();
				velocity = CVL_GetVelocity();
				vecError = CVL_GetErrorNow();
				angTarget = CAV_GetTarget();
				angular = CAV_GetVelocity();
				angError = CAV_GetErrorNow();
				leftDuty  = FTR_GetLeftMotorDuty();
				rightDuty = FTR_GetRightMotorDuty();

				st_BufSize = sprintf(st_SendBuf, "at:%.2f, ang:%.2f, ae:%.2f  ", angTarget, angular, angError);
				SCF_WriteData(st_SendBuf, st_BufSize);

				st_BufSize = sprintf(st_SendBuf, "lD:%.2f, rD:%.2f \r\n", leftDuty, rightDuty);
				SCF_WriteData(st_SendBuf, st_BufSize);
				*/

				markerData = SSR_GetCourceMarker();
				if(markerData.MarkerKind == SSR_COURCE_MARKER_RIGHT)
				{
					++markerCount;
					BZR_SetBeepCount(3);

					CVL_SetTarget(0.0);

					for(volatile int32_t i = 0; i < 10000000; ++i)	;

					CVL_StopDriveMotor();

					for(volatile int32_t i = 0; i < 10000000; ++i)	;

					CSA_StopSensorMotor();
				}
			}
		}
	}
}


void TRC_StartDriveMode(void)
{
	float32_t vecTarget;
		float32_t velocity;
		float32_t vecError;
		float32_t angTarget;
		float32_t angular;
		float32_t angError;
		float32_t vecTargetTemp;
		float32_t theta;
		int32_t vecFlag = 0;
		int32_t threshold = 3;

		SSR_CalibSensor();

		for(volatile int32_t i = 0; i < 1000000; ++i){
			st_Decision = SWT_GetCenterDecision();
		}

		CSA_StartSensorMotor();
		CSA_StartSensorTask();

		while(1)
		{
			st_Decision = SWT_GetCenterDecision();
			if(st_Decision == SWT_DECISION_TRUE)
			{
				for(volatile int32_t i = 0; i < 10000000; ++i)	;

				for(int32_t i = 0; i < 3; ++i)
				{
					BZR_SetBeepCount(1);
					for(volatile int32_t i = 0; i < 10000000; ++i)	;
				}

				CVL_Init();
				CAV_Init();
				TSK_Start(TSK_TASK3_CONTROL_VELOCITY);
				TSK_Start(TSK_TASK4_CONTROL_ANGULAR);

				TSK_Start(TSK_TASKEACH1_ASSERT);
				DAS_Init();

				TSK_Start(TSK_TASK5_Judge_MARKER);

				CVL_StartDriveMotor();
				CAV_StartDriveMotor();
				vecTargetTemp = 1.1;
				CVL_SetTarget(vecTargetTemp);

				break;
			}
		}

		while(1)
		{
			SSR_StrMarkerData markerData;
			int32_t markerCount;
			float32_t leftDuty;
			float32_t rightDuty;
			SSR_EnmMarkerKind kind;
			SSR_EnmMarkerState leftMarker, rightMarker;

			vecTarget = CVL_GetTarget();
			velocity = CVL_GetVelocity();
			vecError = CVL_GetErrorNow();
			angTarget = CAV_GetTarget();
			angular = CAV_GetVelocity();
			angError = CAV_GetErrorNow();
			leftDuty  = FTR_GetLeftMotorDuty();
			rightDuty = FTR_GetRightMotorDuty();
			theta = CAV_GetVirtualThetaDeg();

//			st_BufSize = sprintf(st_SendBuf, "theta:%.2f, vt:%.2f \r\n", theta, vecTarget);
//			SCF_WriteData(st_SendBuf, st_BufSize);
	//		st_BufSize = sprintf(st_SendBuf, "vt:%.2f, v:%.2f, ve:%.2f   ",vecTarget, velocity, vecError);
	//		SCF_WriteData(st_SendBuf, st_BufSize);

	//		st_BufSize = sprintf(st_SendBuf, "at:%.2f, ang:%.2f, ae:%.2f  ", angTarget, angular, angError);
	//		SCF_WriteData(st_SendBuf, st_BufSize);
	//
	//		st_BufSize = sprintf(st_SendBuf, "lD:%.2f, rD:%.2f \r\n", leftDuty, rightDuty);
	//		SCF_WriteData(st_SendBuf, st_BufSize);

			theta = CAV_GetVirtualThetaDeg();
			if(theta < 0.0)
			{
				theta *= -1.0;
			}
			if((theta < threshold) && (vecFlag == 0))
			{
				CVL_SetTarget(1.1 * vecTargetTemp);
				vecFlag = 1;
			}
			if((theta >= threshold) && (vecFlag == 1))
			{
				CVL_SetTarget(vecTargetTemp);
				vecFlag = 0;
			}

			st_SensorData = SSR_GetSensorData();

	//		st_BufSize = sprintf(st_SendBuf, "lm:%d, lc:%d, rc:%d, rm:%d  ", st_SensorData.LeftMarker, st_SensorData.LeftCenter, st_SensorData.RightCenter, st_SensorData.RightMarker);
	//		SCF_WriteData(st_SendBuf, st_BufSize);

			kind = SSR_LEFT_MARKER;
			leftMarker = SSR_GetMarkerState(kind);
			kind = SSR_RIGHT_MARKER;
			rightMarker = SSR_GetMarkerState(kind);

			markerCount = 0;
			markerData = SSR_GetCourceMarker();

	//		st_BufSize = sprintf(st_SendBuf, "l:%d, r:%d, marker:%d, dis:%.3f \r\n", leftMarker, rightMarker, markerData.MarkerKind, markerData.Distance);
	//		SCF_WriteData(st_SendBuf, st_BufSize);

			if(markerData.MarkerKind == SSR_COURCE_MARKER_RIGHT)
			{
				++markerCount;
				BZR_SetBeepCount(3);
				while(1)
				{



					theta = CAV_GetVirtualThetaDeg();
					if(theta < 0.0)
					{
						theta *= -1.0;
					}
					if((theta < threshold) && (vecFlag == 0))
					{
						CVL_SetTarget(1.1 * vecTargetTemp);
						vecFlag = 1;
					}
					if((theta >= threshold) && (vecFlag == 1))
					{
						CVL_SetTarget(vecTargetTemp);
						vecFlag = 0;
					}

					/*
					vecTarget = CVL_GetTarget();
					velocity = CVL_GetVelocity();
					vecError = CVL_GetErrorNow();
					angTarget = CAV_GetTarget();
					angular = CAV_GetVelocity();
					angError = CAV_GetErrorNow();
					leftDuty  = FTR_GetLeftMotorDuty();
					rightDuty = FTR_GetRightMotorDuty();

					st_BufSize = sprintf(st_SendBuf, "at:%.2f, ang:%.2f, ae:%.2f  ", angTarget, angular, angError);
					SCF_WriteData(st_SendBuf, st_BufSize);

					st_BufSize = sprintf(st_SendBuf, "lD:%.2f, rD:%.2f \r\n", leftDuty, rightDuty);
					SCF_WriteData(st_SendBuf, st_BufSize);
					*/

					markerData = SSR_GetCourceMarker();
					if(markerData.MarkerKind == SSR_COURCE_MARKER_RIGHT)
					{
						++markerCount;
						BZR_SetBeepCount(3);

						CVL_SetTarget(0.0);

						for(volatile int32_t i = 0; i < 10000000; ++i)	;

						CVL_StopDriveMotor();

						for(volatile int32_t i = 0; i < 10000000; ++i)	;

						CSA_StopSensorMotor();
					}
				}
			}
		}
}

void TRC_StartDriveMode2(void)
{
	float32_t vecTarget;
	float32_t velocity;
	float32_t vecError;
	float32_t angTarget;
	float32_t angular;
	float32_t angError;
	float32_t vecTargetTemp;
	float32_t theta;
	int32_t vecFlag = 0;
	int32_t threshold = 3;

	SSR_CalibSensor();

	for(volatile int32_t i = 0; i < 1000000; ++i){
		st_Decision = SWT_GetCenterDecision();
	}

	CSA_StartSensorMotor();
	CSA_StartSensorTask();

	while(1)
	{
		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			for(volatile int32_t i = 0; i < 10000000; ++i)	;

			for(int32_t i = 0; i < 3; ++i)
			{
				BZR_SetBeepCount(1);
				for(volatile int32_t i = 0; i < 10000000; ++i)	;
			}

			CVL_Init();
			CAV_Init();
			TSK_Start(TSK_TASK3_CONTROL_VELOCITY);
			TSK_Start(TSK_TASK4_CONTROL_ANGULAR);

			TSK_Start(TSK_TASKEACH1_ASSERT);
			DAS_Init();

			TSK_Start(TSK_TASK5_Judge_MARKER);

			CVL_StartDriveMotor();
			CAV_StartDriveMotor();
			vecTargetTemp = 1.3;
			CVL_SetTarget(vecTargetTemp);

			break;
		}
	}

	while(1)
	{
		SSR_StrMarkerData markerData;
		int32_t markerCount;
		float32_t leftDuty;
		float32_t rightDuty;
		SSR_EnmMarkerKind kind;
		SSR_EnmMarkerState leftMarker, rightMarker;

		vecTarget = CVL_GetTarget();
		velocity = CVL_GetVelocity();
		vecError = CVL_GetErrorNow();
		angTarget = CAV_GetTarget();
		angular = CAV_GetVelocity();
		angError = CAV_GetErrorNow();
		leftDuty  = FTR_GetLeftMotorDuty();
		rightDuty = FTR_GetRightMotorDuty();
		theta = CAV_GetVirtualThetaDeg();

//		st_BufSize = sprintf(st_SendBuf, "theta:%.2f, vt:%.2f \r\n", theta, vecTarget);
//		SCF_WriteData(st_SendBuf, st_BufSize);
//		st_BufSize = sprintf(st_SendBuf, "vt:%.2f, v:%.2f, ve:%.2f   ",vecTarget, velocity, vecError);
//		SCF_WriteData(st_SendBuf, st_BufSize);

//		st_BufSize = sprintf(st_SendBuf, "at:%.2f, ang:%.2f, ae:%.2f  ", angTarget, angular, angError);
//		SCF_WriteData(st_SendBuf, st_BufSize);
//
//		st_BufSize = sprintf(st_SendBuf, "lD:%.2f, rD:%.2f \r\n", leftDuty, rightDuty);
//		SCF_WriteData(st_SendBuf, st_BufSize);

		theta = CAV_GetVirtualThetaDeg();
		if(theta < 0.0)
		{
			theta *= -1.0;
		}
		if((theta < threshold) && (vecFlag == 0))
		{
			CVL_SetTarget(1.0 * vecTargetTemp);
			vecFlag = 1;
		}
		if((theta >= threshold) && (vecFlag == 1))
		{
			CVL_SetTarget(vecTargetTemp);
			vecFlag = 0;
		}

		st_SensorData = SSR_GetSensorData();

//		st_BufSize = sprintf(st_SendBuf, "lm:%d, lc:%d, rc:%d, rm:%d  ", st_SensorData.LeftMarker, st_SensorData.LeftCenter, st_SensorData.RightCenter, st_SensorData.RightMarker);
//		SCF_WriteData(st_SendBuf, st_BufSize);

		kind = SSR_LEFT_MARKER;
		leftMarker = SSR_GetMarkerState(kind);
		kind = SSR_RIGHT_MARKER;
		rightMarker = SSR_GetMarkerState(kind);

		markerCount = 0;
		markerData = SSR_GetCourceMarker();

//		st_BufSize = sprintf(st_SendBuf, "l:%d, r:%d, marker:%d, dis:%.3f \r\n", leftMarker, rightMarker, markerData.MarkerKind, markerData.Distance);
//		SCF_WriteData(st_SendBuf, st_BufSize);

		if(markerData.MarkerKind == SSR_COURCE_MARKER_RIGHT)
		{
			++markerCount;
			BZR_SetBeepCount(3);
			while(1)
			{



				theta = CAV_GetVirtualThetaDeg();
				if(theta < 0.0)
				{
					theta *= -1.0;
				}
				if((theta < threshold) && (vecFlag == 0))
				{
					CVL_SetTarget(1.0 * vecTargetTemp);
					vecFlag = 1;
				}
				if((theta >= threshold) && (vecFlag == 1))
				{
					CVL_SetTarget(vecTargetTemp);
					vecFlag = 0;
				}

				/*
				vecTarget = CVL_GetTarget();
				velocity = CVL_GetVelocity();
				vecError = CVL_GetErrorNow();
				angTarget = CAV_GetTarget();
				angular = CAV_GetVelocity();
				angError = CAV_GetErrorNow();
				leftDuty  = FTR_GetLeftMotorDuty();
				rightDuty = FTR_GetRightMotorDuty();

				st_BufSize = sprintf(st_SendBuf, "at:%.2f, ang:%.2f, ae:%.2f  ", angTarget, angular, angError);
				SCF_WriteData(st_SendBuf, st_BufSize);

				st_BufSize = sprintf(st_SendBuf, "lD:%.2f, rD:%.2f \r\n", leftDuty, rightDuty);
				SCF_WriteData(st_SendBuf, st_BufSize);
				*/

				markerData = SSR_GetCourceMarker();
				if(markerData.MarkerKind == SSR_COURCE_MARKER_RIGHT)
				{
					++markerCount;
					BZR_SetBeepCount(3);

					CVL_SetTarget(0.0);

					for(volatile int32_t i = 0; i < 10000000; ++i)	;

					CVL_StopDriveMotor();

					for(volatile int32_t i = 0; i < 10000000; ++i)	;

					CSA_StopSensorMotor();
				}
			}
		}
	}
}
