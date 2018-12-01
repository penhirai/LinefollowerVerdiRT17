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
#include <machine.h>
#include <stdio.h>
#include <math.h>
#include "Log.h"
#include "Setting.h"

#define SEND_BUF_SIZE 50
#define LOG_DISTANCE (0.01)	// 10mm 刻み

#define CROSS_MARKER_ARRAY_MAX	(200)		// 30cm 刻み60m 換算
#define RIGHT_MARKER_ARRAY_MAX	(10)		// 自宅コースが6m ，60m 換算

#define MODE_KIND_MIN 0

#define GOAL_COUNT (4)

typedef struct strCourceLogStoreParam
{
	float32_t StartDistance;
	float32_t TempDistance;
	float32_t DiffDistance;
	float32_t OffsetDistance;
	SSR_StrMarkerData MarkerData;
	SSR_StrMarkerData MarkerDataLatched;
}StrCourceLogStoreParam;

typedef struct strCourceLog
{
	uint32_t Index;
	uint32_t IndexMax;
	LOG_StrCourceLogArray *Pt_Array;
}StrCourceLog;

typedef struct strCourceAnalysisParam
{
	float32_t RadiusMax;			// [m]
	float32_t TargetVelocityMax;	// [m/s]
	float32_t CentrifugalMax;		// [m/ss]
}StrCourceAnalysisParam;

typedef struct strCourceMarkerArray
{
	uint32_t LogIndex;
	float32_t Distance;
}StrCourceMarkerArray;

typedef struct strCourceMarkerRecord
{
	uint32_t Index;
	uint32_t IndexMax;
	StrCourceMarkerArray *Pt_Array;
}StrCourceMarkerRecord;

typedef enum enmModeDrive
{
	CASE_DRIVE_0 = MODE_KIND_MIN,
	CASE_DRIVE_1,
	CASE_DRIVE_2,
	CASE_DRIVE_3,
	MODE_KIND_MAX
}EnmModeDrive;

static SWT_EnmDecision st_Decision;
static uint8_t st_SendBuf[SEND_BUF_SIZE];
static uint8_t st_BufSize;
static SSR_StrSensorData st_SensorData;
static StrCourceLogStoreParam st_CourceLogStoreParam;
static StrCourceLog st_CourceLog;
static StrCourceAnalysisParam st_CourceAnalysisParam;
static StrCourceMarkerArray st_CrossMarkerArray[CROSS_MARKER_ARRAY_MAX];
static StrCourceMarkerArray st_RightMarkerArray[RIGHT_MARKER_ARRAY_MAX];
static StrCourceMarkerRecord st_CrossMarkerRecord;
static StrCourceMarkerRecord st_RightMarkerRecord;

static EnmModeDrive st_ModeKind;
static SWT_StrSwitch *st_Swt;
static SWT_EnmDecision st_Decision;


static void st_SetAnalysisParam(StrCourceAnalysisParam param);
static void st_StartRecordCource(void);
static SSR_StrMarkerData st_GetCourceMarker(void);
static void st_PrepareCource(void);
static void st_AnalyzeCource(void);
static float32_t st_CalculateRadius(float32_t velocityBuff, float32_t angularBuff);
static void st_ExecuteDriveMode(void);
static float32_t st_CalculateVelocityFromSensorAngle(float32_t sensorAngle, StrCourceAnalysisParam param);


void TRC_Init(void)
{
	StrCourceAnalysisParam analysisParam;

	//
	st_CourceLogStoreParam.OffsetDistance = 0.0;

	analysisParam.RadiusMax = 100.0;
	analysisParam.TargetVelocityMax = 2.0;
	analysisParam.CentrifugalMax = 30.0;
	st_SetAnalysisParam(analysisParam);
//	st_CourceAnalysisParam.RadiusMax = 100.0;
//	st_CourceAnalysisParam.TargetVelocityMax = 2.0;
//	st_CourceAnalysisParam.CentrifugalMax = 40.0;

	// Cross Marker Array ポインタの登録，初期化
	st_CrossMarkerRecord.Index = 0;
	st_CrossMarkerRecord.IndexMax = 0;
	st_CrossMarkerRecord.Pt_Array = st_CrossMarkerArray;
	for(uint32_t i = 0; i < CROSS_MARKER_ARRAY_MAX; ++i)
	{
		st_CrossMarkerRecord.Pt_Array[i].LogIndex = 0;
		st_CrossMarkerRecord.Pt_Array[i].Distance = 0.0;
	}

	// Right Marker Array ポインタの登録，初期化
	st_RightMarkerRecord.Index = 0;
	st_RightMarkerRecord.IndexMax = 0;
	st_RightMarkerRecord.Pt_Array = st_RightMarkerArray;
	for(uint32_t i = 0; i < RIGHT_MARKER_ARRAY_MAX; ++i)
	{
		st_RightMarkerRecord.Pt_Array[i].LogIndex = 0;
		st_RightMarkerRecord.Pt_Array[i].Distance = 0.0;
	}
}


//void TRC_StartSearchMode(void)
void TRC_StartSearchMode(float32_t velocity)
{
	float32_t vecTarget;
//	float32_t velocity;
	float32_t vecError;
	float32_t angTarget;
	float32_t angular;
	float32_t angError;
	float32_t vecTargetTemp;
	float32_t theta;
	int32_t vecFlag = 0;
	int32_t threshold = 8;
	int32_t markerCount;

	LOG_Init();

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

			CVL_SetTarget(0.1);
			CVL_Init();
			CAV_Init();
			TSK_Start(TSK_TASK3_CONTROL_VELOCITY);
			TSK_Start(TSK_TASK4_CONTROL_ANGULAR);

			TSK_Start(TSK_TASKEACH1_ASSERT);
			DAS_Init();

			TSK_Start(TSK_TASK5_Judge_MARKER);

			st_StartRecordCource();
			TSK_Start(TSK_TASK6_RECORD_COURSE);
			TSK_Start(TSK_TASK9_RECORD_CONTROL);

			CAV_Init();
			CVL_StartDriveMotor();
			CAV_StartDriveMotor();

			vecTargetTemp = velocity;
			CVL_SetTarget(vecTargetTemp);

			break;
		}
	}

	markerCount = 0;

	while(1)
	{
		SSR_StrMarkerData markerData;
		float32_t leftDuty;
		float32_t rightDuty;
		SSR_EnmMarkerKind kind;
		SSR_EnmMarkerState leftMarker, rightMarker;

//		vecTarget = CVL_GetTarget();
//		velocity = CVL_GetVelocity();
//		vecError = CVL_GetErrorNow();
//		angTarget = CAV_GetTarget();
//		angular = CAV_GetVelocity();
//		angError = CAV_GetErrorNow();
//		leftDuty  = FTR_GetLeftMotorDuty();
//		rightDuty = FTR_GetRightMotorDuty();
//		theta = CAV_GetVirtualThetaDeg();

//		st_BufSize = sprintf(st_SendBuf, "theta:%.2f, vt:%.2f, v:%.2f \r\n", theta, vecTarget, velocity);
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
			CVL_SetTarget(1.2 * vecTargetTemp);
			vecFlag = 1;
		}
		if((theta >= threshold) && (vecFlag == 1))
		{
			CVL_SetTarget(vecTargetTemp);
			vecFlag = 0;
		}

//		st_SensorData = SSR_GetSensorData();

//		st_BufSize = sprintf(st_SendBuf, "lm:%d, lc:%d, rc:%d, rm:%d  ", st_SensorData.LeftMarker, st_SensorData.LeftCenter, st_SensorData.RightCenter, st_SensorData.RightMarker);
//		SCF_WriteData(st_SendBuf, st_BufSize);

//		kind = SSR_LEFT_MARKER;
//		leftMarker = SSR_GetMarkerState(kind);
//		kind = SSR_RIGHT_MARKER;
//		rightMarker = SSR_GetMarkerState(kind);

//		st_BufSize = sprintf(st_SendBuf, "l:%d, r:%d, marker:%d, dis:%.3f \r\n", leftMarker, rightMarker, markerData.MarkerKind, markerData.Distance);
//		SCF_WriteData(st_SendBuf, st_BufSize);


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

		//markerData = SSR_GetCourceMarker();
		markerData = st_GetCourceMarker();

		if(markerData.MarkerKind == SSR_COURCE_MARKER_CROSS)
		{
			BZR_SetBeepCount(2);
		}

		if(markerData.MarkerKind == SSR_COURCE_MARKER_RIGHT)
		{
			++markerCount;
			BZR_SetBeepCount(3);
		}

		if(markerCount >= GOAL_COUNT)
		{
			CVL_SetTarget(0.0);

			for(volatile int32_t i = 0; i < 10000000; ++i)	;

			CVL_StopDriveMotor();

			TSK_Stop(TSK_TASK9_RECORD_CONTROL);
			TSK_Stop(TSK_TASK6_RECORD_COURSE);
			TSK_Stop(TSK_TASKEACH1_ASSERT);

			BZR_SetBeepCount(3);

			for(volatile int32_t i = 0; i < 10000000; ++i)	;

			CSA_StopSensorMotor();

			for(volatile int32_t i = 0; i < 10000000; ++i)	;

			TSK_Stop(TSK_TASK2_CONTROL_SENSOR);
			TSK_Stop(TSK_TASK3_CONTROL_VELOCITY);
			TSK_Stop(TSK_TASK4_CONTROL_ANGULAR);

			st_BufSize = sprintf(st_SendBuf, "\r\n start log analysis \r\n");
			SCF_WriteData(st_SendBuf, st_BufSize);

			// コース解析
			st_PrepareCource();
			st_AnalyzeCource();

			// ログ出力
			//LOG_PrintCourceRecord();
			//LOG_PrintControlRecord();

			// ドライブモード実行
			TRC_DriveMode();
		}
	}
}


void TRC_DriveMode(void)
{
	StrCourceAnalysisParam analysisParam;
	st_ModeKind  = CASE_DRIVE_0;


	SWT_Init(MODE_KIND_MIN, MODE_KIND_MAX, SWT_UD_MIN, SWT_UD_MAX);
	st_Swt = SWT_GetSwitch();

	BZR_SetBeepCount(3);
	//st_AnalyzeCource();

	while(1)
	{
		st_ModeKind = (EnmModeDrive)st_Swt->RL_Dif;
		st_Decision = SWT_GetCenterDecision();

		LED_binaryOn(st_ModeKind);
		switch(st_ModeKind)
		{
			case CASE_DRIVE_0:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					analysisParam.RadiusMax = 100.0;
					analysisParam.TargetVelocityMax = 2.0;
					analysisParam.CentrifugalMax = 30.0;
					st_SetAnalysisParam(analysisParam);
					st_AnalyzeCource();
					st_ExecuteDriveMode();
					BZR_SetBeepCount(5);
				}
				break;
			case CASE_DRIVE_1:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					analysisParam.RadiusMax = 100.0;
					analysisParam.TargetVelocityMax = 2.2;
					analysisParam.CentrifugalMax = 30.0;
					st_SetAnalysisParam(analysisParam);
					st_AnalyzeCource();
					st_ExecuteDriveMode();
					BZR_SetBeepCount(5);
				}
				break;
			case CASE_DRIVE_2:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					analysisParam.RadiusMax = 100.0;
					analysisParam.TargetVelocityMax = 2.5;
					analysisParam.CentrifugalMax = 30.0;
					st_SetAnalysisParam(analysisParam);
					st_AnalyzeCource();
					st_ExecuteDriveMode();
					BZR_SetBeepCount(5);
				}
				break;
			case CASE_DRIVE_3:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					analysisParam.RadiusMax = 100.0;
					analysisParam.TargetVelocityMax = 3.0;
					analysisParam.CentrifugalMax = 35.0;
					st_SetAnalysisParam(analysisParam);
					st_AnalyzeCource();
					st_ExecuteDriveMode();
					BZR_SetBeepCount(5);
				}
				break;
			default:
				break;
		}
	}
}

//
//void TRC_StartDriveMode(void)
//{
//	float32_t vecTarget;
//		float32_t velocity;
//		float32_t vecError;
//		float32_t angTarget;
//		float32_t angular;
//		float32_t angError;
//		float32_t vecTargetTemp;
//		float32_t theta;
//		int32_t vecFlag = 0;
//		int32_t threshold = 3;
//
//		SSR_CalibSensor();
//
//		for(volatile int32_t i = 0; i < 1000000; ++i){
//			st_Decision = SWT_GetCenterDecision();
//		}
//
//		CSA_StartSensorMotor();
//		CSA_StartSensorTask();
//
//		while(1)
//		{
//			st_Decision = SWT_GetCenterDecision();
//			if(st_Decision == SWT_DECISION_TRUE)
//			{
//				for(volatile int32_t i = 0; i < 10000000; ++i)	;
//
//				for(int32_t i = 0; i < 3; ++i)
//				{
//					BZR_SetBeepCount(1);
//					for(volatile int32_t i = 0; i < 10000000; ++i)	;
//				}
//
//				CVL_Init();
//				CAV_Init();
//				TSK_Start(TSK_TASK3_CONTROL_VELOCITY);
//				TSK_Start(TSK_TASK4_CONTROL_ANGULAR);
//
//				TSK_Start(TSK_TASKEACH1_ASSERT);
//				DAS_Init();
//
//				TSK_Start(TSK_TASK5_Judge_MARKER);
//
//				CVL_StartDriveMotor();
//				CAV_StartDriveMotor();
//				vecTargetTemp = 1.5;
//				CVL_SetTarget(vecTargetTemp);
//
//				break;
//			}
//		}
//
//		while(1)
//		{
//			SSR_StrMarkerData markerData;
//			int32_t markerCount;
//			float32_t leftDuty;
//			float32_t rightDuty;
//			SSR_EnmMarkerKind kind;
//			SSR_EnmMarkerState leftMarker, rightMarker;
//
//			vecTarget = CVL_GetTarget();
//			velocity = CVL_GetVelocity();
//			vecError = CVL_GetErrorNow();
//			angTarget = CAV_GetTarget();
//			angular = CAV_GetVelocity();
//			angError = CAV_GetErrorNow();
//			leftDuty  = FTR_GetLeftMotorDuty();
//			rightDuty = FTR_GetRightMotorDuty();
//			theta = CAV_GetVirtualThetaDeg();
//
////			st_BufSize = sprintf(st_SendBuf, "theta:%.2f, vt:%.2f \r\n", theta, vecTarget);
////			SCF_WriteData(st_SendBuf, st_BufSize);
//	//		st_BufSize = sprintf(st_SendBuf, "vt:%.2f, v:%.2f, ve:%.2f   ",vecTarget, velocity, vecError);
//	//		SCF_WriteData(st_SendBuf, st_BufSize);
//
//	//		st_BufSize = sprintf(st_SendBuf, "at:%.2f, ang:%.2f, ae:%.2f  ", angTarget, angular, angError);
//	//		SCF_WriteData(st_SendBuf, st_BufSize);
//	//
//	//		st_BufSize = sprintf(st_SendBuf, "lD:%.2f, rD:%.2f \r\n", leftDuty, rightDuty);
//	//		SCF_WriteData(st_SendBuf, st_BufSize);
//
//			theta = CAV_GetVirtualThetaDeg();
//			if(theta < 0.0)
//			{
//				theta *= -1.0;
//			}
//			if((theta < threshold) && (vecFlag == 0))
//			{
//				CVL_SetTarget(1.1 * vecTargetTemp);
//				vecFlag = 1;
//			}
//			if((theta >= threshold) && (vecFlag == 1))
//			{
//				CVL_SetTarget(vecTargetTemp);
//				vecFlag = 0;
//			}
//
//			st_SensorData = SSR_GetSensorData();
//
//	//		st_BufSize = sprintf(st_SendBuf, "lm:%d, lc:%d, rc:%d, rm:%d  ", st_SensorData.LeftMarker, st_SensorData.LeftCenter, st_SensorData.RightCenter, st_SensorData.RightMarker);
//	//		SCF_WriteData(st_SendBuf, st_BufSize);
//
//			kind = SSR_LEFT_MARKER;
//			leftMarker = SSR_GetMarkerState(kind);
//			kind = SSR_RIGHT_MARKER;
//			rightMarker = SSR_GetMarkerState(kind);
//
//			markerCount = 0;
//			markerData = SSR_GetCourceMarker();
//
//	//		st_BufSize = sprintf(st_SendBuf, "l:%d, r:%d, marker:%d, dis:%.3f \r\n", leftMarker, rightMarker, markerData.MarkerKind, markerData.Distance);
//	//		SCF_WriteData(st_SendBuf, st_BufSize);
//
//			if(markerData.MarkerKind == SSR_COURCE_MARKER_RIGHT)
//			{
//				++markerCount;
//				BZR_SetBeepCount(3);
//				while(1)
//				{
//
//
//
//					theta = CAV_GetVirtualThetaDeg();
//					if(theta < 0.0)
//					{
//						theta *= -1.0;
//					}
//					if((theta < threshold) && (vecFlag == 0))
//					{
//						CVL_SetTarget(1.1 * vecTargetTemp);
//						vecFlag = 1;
//					}
//					if((theta >= threshold) && (vecFlag == 1))
//					{
//						CVL_SetTarget(vecTargetTemp);
//						vecFlag = 0;
//					}
//
//					/*
//					vecTarget = CVL_GetTarget();
//					velocity = CVL_GetVelocity();
//					vecError = CVL_GetErrorNow();
//					angTarget = CAV_GetTarget();
//					angular = CAV_GetVelocity();
//					angError = CAV_GetErrorNow();
//					leftDuty  = FTR_GetLeftMotorDuty();
//					rightDuty = FTR_GetRightMotorDuty();
//
//					st_BufSize = sprintf(st_SendBuf, "at:%.2f, ang:%.2f, ae:%.2f  ", angTarget, angular, angError);
//					SCF_WriteData(st_SendBuf, st_BufSize);
//
//					st_BufSize = sprintf(st_SendBuf, "lD:%.2f, rD:%.2f \r\n", leftDuty, rightDuty);
//					SCF_WriteData(st_SendBuf, st_BufSize);
//					*/
//
//					markerData = SSR_GetCourceMarker();
//					if(markerData.MarkerKind == SSR_COURCE_MARKER_RIGHT)
//					{
//						++markerCount;
//						BZR_SetBeepCount(3);
//
//						CVL_SetTarget(0.0);
//
//						for(volatile int32_t i = 0; i < 10000000; ++i)	;
//
//						CVL_StopDriveMotor();
//
//						for(volatile int32_t i = 0; i < 10000000; ++i)	;
//
//						CSA_StopSensorMotor();
//					}
//				}
//			}
//		}
//}
//
//void TRC_StartDriveMode2(void)
//{
//	float32_t vecTarget;
//	float32_t velocity;
//	float32_t vecError;
//	float32_t angTarget;
//	float32_t angular;
//	float32_t angError;
//	float32_t vecTargetTemp;
//	float32_t theta;
//	int32_t vecFlag = 0;
//	int32_t threshold = 3;
//
//	SSR_CalibSensor();
//
//	for(volatile int32_t i = 0; i < 1000000; ++i){
//		st_Decision = SWT_GetCenterDecision();
//	}
//
//	CSA_StartSensorMotor();
//	CSA_StartSensorTask();
//
//	while(1)
//	{
//		st_Decision = SWT_GetCenterDecision();
//		if(st_Decision == SWT_DECISION_TRUE)
//		{
//			for(volatile int32_t i = 0; i < 10000000; ++i)	;
//
//			for(int32_t i = 0; i < 3; ++i)
//			{
//				BZR_SetBeepCount(1);
//				for(volatile int32_t i = 0; i < 10000000; ++i)	;
//			}
//
//			CVL_Init();
//			CAV_Init();
//			TSK_Start(TSK_TASK3_CONTROL_VELOCITY);
//			TSK_Start(TSK_TASK4_CONTROL_ANGULAR);
//
//			TSK_Start(TSK_TASKEACH1_ASSERT);
//			DAS_Init();
//
//			TSK_Start(TSK_TASK5_Judge_MARKER);
//
//			CVL_StartDriveMotor();
//			CAV_StartDriveMotor();
//			vecTargetTemp = 1.6;
//			CVL_SetTarget(vecTargetTemp);
//
//			break;
//		}
//	}
//
//	while(1)
//	{
//		SSR_StrMarkerData markerData;
//		int32_t markerCount;
//		float32_t leftDuty;
//		float32_t rightDuty;
//		SSR_EnmMarkerKind kind;
//		SSR_EnmMarkerState leftMarker, rightMarker;
//
//		vecTarget = CVL_GetTarget();
//		velocity = CVL_GetVelocity();
//		vecError = CVL_GetErrorNow();
//		angTarget = CAV_GetTarget();
//		angular = CAV_GetVelocity();
//		angError = CAV_GetErrorNow();
//		leftDuty  = FTR_GetLeftMotorDuty();
//		rightDuty = FTR_GetRightMotorDuty();
//		theta = CAV_GetVirtualThetaDeg();
//
////		st_BufSize = sprintf(st_SendBuf, "theta:%.2f, vt:%.2f \r\n", theta, vecTarget);
////		SCF_WriteData(st_SendBuf, st_BufSize);
////		st_BufSize = sprintf(st_SendBuf, "vt:%.2f, v:%.2f, ve:%.2f   ",vecTarget, velocity, vecError);
////		SCF_WriteData(st_SendBuf, st_BufSize);
//
////		st_BufSize = sprintf(st_SendBuf, "at:%.2f, ang:%.2f, ae:%.2f  ", angTarget, angular, angError);
////		SCF_WriteData(st_SendBuf, st_BufSize);
////
////		st_BufSize = sprintf(st_SendBuf, "lD:%.2f, rD:%.2f \r\n", leftDuty, rightDuty);
////		SCF_WriteData(st_SendBuf, st_BufSize);
//
//		theta = CAV_GetVirtualThetaDeg();
//		if(theta < 0.0)
//		{
//			theta *= -1.0;
//		}
//		if((theta < threshold) && (vecFlag == 0))
//		{
//			CVL_SetTarget(1.0 * vecTargetTemp);
//			vecFlag = 1;
//		}
//		if((theta >= threshold) && (vecFlag == 1))
//		{
//			CVL_SetTarget(vecTargetTemp);
//			vecFlag = 0;
//		}
//
//		st_SensorData = SSR_GetSensorData();
//
////		st_BufSize = sprintf(st_SendBuf, "lm:%d, lc:%d, rc:%d, rm:%d  ", st_SensorData.LeftMarker, st_SensorData.LeftCenter, st_SensorData.RightCenter, st_SensorData.RightMarker);
////		SCF_WriteData(st_SendBuf, st_BufSize);
//
//		kind = SSR_LEFT_MARKER;
//		leftMarker = SSR_GetMarkerState(kind);
//		kind = SSR_RIGHT_MARKER;
//		rightMarker = SSR_GetMarkerState(kind);
//
//		markerCount = 0;
//		markerData = SSR_GetCourceMarker();
//
////		st_BufSize = sprintf(st_SendBuf, "l:%d, r:%d, marker:%d, dis:%.3f \r\n", leftMarker, rightMarker, markerData.MarkerKind, markerData.Distance);
////		SCF_WriteData(st_SendBuf, st_BufSize);
//
//		if(markerData.MarkerKind == SSR_COURCE_MARKER_RIGHT)
//		{
//			++markerCount;
//			BZR_SetBeepCount(3);
//			while(1)
//			{
//
//
//
//				theta = CAV_GetVirtualThetaDeg();
//				if(theta < 0.0)
//				{
//					theta *= -1.0;
//				}
//				if((theta < threshold) && (vecFlag == 0))
//				{
//					CVL_SetTarget(1.0 * vecTargetTemp);
//					vecFlag = 1;
//				}
//				if((theta >= threshold) && (vecFlag == 1))
//				{
//					CVL_SetTarget(vecTargetTemp);
//					vecFlag = 0;
//				}
//
//				/*
//				vecTarget = CVL_GetTarget();
//				velocity = CVL_GetVelocity();
//				vecError = CVL_GetErrorNow();
//				angTarget = CAV_GetTarget();
//				angular = CAV_GetVelocity();
//				angError = CAV_GetErrorNow();
//				leftDuty  = FTR_GetLeftMotorDuty();
//				rightDuty = FTR_GetRightMotorDuty();
//
//				st_BufSize = sprintf(st_SendBuf, "at:%.2f, ang:%.2f, ae:%.2f  ", angTarget, angular, angError);
//				SCF_WriteData(st_SendBuf, st_BufSize);
//
//				st_BufSize = sprintf(st_SendBuf, "lD:%.2f, rD:%.2f \r\n", leftDuty, rightDuty);
//				SCF_WriteData(st_SendBuf, st_BufSize);
//				*/
//
//				markerData = SSR_GetCourceMarker();
//				if(markerData.MarkerKind == SSR_COURCE_MARKER_RIGHT)
//				{
//					++markerCount;
//					BZR_SetBeepCount(3);
//
//					CVL_SetTarget(0.0);
//
//					for(volatile int32_t i = 0; i < 10000000; ++i)	;
//
//					CVL_StopDriveMotor();
//
//					for(volatile int32_t i = 0; i < 10000000; ++i)	;
//
//					CSA_StopSensorMotor();
//				}
//			}
//		}
//	}
//}


void TRC_RecordCourceTask(void)
{
	float32_t sensorAngle;

	// 10mm 刻み
	st_CourceLogStoreParam.TempDistance = CVL_GetDistance();
	st_CourceLogStoreParam.DiffDistance = st_CourceLogStoreParam.TempDistance - st_CourceLogStoreParam.StartDistance;
	if(st_CourceLogStoreParam.DiffDistance >= LOG_DISTANCE)
	{
		st_CourceLogStoreParam.StartDistance = st_CourceLogStoreParam.TempDistance;
		LOG_RecordCource(SSR_COURCE_MARKER_NON, st_CourceLogStoreParam.TempDistance);
	}

	// マーカーを検出したら
	st_CourceLogStoreParam.MarkerData = SSR_GetCourceMarker();
	if(st_CourceLogStoreParam.MarkerData.MarkerKind != SSR_COURCE_MARKER_NON)
	{
		sensorAngle = CSA_GetSensorTheta();
		if((-10.0 < sensorAngle) && (sensorAngle < 10.0))
		{
			LOG_RecordCource(st_CourceLogStoreParam.MarkerData.MarkerKind, st_CourceLogStoreParam.MarkerData.Distance);
			st_CourceLogStoreParam.MarkerDataLatched = st_CourceLogStoreParam.MarkerData;
		}
	}
}


void TRC_PlayCourceTask(void)
{
	float32_t velocity;
	int32_t indexDif;

	// 走行距離を計測
	st_CourceLogStoreParam.TempDistance = CVL_GetDistance();
	st_CourceLogStoreParam.DiffDistance = st_CourceLogStoreParam.TempDistance - st_CourceLogStoreParam.StartDistance + st_CourceLogStoreParam.OffsetDistance;
	if(st_CourceLogStoreParam.DiffDistance >= st_CourceLog.Pt_Array[st_CourceLog.Index].Distance)
	{
		velocity = st_CalculateVelocityFromSensorAngle(st_CourceLog.Pt_Array[st_CourceLog.Index].SensorAngle, st_CourceAnalysisParam);
		if(st_CourceLog.Pt_Array[st_CourceLog.Index].TargetVelocity >= velocity)
		{
			CVL_SetTarget(velocity);
		}
		else
		{
			CVL_SetTarget(st_CourceLog.Pt_Array[st_CourceLog.Index].TargetVelocity);
		}
		++st_CourceLog.Index;
		if(st_CourceLog.Index >= st_CourceLog.IndexMax)
		{
			st_CourceLog.Index = st_CourceLog.IndexMax - 1;
		}
	}

	// マーカーを検出
	st_CourceLogStoreParam.MarkerData = SSR_GetCourceMarker();
	switch(st_CourceLogStoreParam.MarkerData.MarkerKind)
	{
	case SSR_COURCE_MARKER_LEFT:
		break;
	case SSR_COURCE_MARKER_RIGHT:
		indexDif = (int32_t)st_CourceLog.Index - (int32_t)st_RightMarkerRecord.Pt_Array[st_RightMarkerRecord.Index].LogIndex;
		if((-50 < indexDif) & (indexDif < 50))
		{
			st_CourceLogStoreParam.MarkerDataLatched = st_CourceLogStoreParam.MarkerData;
			st_CourceLogStoreParam.OffsetDistance = st_RightMarkerRecord.Pt_Array[st_RightMarkerRecord.Index].Distance - st_CourceLogStoreParam.DiffDistance;
			st_CourceLog.Index = st_RightMarkerRecord.Pt_Array[st_RightMarkerRecord.Index].LogIndex;
			++st_RightMarkerRecord.Index;
			if(st_RightMarkerRecord.Index >= st_RightMarkerRecord.IndexMax)
			{
				st_RightMarkerRecord.Index = st_RightMarkerRecord.IndexMax - 1;
			}
		}
		break;
	case SSR_COURCE_MARKER_CROSS:
		indexDif = (int32_t)st_CourceLog.Index - (int32_t)st_CrossMarkerRecord.Pt_Array[st_CrossMarkerRecord.Index].LogIndex;
		if((-20 < indexDif) & (indexDif < 20))
		{
			st_CourceLogStoreParam.MarkerDataLatched = st_CourceLogStoreParam.MarkerData;
			st_CourceLogStoreParam.OffsetDistance = st_CrossMarkerRecord.Pt_Array[st_CrossMarkerRecord.Index].Distance - st_CourceLogStoreParam.DiffDistance;
			st_CourceLog.Index = st_CrossMarkerRecord.Pt_Array[st_CrossMarkerRecord.Index].LogIndex;
			++st_CrossMarkerRecord.Index;
			if(st_CrossMarkerRecord.Index >= st_CrossMarkerRecord.IndexMax)
			{
				st_CrossMarkerRecord.Index = st_CrossMarkerRecord.Index - 1;
			}
		}
		break;
	}
}


static void st_SetAnalysisParam(StrCourceAnalysisParam param)
{
	st_CourceAnalysisParam = param;
}


static void st_StartRecordCource(void)
{
	st_CourceLogStoreParam.StartDistance = CVL_GetDistance();
	st_CourceLogStoreParam.TempDistance = 0.0;
	st_CourceLogStoreParam.DiffDistance = 0.0;
	st_CourceLogStoreParam.OffsetDistance = 0.0;
	st_CourceLogStoreParam.MarkerData.MarkerKind = SSR_COURCE_MARKER_NON;
	st_CourceLogStoreParam.MarkerData.Distance   = 0.0;
	st_CourceLogStoreParam.MarkerDataLatched = st_CourceLogStoreParam.MarkerData;

	st_CourceLog.Index = 0;
	st_RightMarkerRecord.Index = 0;
	st_CrossMarkerRecord.Index = 0;
}


static SSR_StrMarkerData st_GetCourceMarker(void)
{
	SSR_StrMarkerData markerKind;

	if(st_CourceLogStoreParam.MarkerDataLatched.MarkerKind != SSR_COURCE_MARKER_NON)
	{
		markerKind = st_CourceLogStoreParam.MarkerDataLatched;

		st_CourceLogStoreParam.MarkerDataLatched.Distance = 0.0;
		st_CourceLogStoreParam.MarkerDataLatched.MarkerKind = SSR_COURCE_MARKER_NON;
	}
	else
	{
		markerKind.Distance = 0.0;
		markerKind.MarkerKind = SSR_COURCE_MARKER_NON;
	}

	return markerKind;
}

static void st_PrepareCource(void)
{
	SSR_EnmCourceMarkerKind markerKind;
	int32_t i;

	BZR_SetBeepCount(2);

	st_CourceLog.Index = LOG_GetCourceRecordIndex();
	st_CourceLog.Pt_Array = LOG_GetCourceRecord();

	st_BufSize = sprintf(st_SendBuf, "Index:%d, v[0]:%.2f \r\n", st_CourceLog.Index, st_CourceLog.Pt_Array[0].Velocity);
	SCF_WriteData(st_SendBuf, st_BufSize);

	BZR_SetBeepCount(2);

	// コースログindex 記録
	st_CourceLog.IndexMax = st_CourceLog.Index;

	// ラジアン変換
	for(i = 0; i < st_CourceLog.Index; ++i)
	{
		st_CourceLog.Pt_Array[i].AngularVelocity *= K_ANGULAR_VELOCITY;
	}

	// 距離初期化
	for(i = 0; i < st_CourceLog.Index; ++i)
	{
		st_CourceLog.Pt_Array[i].Distance -= st_CourceLog.Pt_Array[0].Distance;
	}

	// マーカー位置を記録
	st_RightMarkerRecord.Index = 0;
	st_CrossMarkerRecord.Index = 0;
	for(i = 0; i < st_CourceLog.Index; ++i)
	{
		markerKind = st_CourceLog.Pt_Array[i].MarkerKind;
		switch(markerKind)
		{
		case SSR_COURCE_MARKER_LEFT:
			break;
		case SSR_COURCE_MARKER_RIGHT:
			st_RightMarkerRecord.Pt_Array[st_RightMarkerRecord.Index].LogIndex = i;
			st_RightMarkerRecord.Pt_Array[st_RightMarkerRecord.Index].Distance = st_CourceLog.Pt_Array[i].Distance;
			++st_RightMarkerRecord.Index;
			break;
		case SSR_COURCE_MARKER_CROSS:
			st_CrossMarkerRecord.Pt_Array[st_CrossMarkerRecord.Index].LogIndex = i;
			st_CrossMarkerRecord.Pt_Array[st_CrossMarkerRecord.Index].Distance = st_CourceLog.Pt_Array[i].Distance;
			++st_CrossMarkerRecord.Index;
			break;
		}
	}
	st_RightMarkerRecord.IndexMax = st_RightMarkerRecord.Index;
	st_CrossMarkerRecord.IndexMax = st_CrossMarkerRecord.Index;

	/*
	// マーカー位置表示
	for(i = 0; i < st_RightMarkerRecord.Index; ++i)
	{
		st_BufSize = sprintf(st_SendBuf, "Right[%d], %d, %.4f \r\n", i, st_RightMarkerRecord.Pt_Array[i].LogIndex, st_RightMarkerRecord.Pt_Array[i].Distance);
		SCF_WriteData(st_SendBuf, st_BufSize);
	}

	for(i = 0; i < st_CrossMarkerRecord.Index; ++i)
	{
		st_BufSize = sprintf(st_SendBuf, "Cross[%d], %d, %.4f \r\n", i, st_CrossMarkerRecord.Pt_Array[i].LogIndex, st_CrossMarkerRecord.Pt_Array[i].Distance);
		SCF_WriteData(st_SendBuf, st_BufSize);
	}
	*/
}


static void st_AnalyzeCource(void)
{
	SSR_EnmCourceMarkerKind markerKind;
	int32_t averageNum = 5;
	int32_t averageOffset = 2;
	float32_t velocityBuff;
	float32_t angularBuff;
	float32_t radiusBuff;
	float32_t upAccelBuff;
	float32_t downAccelBuff;
	volatile float32_t s_BrakeCalc;
	volatile float32_t s_BrakeSum;
	int32_t i, j, k, n;

	st_CourceLog.Index = st_CourceLog.IndexMax;

	// 移動平均
	for(i = 0; i < averageOffset; ++i)
	{
		velocityBuff = 0.0;
	    angularBuff = 0.0;
	    for(j = 0; j < (averageNum - (averageOffset - i)); ++j)
	    {
	    	velocityBuff += st_CourceLog.Pt_Array[j].Velocity;
	        angularBuff  += st_CourceLog.Pt_Array[j].AngularVelocity;
	    }
		velocityBuff /= (float32_t)(averageNum - (averageOffset - i));
		angularBuff  /= (float32_t)(averageNum - (averageOffset - i));
		st_CourceLog.Pt_Array[i].Radius = st_CalculateRadius(velocityBuff, angularBuff);

		/*
		st_BufSize = sprintf(st_SendBuf, "Radius[%d]: %.2f \r\n", i, st_CourceLog.Pt_Array[i].Radius);
		SCF_WriteData(st_SendBuf, st_BufSize);
		*/
	}
	for(i = averageOffset; i < (st_CourceLog.Index - averageOffset); ++i)
	{
		velocityBuff = 0.0;
		angularBuff = 0.0;
		for(j = 0; j < averageNum; ++j)
		{
			velocityBuff += st_CourceLog.Pt_Array[i - averageOffset + j].Velocity;
			angularBuff  += st_CourceLog.Pt_Array[i - averageOffset + j].AngularVelocity;
		}
		velocityBuff /= (float32_t)averageNum;
		angularBuff  /= (float32_t)averageNum;
		st_CourceLog.Pt_Array[i].Radius = st_CalculateRadius(velocityBuff, angularBuff);

		/*
		st_BufSize = sprintf(st_SendBuf, "Radius[%d]: %.2f \r\n", i, st_CourceLog.Pt_Array[i].Radius);
		SCF_WriteData(st_SendBuf, st_BufSize);
		*/
	}
	for(i = (st_CourceLog.Index - averageOffset); i < st_CourceLog.Index; ++i)
	{
		velocityBuff = 0.0;
		angularBuff = 0.0;
		for(j = (i - averageOffset); j < st_CourceLog.Index; ++j)
		{
			velocityBuff += st_CourceLog.Pt_Array[j].Velocity;
			angularBuff  += st_CourceLog.Pt_Array[j].AngularVelocity;
		}
		velocityBuff /= (float32_t)(averageOffset + (st_CourceLog.Index - i));
		angularBuff  /= (float32_t)(averageOffset + (st_CourceLog.Index - i));
		st_CourceLog.Pt_Array[i].Radius = st_CalculateRadius(velocityBuff, angularBuff);

		/*
		st_BufSize = sprintf(st_SendBuf, "Radius[%d]: %.2f \r\n", i, st_CourceLog.Pt_Array[i].Radius);
		SCF_WriteData(st_SendBuf, st_BufSize);
		*/
	}


	// 目標速度計算
	BZR_SetBeepCount(3);
	for(i = 0; i < st_CourceLog.Index; ++i)
	{
		st_CourceLog.Pt_Array[i].TargetVelocity = sqrtf(st_CourceAnalysisParam.CentrifugalMax * st_CourceLog.Pt_Array[i].Radius);
		if(st_CourceLog.Pt_Array[i].TargetVelocity > st_CourceAnalysisParam.TargetVelocityMax)
		{
			st_CourceLog.Pt_Array[i].TargetVelocity = st_CourceAnalysisParam.TargetVelocityMax;
		}
		/*
		st_BufSize = sprintf(st_SendBuf, "Target[%d]: %.2f \r\n", i, st_CourceLog.Pt_Array[i].TargetVelocity);
		SCF_WriteData(st_SendBuf, st_BufSize);
		*/
	}

	// 速度シミュレーション
	BZR_SetBeepCount(4);
	upAccelBuff   = CVL_GetUpAccel();
	downAccelBuff = CVL_GetDownAccel();
	st_CourceLog.Pt_Array[0].v_n = st_CourceLog.Pt_Array[0].Velocity;
	for(i = 0; i < (st_CourceLog.Index - 1); ++i)
	{
		st_CourceLog.Pt_Array[i].s_n = st_CourceLog.Pt_Array[i + 1].Distance - st_CourceLog.Pt_Array[i].Distance;

		/*
		st_BufSize = sprintf(st_SendBuf, "s_n[%d]: %.4f \r\n", i, st_CourceLog.Pt_Array[i].s_n);
		SCF_WriteData(st_SendBuf, st_BufSize);
		*/

		if(st_CourceLog.Pt_Array[i + 1].TargetVelocity >= st_CourceLog.Pt_Array[i].TargetVelocity)
		{
			if((-5.0 < st_CourceLog.Pt_Array[i].SensorAngle) && (st_CourceLog.Pt_Array[i].SensorAngle > 5.0))
			{
				st_CourceLog.Pt_Array[i + 1].TargetVelocity = st_CourceLog.Pt_Array[i].TargetVelocity;
			}
		}

		if(st_CourceLog.Pt_Array[i + 1].TargetVelocity >= st_CourceLog.Pt_Array[i].v_n)
		{
			st_CourceLog.Pt_Array[i].a_n = upAccelBuff;
		}
		else
		{
			st_CourceLog.Pt_Array[i].a_n = -downAccelBuff;

			j = i;

			do
			{
				s_BrakeCalc = -(powf(st_CourceLog.Pt_Array[i + 1].TargetVelocity, 2.0) - powf(st_CourceLog.Pt_Array[j].v_n, 2.0)) / (2.0 * downAccelBuff);
				s_BrakeCalc *= 2.0;
				s_BrakeSum  = 0.0;
				for(n = j; n < i; ++n)
				{
					s_BrakeSum += st_CourceLog.Pt_Array[n].s_n;
				}
				/*
				st_BufSize = sprintf(st_SendBuf, "[%d] Calc:%.4f Sum:%.4f \r\n", j, s_BrakeCalc, s_BrakeSum);
				SCF_WriteData(st_SendBuf, st_BufSize);
				*/
				/*if(s_BrakeSum > s_BrakeCalc)
				{
					nop();
					break;
				}*/
				--j;
			}while(s_BrakeSum < s_BrakeCalc);

			++j;

			for(k = j; k < i; ++k)
			{
				if(st_CourceLog.Pt_Array[k].TargetVelocity > st_CourceLog.Pt_Array[i + 1].TargetVelocity)
				{
					st_CourceLog.Pt_Array[k].TargetVelocity = st_CourceLog.Pt_Array[i + 1].TargetVelocity;
				}
				st_CourceLog.Pt_Array[k].a_n = -downAccelBuff;
				st_CourceLog.Pt_Array[k + 1].v_n = sqrtf(2.0 * st_CourceLog.Pt_Array[k].a_n * st_CourceLog.Pt_Array[k].s_n + powf(st_CourceLog.Pt_Array[k].v_n, 2.0));

				/*
				st_BufSize = sprintf(st_SendBuf, "v_n[%d]: %.4f \r\n", k, st_CourceLog.Pt_Array[i].v_n);
				SCF_WriteData(st_SendBuf, st_BufSize);
				*/
			}
		}
		st_CourceLog.Pt_Array[i + 1].v_n = sqrtf(2.0 * st_CourceLog.Pt_Array[i].a_n * st_CourceLog.Pt_Array[i].s_n + powf(st_CourceLog.Pt_Array[i].v_n, 2.0));
	}

}


static float32_t st_CalculateRadius(float32_t velocityBuff, float32_t angularBuff)
{
	float32_t radiusBuff;

	if(angularBuff == 0.0)
	{
		radiusBuff = st_CourceAnalysisParam.RadiusMax;
	}
	else
	{
		if(angularBuff < 0.0)
		{
			angularBuff *= -1.0;
		}
		radiusBuff = velocityBuff / angularBuff;
		if(radiusBuff > st_CourceAnalysisParam.RadiusMax)
		{
			radiusBuff = st_CourceAnalysisParam.RadiusMax;
		}
	}

	return radiusBuff;
}


static void st_ExecuteDriveMode(void)
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
	int32_t markerCount;
	DAS_EnmAssertFlag assertFlag;

//	SSR_CalibSensor();

//	for(volatile int32_t i = 0; i < 1000000; ++i){
//		st_Decision = SWT_GetCenterDecision();
//	}

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

			CVL_SetTarget(0.2);

			TSK_Start(TSK_TASK3_CONTROL_VELOCITY);
//			TSK_Start(TSK_TASK4_CONTROL_ANGULAR);

			TSK_Start(TSK_TASKEACH1_ASSERT);
			DAS_Init();

			TSK_Start(TSK_TASK5_Judge_MARKER);

			st_StartRecordCource();

			CAV_Init();
			CVL_StartDriveMotor();
			CAV_StartDriveMotor();

			TSK_Start(TSK_TASK4_CONTROL_ANGULAR);

			//vecTargetTemp = 0.0;
			CVL_SetTarget(1.0);

//			TSK_Start(TSK_TASK6_RECORD_COURSE);
//			TSK_Start(TSK_TASK9_RECORD_CONTROL);
			TSK_Start(TSK_TASK7_PLAY_COURSE);

//			CVL_StartDriveMotor();
//			CAV_StartDriveMotor();
//			CVL_SetTarget(1.0);

			break;
		}
	}

	markerCount = 0;

	while(1)
	{
		SSR_StrMarkerData markerData;

		float32_t leftDuty;
		float32_t rightDuty;
		SSR_EnmMarkerKind kind;
		SSR_EnmMarkerState leftMarker, rightMarker;

		vecTarget = CVL_GetTarget();
		velocity = CVL_GetVelocity();

		/*
		st_BufSize = sprintf(st_SendBuf, "[%d, %d], vt:%.2f, v:%.2f \r\n"
											, st_CourceLog.Index
											, st_CourceLog.IndexMax
											, vecTarget
											, velocity);
		SCF_WriteData(st_SendBuf, st_BufSize);
		*/
		/*
		st_BufSize = sprintf(st_SendBuf, "[%d, %d], tmp:%.4f, off:%.4f, str:%.4f, dif:%.4f, log:%.4f, vt:%.2f, v:%.2f \r\n"
											, st_CourceLog.Index
											, st_CourceLog.IndexMax
											, st_CourceLogStoreParam.TempDistance
											, st_CourceLogStoreParam.OffsetDistance
											, st_CourceLogStoreParam.StartDistance
											, st_CourceLogStoreParam.DiffDistance
											, st_CourceLog.Pt_Array[st_CourceLog.Index].Distance
											, vecTarget
											, velocity);
		SCF_WriteData(st_SendBuf, st_BufSize);
		*/

//		theta = CAV_GetVirtualThetaDeg();
//		if(theta < 0.0)
//		{
//			theta *= -1.0;
//		}
//		if((theta < threshold) && (vecFlag == 0))
//		{
//			CVL_SetTarget(1.2 * vecTargetTemp);
//			vecFlag = 1;
//		}
//		if((theta >= threshold) && (vecFlag == 1))
//		{
//			CVL_SetTarget(vecTargetTemp);
//			vecFlag = 0;
//		}

//		st_SensorData = SSR_GetSensorData();

		assertFlag = DAS_GetAssertFlag();

		markerData = st_GetCourceMarker();

		if(markerData.MarkerKind == SSR_COURCE_MARKER_CROSS)
		{
			BZR_SetBeepCount(2);
		}

		if(markerData.MarkerKind == SSR_COURCE_MARKER_RIGHT)
		{
			++markerCount;
			BZR_SetBeepCount(3);
		}

		if((markerCount >= GOAL_COUNT) || (st_CourceLog.Index >= (st_CourceLog.IndexMax - 5)) || (assertFlag == DAS_ASSERTED))
		{
			TSK_Stop(TSK_TASK7_PLAY_COURSE);

			CVL_SetTarget(0.0);

			for(volatile int32_t i = 0; i < 10000000; ++i)	;

			CVL_StopDriveMotor();

//					TSK_Stop(TSK_TASK9_RECORD_CONTROL);
			TSK_Stop(TSK_TASKEACH1_ASSERT);

			for(volatile int32_t i = 0; i < 10000000; ++i)	;

			CSA_StopSensorMotor();

			for(volatile int32_t i = 0; i < 10000000; ++i)	;

			TSK_Stop(TSK_TASK2_CONTROL_SENSOR);
			TSK_Stop(TSK_TASK3_CONTROL_VELOCITY);
			TSK_Stop(TSK_TASK4_CONTROL_ANGULAR);

			// ドライブモード終了
			return;
		}
	}
}


static float32_t st_CalculateVelocityFromSensorAngle(float32_t sensorAngle, StrCourceAnalysisParam param)
{
	float32_t velocity;
	float32_t radius;

	if(sensorAngle < 0.0)
	{
		sensorAngle *= -1.0;
	}

	if(sensorAngle < 5.0)
	{
		velocity = param.TargetVelocityMax;
	}
	else
	{
		radius = 0.5 * LENGTH_SENSOR / sinf(0.5 * sensorAngle);
		velocity = sqrtf(param.CentrifugalMax * radius);
	}

	return velocity;
}
