/*
 * TaskTimer.c
 *
 *  Created on: 2018/07/10
 *      Author: shuuichi
 */

#include "TaskTimer.h"
#include "Led.h"
#include "r_cg_cmt.h"
#include "Buzzer.h"
#include "Sensor.h"
#include "ControlSensorAngle.h"
#include "ControlVelocity.h"
#include "ControlAngularVelocity.h"
#include "DriveAssert.h"

#define TASK_MAX 12

typedef enum enmTaskSchedule
{
	TASK_OFF = 0,
	TASK_ON
}EnmTaskSchedule;

static TSK_EnmTask st_TaskState;
static EnmTaskSchedule st_TaskSchedule[TASK_MAX];


static void st_Task0(void);
static void st_Task1(void);
static void st_Task2(void);
static void st_Task3(void);
static void st_Task4(void);
static void st_Task5(void);
static void st_Task6(void);
static void st_Task7(void);
static void st_Task8(void);
static void st_Task9(void);
static void st_Task10_StartCalc(void);
static void st_Task10_StartComm(void);
static void st_Task11(void);


void TSK_Init(void)
{
	st_TaskState = TSK_TASK0_BUZZER;

	for(int32_t i = 0; i < TASK_MAX; ++i)
	{
		st_TaskSchedule[i] = TASK_OFF;
	}

	R_CMT0_Start();
}


void TSK_Main(void)
{
	LED_On(LED_3);

	// execute every interrupt
	if(st_TaskSchedule[TSK_TASKEACH0_SENSOR] == TASK_ON)
	{
		st_Task10_StartCalc();
	}

	if(st_TaskSchedule[TSK_TASKEACH1_ASSERT] == TASK_ON)
	{
		st_Task11();
	}

	// execute 1/10 interrupt
	switch(st_TaskState)
	{
		case TSK_TASK0_BUZZER:
			st_TaskState = TSK_TASK1_SENSOR_FILTER;
			if(st_TaskSchedule[TSK_TASK0_BUZZER] == TASK_ON)
			{
				st_Task0();
			}
			break;
		case TSK_TASK1_SENSOR_FILTER:
			st_TaskState = TSK_TASK2_CONTROL_SENSOR;
			if(st_TaskSchedule[TSK_TASK1_SENSOR_FILTER] == TASK_ON)
			{
				st_Task1();
			}
			break;
		case TSK_TASK2_CONTROL_SENSOR:
			st_TaskState = TSK_TASK3_CONTROL_VELOCITY;
			if(st_TaskSchedule[TSK_TASK2_CONTROL_SENSOR] == TASK_ON)
			{
				st_Task2();
			}
			break;
		case TSK_TASK3_CONTROL_VELOCITY:
			st_TaskState = TSK_TASK4_CONTROL_ANGULAR;
			if(st_TaskSchedule[TSK_TASK3_CONTROL_VELOCITY] == TASK_ON)
			{
				st_Task3();
			}
			break;
		case TSK_TASK4_CONTROL_ANGULAR:
			st_TaskState = TSK_TASK5_Judge_MARKER;
			if(st_TaskSchedule[TSK_TASK4_CONTROL_ANGULAR] == TASK_ON)
			{
				st_Task4();
			}
			break;
		case TSK_TASK5_Judge_MARKER:
			st_TaskState = TSK_TASK6_RECORD_COURSE;
			if(st_TaskSchedule[TSK_TASK5_Judge_MARKER] == TASK_ON)
			{
				st_Task5();
			}
			break;
		case TSK_TASK6_RECORD_COURSE:
			st_TaskState = TSK_TASK7_PLAY_COURSE;
			if(st_TaskSchedule[TSK_TASK6_RECORD_COURSE] == TASK_ON)
			{
				st_Task6();
			}
			break;
		case TSK_TASK7_PLAY_COURSE:
			st_TaskState = TSK_TASK8;
			if(st_TaskSchedule[TSK_TASK7_PLAY_COURSE] == TASK_ON)
			{
				st_Task7();
			}
			break;
		case TSK_TASK8:
			st_TaskState = TSK_TASK9_RECORD_CONTROL;
			if(st_TaskSchedule[TSK_TASK8] == TASK_ON)
			{
				st_Task8();
			}
			break;
		case TSK_TASK9_RECORD_CONTROL:
			st_TaskState = TSK_TASK0_BUZZER;
			if(st_TaskSchedule[TSK_TASK9_RECORD_CONTROL] == TASK_ON)
			{
				st_Task9();
			}
			break;
	}

	if(st_TaskSchedule[TSK_TASKEACH0_SENSOR] == TASK_ON)
	{
		st_Task10_StartComm();
	}

	LED_Off(LED_3);
}


void TSK_Start(TSK_EnmTask task)
{
	st_TaskSchedule[task] = TASK_ON;
}


void TSK_Stop(TSK_EnmTask task)
{
	st_TaskSchedule[task] = TASK_OFF;
}


static void st_Task0(void)
{
	BZR_BuzzerTask();
}

static void st_Task1(void)
{
	SSR_TaskCalcFilter();
}

static void st_Task2(void)
{
	CSA_ControlSensorTask();
}

static void st_Task3(void)
{
	CVL_ControlTask();
}

static void st_Task4(void)
{
	CAV_ControlTask();
}

static void st_Task5(void)
{
	SSR_TaskJudgeMarkerSensor();
}

static void st_Task6(void)
{
	// 10mm 刻み
	// マーカーを検出したら
}

static void st_Task7(void)
{

}

static void st_Task8(void)
{

}

static void st_Task9(void)
{
	LOG_RecordControl();
}

static void st_Task10_StartCalc(void)
{
	SSR_TaskCalcSensor();
	SSR_TaskStopSensorGate();
}

static void st_Task10_StartComm(void)
{
	SSR_TaskGetAnalogSensor();
	SSR_TaskStartReadGyro();
	SSR_TaskStartSensorGate();
}

static void st_Task11(void)
{
	DAS_AssertTask();
}
