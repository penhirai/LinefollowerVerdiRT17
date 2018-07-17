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

#define TASK_MAX 11

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
static void st_Task10(void);


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
		st_Task10();
	}

	// execute 1/10 interrupt
	switch(st_TaskState)
	{
		case TSK_TASK0_BUZZER:
			st_TaskState = TSK_TASK1;
			if(st_TaskSchedule[TSK_TASK0_BUZZER] == TASK_ON)
			{
				st_Task0();
			}
			break;
		case TSK_TASK1:
			st_TaskState = TSK_TASK2;
			if(st_TaskSchedule[TSK_TASK1] == TASK_ON)
			{
				st_Task1();
			}
			break;
		case TSK_TASK2:
			st_TaskState = TSK_TASK3;
			if(st_TaskSchedule[TSK_TASK2] == TASK_ON)
			{
				st_Task2();
			}
			break;
		case TSK_TASK3:
			st_TaskState = TSK_TASK4;
			if(st_TaskSchedule[TSK_TASK3] == TASK_ON)
			{
				st_Task3();
			}
			break;
		case TSK_TASK4:
			st_TaskState = TSK_TASK5;
			if(st_TaskSchedule[TSK_TASK4] == TASK_ON)
			{
				st_Task4();
			}
			break;
		case TSK_TASK5:
			st_TaskState = TSK_TASK6;
			if(st_TaskSchedule[TSK_TASK5] == TASK_ON)
			{
				st_Task5();
			}
			break;
		case TSK_TASK6:
			st_TaskState = TSK_TASK7;
			if(st_TaskSchedule[TSK_TASK6] == TASK_ON)
			{
				st_Task6();
			}
			break;
		case TSK_TASK7:
			st_TaskState = TSK_TASK8;
			if(st_TaskSchedule[TSK_TASK7] == TASK_ON)
			{
				st_Task7();
			}
			break;
		case TSK_TASK8:
			st_TaskState = TSK_TASK9;
			if(st_TaskSchedule[TSK_TASK8] == TASK_ON)
			{
				st_Task8();
			}
			break;
		case TSK_TASK9:
			st_TaskState = TSK_TASK0_BUZZER;
			if(st_TaskSchedule[TSK_TASK9] == TASK_ON)
			{
				st_Task9();
			}
			break;
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

}

static void st_Task2(void)
{

}

static void st_Task3(void)
{

}

static void st_Task4(void)
{

}

static void st_Task5(void)
{

}

static void st_Task6(void)
{

}

static void st_Task7(void)
{

}

static void st_Task8(void)
{

}

static void st_Task9(void)
{

}

static void st_Task10(void)
{
	SSR_TaskCalcSensor();
}
