/*
 * TaskTimer.h
 *
 *  Created on: 2018/07/10
 *      Author: shuuichi
 */

#ifndef TASKTIMER_H_
#define TASKTIMER_H_

#include "typedef.h"

typedef enum enmTask
{
	TSK_TASK0_BUZZER = 0,
	TSK_TASK1_SENSOR_FILTER,
	TSK_TASK2_CONTROL_SENSOR,
	TSK_TASK3_CONTROL_VELOCITY,
	TSK_TASK4_CONTROL_ANGULAR,
	TSK_TASK5_Judge_MARKER,
	TSK_TASK6_RECORD_COURSE,
	TSK_TASK7_PLAY_COURSE,
	TSK_TASK8,
	TSK_TASK9,
	TSK_TASKEACH0_SENSOR,
	TSK_TASKEACH1_ASSERT
}TSK_EnmTask;

void TSK_Init(void);
void TSK_Main(void);

void TSK_Start(TSK_EnmTask task);
void TSK_Stop(TSK_EnmTask task);

#endif /* TASKTIMER_H_ */
