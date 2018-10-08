/*
 * Buzzer.c
 *
 *  Created on: 2018/07/12
 *      Author: shuuichi
 */

#include "Buzzer.h"
#include "FunctionTimer.h"
#include "TaskTimer.h"


static uint32_t st_BeepCount;
static uint32_t st_TaskCounter;
static uint32_t st_TaskInterval;


void BZR_Init(void)
{
	st_BeepCount = 0;
	st_TaskCounter = 0;
	st_TaskInterval = 100;

	TSK_Start(TSK_TASK0_BUZZER);
	FTR_StartBuzzerTimer();
}


void BZR_BuzzerOn(void)
{
	FTR_SetBuzzerDuty(50);
}

void BZR_BuzzerOff(void)
{
	FTR_SetBuzzerDuty(0);
}


void BZR_SetBeepCount(uint8_t count)
{
	st_BeepCount = 2 * count;
	st_TaskCounter = st_TaskInterval;
}

void BZR_BuzzerTask(void)
{
	if(st_BeepCount > 0)
	{
		++st_TaskCounter;

		if(st_TaskCounter >= st_TaskInterval)
		{
			st_TaskCounter = 0;

			if((st_BeepCount % 2) == 0)
			{
				BZR_BuzzerOn();
			}
			else
			{
				BZR_BuzzerOff();
			}

			--st_BeepCount;
		}
	}
}
