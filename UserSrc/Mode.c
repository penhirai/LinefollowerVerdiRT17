/*
 * Mode.c
 *
 *  Created on: 2018/06/26
 *      Author: shuuichi
 */

#include "Mode.h"
#include "Switch.h"
#include "Led.h"
#include "Test.h"
#include "FunctionTimer.h"
#include "SciFifo.h"
#include "Tracer.h"

#define MODE_KIND_MIN 0


typedef enum enmModeKind
{
	CASE_SEARCH_MODE = MODE_KIND_MIN,
	CASE_TRACE_MODE,
	CASE_TRACE2_MODE,
	CASE_TEST_MODE,
	MODE_KIND_MAX
}EnmModeKind;


static SWT_StrSwitch *st_Swt;
static EnmModeKind st_ModeKind;
static SWT_EnmDecision st_Decision;



void MDE_InitMode(void)
{
	st_ModeKind  = CASE_SEARCH_MODE;
	FTR_Init();
}


void MDE_SelectMode(void)
{
	st_ModeKind  = CASE_SEARCH_MODE;


	SWT_Init(MODE_KIND_MIN, MODE_KIND_MAX, SWT_UD_MIN, SWT_UD_MAX);
	st_Swt = SWT_GetSwitch();

	while(1)
	{
		st_ModeKind = (EnmModeKind)st_Swt->RL_Dif;
		st_Decision = SWT_GetCenterDecision();

		LED_binaryOn(st_ModeKind);
		switch(st_ModeKind)
		{
			case CASE_SEARCH_MODE:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					TRC_StartSearchMode(1.2, 1.2);
				}
				break;
			case CASE_TRACE_MODE:
				if(st_Decision == SWT_DECISION_TRUE)
				{
//					CVL_SetTargetUpAccel(7.0);
//					CVL_SetTargetDownAccel(7.0);
					TRC_StartSearchMode(1.3, 1.2);
					//TRC_StartDriveMode();
				}
				break;
			case CASE_TRACE2_MODE:
				if(st_Decision == SWT_DECISION_TRUE)
				{
//					CVL_SetTargetUpAccel(7.0);
//					CVL_SetTargetDownAccel(7.0);
					TRC_StartSearchMode(1.4, 1.3);
					//TRC_StartDriveMode();
				}
				break;
			case CASE_TEST_MODE:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					TST_TestMode();
					SWT_Init(MODE_KIND_MIN, MODE_KIND_MAX, SWT_UD_MIN, SWT_UD_MAX);
				}
				break;
			default:
				break;
		}
	}
}

