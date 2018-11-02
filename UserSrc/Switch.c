/*
 * IrqSwitch.c
 *
 *  Created on: 2018/06/25
 *      Author: shuuichi
 */

#include "Switch.h"
#include "r_cg_icu.h"
#include "Buzzer.h"


typedef struct strDecision
{
	SWT_EnmDecision Decision;
	int32_t BeforeCall;
	int32_t AfterCall;
}StrDecision;


static SWT_StrSwitch st_Swt;
static int32_t st_udMin;
static int32_t st_udMax;
static int32_t st_rlMin;
static int32_t st_rlMax;
static StrDecision st_Decision;


static void st_CalcRL(void);
static void st_CalcUD(void);
static void st_CalcDecision(void);


void SWT_Init(int32_t rlMin, int32_t rlMax, int32_t udMin, int32_t udMax)
{
	st_Swt.Center = 0;
	st_Swt.Right  = 0;
	st_Swt.Left   = 0;
	st_Swt.Up     = 0;
	st_Swt.Down   = 0;
	st_Swt.UD_Dif = 0;
	st_Swt.RL_Dif = 0;
	st_rlMin = rlMin;
	st_rlMax = rlMax;
	st_udMin = udMin;
	st_udMax = udMax;

	st_Decision.Decision = SWT_DECISION_FALSE;
	st_Decision.AfterCall = 0;
	st_Decision.BeforeCall = 0;

	R_ICU_IRQ5_Start();
	R_ICU_IRQ6_Start();
	R_ICU_IRQ7_Start();
	R_ICU_IRQ8_Start();
	R_ICU_IRQ9_Start();
}

void SWT_CallCenter(void)
{
	++st_Swt.Center;

	BZR_SetBeepCount(2);
}


void SWT_CallRight(void)
{
	++st_Swt.Right;
	st_CalcRL();

	BZR_SetBeepCount(1);
}

void SWT_CallLeft(void)
{
	++st_Swt.Left;
	st_CalcRL();

	BZR_SetBeepCount(1);
}

void SWT_CallUp(void)
{
	++st_Swt.Up;
	st_CalcUD();

	BZR_SetBeepCount(1);
}

void SWT_CallDown(void)
{
	++st_Swt.Down;
	st_CalcUD();

	BZR_SetBeepCount(1);
}

void SWT_ClearSwitchCount(void)
{
	st_Swt.Center = 0;
	st_Swt.Right  = 0;
	st_Swt.Left   = 0;
	st_Swt.Up     = 0;
	st_Swt.Down   = 0;

	st_CalcRL();
	st_CalcUD();
}


SWT_StrSwitch *SWT_GetSwitch(void)
{
	return &st_Swt;
}

void SWT_SetSwitch(SWT_StrSwitch *swt)
{
	st_Swt = *swt;
}


SWT_EnmDecision SWT_GetCenterDecision(void)
{
	SWT_EnmDecision decision = SWT_DECISION_FALSE;

	st_CalcDecision();

	if(st_Decision.Decision == SWT_DECISION_TRUE)
	{
		decision = SWT_DECISION_TRUE;

		for(volatile int32_t i = 0; i < 100000; ++i)
		{
			st_CalcDecision();
			if(st_Decision.Decision == SWT_DECISION_FALSE)
			{
				break;
			}
		}
	}

	return decision;
}


static void st_CalcRL(void)
{
	st_Swt.RL_Dif = st_Swt.Right - st_Swt.Left;
	if(st_Swt.RL_Dif < st_rlMin)
	{
		st_Swt.Left = st_Swt.Right - st_rlMin;
		st_Swt.RL_Dif = st_rlMin;
	}
	if(st_Swt.RL_Dif > st_rlMax)
	{
		st_Swt.Right = st_Swt.Left + st_rlMax;
		st_Swt.RL_Dif = st_rlMax;
	}

//	for(volatile int32_t i = 0; i < 100000; ++i)	;
}

static void st_CalcUD(void)
{
	st_Swt.UD_Dif = st_Swt.Up - st_Swt.Down;
	if(st_Swt.UD_Dif < st_udMin)
	{
		st_Swt.Down = st_Swt.Up - st_udMin;
		st_Swt.UD_Dif = st_udMin;
	}
	if(st_Swt.UD_Dif > st_udMax)
	{
		st_Swt.Up = st_Swt.Down + st_udMax;
		st_Swt.UD_Dif = st_udMax;
	}

//	for(volatile int32_t i = 0; i < 100000; ++i)	;
}


static void st_CalcDecision(void)
{
	int32_t dif;

	st_Decision.AfterCall = st_Swt.Center;

	dif = st_Decision.AfterCall - st_Decision.BeforeCall;

	if(dif > 0)
	{
		st_Decision.Decision = SWT_DECISION_TRUE;
	}
	else
	{
		st_Decision.Decision = SWT_DECISION_FALSE;
	}

	st_Decision.BeforeCall = st_Decision.AfterCall;
}
