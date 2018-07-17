/*
 * IrqSwitch.h
 *
 *  Created on: 2018/06/25
 *      Author: shuuichi
 */

#ifndef SWITCH_H_
#define SWITCH_H_

#include "typedef.h"

#define SWT_RL_MIN 0
#define SWT_RL_MAX 9
#define SWT_UD_MIN 0
#define SWT_UD_MAX 99

typedef struct strSwitch
{
	int32_t Center;
	int32_t Right;
	int32_t Left;
	int32_t Up;
	int32_t Down;
	int32_t UD_Dif;
	int32_t RL_Dif;
}SWT_StrSwitch;

typedef enum enmDecision
{
	SWT_DECISION_FALSE = 0,
	SWT_DECISION_TRUE
}SWT_EnmDecision;


void SWT_Init(int32_t rlMin, int32_t rlMax, int32_t udMin, int32_t udMax);

void SWT_CallCenter(void);
void SWT_CallLeft(void);
void SWT_CallRight(void);
void SWT_CallUp(void);
void SWT_CallDown(void);

void SWT_ClearSwitchCount(void);

//uint32_t SWT_GetSwitchNumber(void);
//void SWT_SetSwitchNumber(uint32_t arrayNum);

SWT_StrSwitch *SWT_GetSwitch(void);
void SWT_SetSwitch(SWT_StrSwitch *swt);

SWT_EnmDecision SWT_GetCenterDecision(void);

#endif /* SWITCH_H_ */
