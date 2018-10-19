/*
 * ControlStructure.h
 *
 *  Created on: 2018/10/15
 *      Author: shuuichi
 */

#ifndef CONTROLSTRUCTURE_H_
#define CONTROLSTRUCTURE_H_

#include "typedef.h"

typedef struct strControlFactor
{
	float32_t FF;
	float32_t P;
	float32_t I;
	float32_t D;
}CST_StrFactor;

typedef struct strControlError
{
	CST_StrFactor Factor;
	float32_t Sum;
	float32_t Past;
	float32_t Now;
}CST_StrError;

typedef struct strControlGain
{
	CST_StrFactor Factor;
	float32_t Scale;
}CST_StrGain;


#endif /* CONTROLSTRUCTURE_H_ */
