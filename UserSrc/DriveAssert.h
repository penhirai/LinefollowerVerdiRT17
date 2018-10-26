/*
 * DriveAssert.h
 *
 *  Created on: 2018/10/18
 *      Author: shuuichi
 */

#ifndef DRIVEASSERT_H_
#define DRIVEASSERT_H_

#include "typedef.h"

typedef enum eumAssertFlag
{
	DAS_NOT_ASSERT = 0,
	DAS_ASSERTED
}DAS_EnmAssertFlag;


void DAS_Init(void);

void DAS_AssertTask(void);

DAS_EnmAssertFlag DAS_GetAssertFlag(void);
void DAS_ClearAssertFlag(void);

#endif /* DRIVEASSERT_H_ */
