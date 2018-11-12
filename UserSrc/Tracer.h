/*
 * Tracer.h
 *
 *  Created on: 2018/10/15
 *      Author: shuuichi
 */

#ifndef TRACER_H_
#define TRACER_H_

#include "typedef.h"

void TRC_Init(void);

void TRC_StartSearchMode(void);
void TRC_StartDriveMode(void);

void TRC_RecordCourceTask(void);

#endif /* TRACER_H_ */
