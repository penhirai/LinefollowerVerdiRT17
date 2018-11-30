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

//void TRC_StartSearchMode(void);
void TRC_StartSearchMode(float32_t velocity);
//void TRC_StartDriveMode(void);
void TRC_DriveMode(void);

void TRC_RecordCourceTask(void);
void TRC_PlayCourceTask(void);

void st_AnalyzeCource(void);

#endif /* TRACER_H_ */
