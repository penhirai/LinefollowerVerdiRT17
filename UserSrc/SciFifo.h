/*
 * SciFifo.h
 *
 *  Created on: 2018/10/01
 *      Author: shuuichi
 */

#ifndef SCIFIFO_H_
#define SCIFIFO_H_

#include "typedef.h"

void SCF_Init(void);

void SCF_WriteData(uint8_t *data, uint8_t size);

#endif /* SCIFIFO_H_ */
