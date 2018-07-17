/*
 * Buzzer.h
 *
 *  Created on: 2018/07/12
 *      Author: shuuichi
 */

#ifndef BUZZER_H_
#define BUZZER_H_

#include "typedef.h"

void BZR_Init(void);

void BZR_BuzzerOn(void);
void BZR_BuzzerOff(void);
void BZR_SetBeepCount(uint8_t count);

void BZR_BuzzerTask(void);

#endif /* BUZZER_H_ */
