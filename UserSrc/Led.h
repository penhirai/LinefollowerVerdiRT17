/*
 * Led.h
 *
 *  Created on: 2018/07/02
 *      Author: hirai
 */

#ifndef LED_H_
#define LED_H_

#include "typedef.h"

#define LED_0 0
#define LED_1 1
#define LED_2 2
#define LED_3 3


void LED_binaryOn(int32_t binary);
void LED_AllOff(void);
void LED_On(uint8_t num);
void LED_Off(uint8_t num);

#endif /* LED_H_ */
