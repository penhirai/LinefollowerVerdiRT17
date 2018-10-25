/*
 * SciFifo.c
 *
 *  Created on: 2018/10/01
 *      Author: shuuichi
 */

#include "SciFifo.h"
#include "r_cg_sci.h"

void SCF_Init(void)
{
	R_SCI2_Start();
}

void SCF_WriteData(uint8_t *data, uint8_t size)
{
	// temporally でwait するだけ
	R_SCI2_Serial_Send(data, size);

	for(volatile int32_t i = 0; i < 15000 * size; ++i)
	{
		;
	}
}
