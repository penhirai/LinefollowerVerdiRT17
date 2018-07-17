/*
 * Led.c
 *
 *  Created on: 2018/07/02
 *      Author: hirai
 */

#include "Led.h"
#include <r_cg_port.h>

void LED_binaryOn(int32_t binary)
{
	uint8_t bin;

	bin = (uint8_t)(binary & 0x0F);

	switch(bin)
	{
		case 0x00:
			LED_Off(LED_0);
			LED_Off(LED_1);
			LED_Off(LED_2);
			LED_Off(LED_3);
			break;
		case 0x01:
			LED_On(LED_0);
			LED_Off(LED_1);
			LED_Off(LED_2);
			LED_Off(LED_3);
			break;
		case 0x02:
			LED_Off(LED_0);
			LED_On(LED_1);
			LED_Off(LED_2);
			LED_Off(LED_3);
			break;
		case 0x03:
			LED_On(LED_0);
			LED_On(LED_1);
			LED_Off(LED_2);
			LED_Off(LED_3);
			break;
		case 0x04:
			LED_Off(LED_0);
			LED_Off(LED_1);
			LED_On(LED_2);
			LED_Off(LED_3);
			break;
		case 0x05:
			LED_On(LED_0);
			LED_Off(LED_1);
			LED_On(LED_2);
			LED_Off(LED_3);
			break;
		case 0x06:
			LED_Off(LED_0);
			LED_On(LED_1);
			LED_On(LED_2);
			LED_Off(LED_3);
			break;
		case 0x07:
			LED_On(LED_0);
			LED_On(LED_1);
			LED_On(LED_2);
			LED_Off(LED_3);
			break;
		case 0x08:
			LED_Off(LED_0);
			LED_Off(LED_1);
			LED_Off(LED_2);
			LED_On(LED_3);
			break;
		case 0x09:
			LED_On(LED_0);
			LED_Off(LED_1);
			LED_Off(LED_2);
			LED_On(LED_3);
			break;
		case 0x0A:
			LED_Off(LED_0);
			LED_On(LED_1);
			LED_Off(LED_2);
			LED_On(LED_3);
			break;
		case 0x0B:
			LED_On(LED_0);
			LED_On(LED_1);
			LED_Off(LED_2);
			LED_On(LED_3);
			break;
		case 0x0C:
			LED_Off(LED_0);
			LED_Off(LED_1);
			LED_On(LED_2);
			LED_On(LED_3);
			break;
		case 0x0D:
			LED_On(LED_0);
			LED_Off(LED_1);
			LED_On(LED_2);
			LED_On(LED_3);
			break;
		case 0x0E:
			LED_Off(LED_0);
			LED_On(LED_1);
			LED_On(LED_2);
			LED_On(LED_3);
			break;
		case 0x0F:
			LED_On(LED_0);
			LED_On(LED_1);
			LED_On(LED_2);
			LED_On(LED_3);
			break;
		default:
			LED_AllOff();
			break;
	}
}

void LED_AllOff(void)
{
	LED_Off(LED_0);
	LED_Off(LED_1);
	LED_Off(LED_2);
	LED_Off(LED_3);
}

void LED_On(uint8_t num)
{
	R_PORT_EnmPort state = R_PORT_LOW;
	switch(num)
	{
		case LED_0:
			R_PORT_SetPC0(state);
			break;
		case LED_1:
			R_PORT_SetPC1(state);
			break;
		case LED_2:
			R_PORT_SetPC2(state);
			break;
		case LED_3:
			R_PORT_SetPC3(state);
			break;
		default:
			state = R_PORT_HIGH;
			R_PORT_SetPC0(state);
			R_PORT_SetPC1(state);
			R_PORT_SetPC2(state);
			R_PORT_SetPC3(state);
	}
}

void LED_Off(uint8_t num)
{
	R_PORT_EnmPort state = R_PORT_HIGH;
	switch(num)
	{
		case LED_0:
			R_PORT_SetPC0(state);
			break;
		case LED_1:
			R_PORT_SetPC1(state);
			break;
		case LED_2:
			R_PORT_SetPC2(state);
			break;
		case LED_3:
			R_PORT_SetPC3(state);
			break;
		default:
			state = R_PORT_HIGH;
			R_PORT_SetPC0(state);
			R_PORT_SetPC1(state);
			R_PORT_SetPC2(state);
			R_PORT_SetPC3(state);
	}
}
