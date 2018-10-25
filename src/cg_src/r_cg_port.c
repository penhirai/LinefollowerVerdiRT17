/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIESREGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2013, 2016 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_port.c
* Version      : Code Generator for RX64M V1.02.05.01 [11 Nov 2016]
* Device(s)    : R5F564MLDxFP
* Tool-Chain   : CCRX
* Description  : This file implements device driver for Port module.
* Creation Date: 2018/10/26
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_port.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_PORT_Create
* Description  : This function initializes the Port I/O.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_PORT_Create(void)
{
    PORTB.DSCR.BYTE = _00_Pm0_HIDRV_OFF | _00_Pm2_HIDRV_OFF | _00_Pm4_HIDRV_OFF;
    PORTC.DSCR.BYTE = _00_Pm0_HIDRV_OFF | _00_Pm1_HIDRV_OFF | _00_Pm2_HIDRV_OFF | _00_Pm3_HIDRV_OFF;
    PORTD.DSCR.BYTE = _00_Pm0_HIDRV_OFF | _00_Pm1_HIDRV_OFF;
    PORTE.DSCR.BYTE = _00_Pm0_HIDRV_OFF | _00_Pm3_HIDRV_OFF | _00_Pm4_HIDRV_OFF;
    PORT1.PMR.BYTE = 0x00U;
    PORT1.PDR.BYTE = _10_Pm4_MODE_OUTPUT | _03_PDR1_DEFAULT;
    PORTB.PMR.BYTE = 0x00U;
    PORTB.PDR.BYTE = _01_Pm0_MODE_OUTPUT | _04_Pm2_MODE_OUTPUT | _10_Pm4_MODE_OUTPUT;
    PORTC.PMR.BYTE = 0x00U;
    PORTC.PDR.BYTE = _01_Pm0_MODE_OUTPUT | _02_Pm1_MODE_OUTPUT | _04_Pm2_MODE_OUTPUT | _08_Pm3_MODE_OUTPUT;
    PORTD.PMR.BYTE = 0x00U;
    PORTD.PDR.BYTE = _01_Pm0_MODE_OUTPUT | _02_Pm1_MODE_OUTPUT;
    PORTE.PMR.BYTE = 0x00U;
    PORTE.PDR.BYTE = _01_Pm0_MODE_OUTPUT | _08_Pm3_MODE_OUTPUT | _10_Pm4_MODE_OUTPUT;
}

/* Start user code for adding. Do not edit comment generated here */
void R_PORT_SetP05(R_PORT_EnmPort state)
{
	PORT0.PODR.BIT.B5 = (uint8_t)state;
}

void R_PORT_SetP07(R_PORT_EnmPort state)
{
	PORT0.PODR.BIT.B7 = (uint8_t)state;
}

void R_PORT_SetP14(R_PORT_EnmPort state)
{
	PORT1.PODR.BIT.B4 = (uint8_t)state;
}

void R_PORT_SetPB0(R_PORT_EnmPort state)
{
	PORTB.PODR.BIT.B0 = (uint8_t)state;
}

void R_PORT_SetPB2(R_PORT_EnmPort state)
{
	PORTB.PODR.BIT.B2 = (uint8_t)state;
}

void R_PORT_SetPB4(R_PORT_EnmPort state)
{
	PORTB.PODR.BIT.B4 = (uint8_t)state;
}

void R_PORT_SetPC0(R_PORT_EnmPort state)
{
	PORTC.PODR.BIT.B0 = (uint8_t)state;
}

void R_PORT_SetPC1(R_PORT_EnmPort state)
{
	PORTC.PODR.BIT.B1 = (uint8_t)state;
}

void R_PORT_SetPC2(R_PORT_EnmPort state)
{
	PORTC.PODR.BIT.B2 = (uint8_t)state;
}

void R_PORT_SetPC3(R_PORT_EnmPort state)
{
	PORTC.PODR.BIT.B3 = (uint8_t)state;
}

void R_PORT_SetPD0(R_PORT_EnmPort state)
{
	PORTD.PODR.BIT.B0 = (uint8_t)state;
}

void R_PORT_SetPD1(R_PORT_EnmPort state)
{
	PORTD.PODR.BIT.B1 = (uint8_t)state;
}

void R_PORT_SetPE0(R_PORT_EnmPort state)
{
	PORTE.PODR.BIT.B0 = (uint8_t)state;
}

void R_PORT_SetPE3(R_PORT_EnmPort state)
{
	PORTE.PODR.BIT.B3 = (uint8_t)state;
}

void R_PORT_SetPE4(R_PORT_EnmPort state)
{
	PORTE.PODR.BIT.B4 = (uint8_t)state;
}
/* End user code. Do not edit comment generated here */
