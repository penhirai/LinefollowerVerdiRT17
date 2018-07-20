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
* File Name    : r_cg_hardware_setup.c
* Version      : Code Generator for RX64M V1.02.05.01 [11 Nov 2016]
* Device(s)    : R5F564MLDxFP
* Tool-Chain   : CCRX
* Description  : This file implements system initializing function.
* Creation Date: 2018/07/20
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
#include "r_cg_cgc.h"
#include "r_cg_icu.h"
#include "r_cg_port.h"
#include "r_cg_mtu3.h"
#include "r_cg_cmt.h"
#include "r_cg_sci.h"
#include "r_cg_rspi.h"
#include "r_cg_s12ad.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Systeminit
* Description  : This function initializes every macro.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Systeminit(void)
{
    /* Enable writing to registers related to operating modes, LPC, CGC and software reset */
    SYSTEM.PRCR.WORD = 0xA50BU; 

    /* Enable writing to MPC pin function control registers */
    MPC.PWPR.BIT.B0WI = 0U;
    MPC.PWPR.BIT.PFSWE = 1U;

    /* Initialize non-existent pins */
    PORT0.PDR.BYTE = 0x0FU;
    PORT1.PDR.BYTE = 0x03U;
    PORT5.PDR.BYTE = 0x40U;
    PORT6.PDR.BYTE = 0xFFU;
    PORT7.PDR.BYTE = 0xFFU;
    PORT8.PDR.BYTE = 0xCFU;
    PORT9.PDR.BYTE = 0xFFU;
    PORTF.PDR.BYTE = 0x3FU;
    PORTG.PDR.BYTE = 0xFFU;
    PORTJ.PDR.BYTE = 0x20U;

    /* Set peripheral settings */
    R_CGC_Create();
    R_ICU_Create();
    R_PORT_Create();
    R_MTU3_Create();
    R_CMT0_Create();
    R_SCI2_Create();
    R_RSPI0_Create();
    R_S12AD0_Create();

    /* Disable writing to MPC pin function control registers */
    MPC.PWPR.BIT.PFSWE = 0U;    
    MPC.PWPR.BIT.B0WI = 1U;     

    /* Enable protection */
    SYSTEM.PRCR.WORD = 0xA500U;  
}
/***********************************************************************************************************************
* Function Name: HardwareSetup
* Description  : This function initializes hardware setting.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void HardwareSetup(void)
{
    R_Systeminit();
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
