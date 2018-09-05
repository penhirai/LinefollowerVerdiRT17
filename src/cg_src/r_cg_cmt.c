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
* File Name    : r_cg_cmt.c
* Version      : Code Generator for RX64M V1.02.05.01 [11 Nov 2016]
* Device(s)    : R5F564MLDxFP
* Tool-Chain   : CCRX
* Description  : This file implements device driver for CMT module.
* Creation Date: 2018/09/03
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
#include "r_cg_cmt.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_CMT0_Create
* Description  : This function initializes the CMT0 channel.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CMT0_Create(void)
{
    /* Disable CMI interrupt */
    IEN(CMT0, CMI0) = 0U;

    /* Cancel CMT stop state in LPC */
    MSTP(CMT0) = 0U;

    /* Set control registers */
    CMT0.CMCR.WORD = _0000_CMT_CMCR_CKS_PCLK8 | _0040_CMT_CMCR_CMIE_ENABLE | _0080_CMT_CMCR_DEFAULT;
    CMT0.CMCOR = _02ED_CMT0_CMCOR_VALUE;

    /* Set CMI0 priority level */
    IPR(CMT0,CMI0) = _0F_CMT_PRIORITY_LEVEL15;
}
/***********************************************************************************************************************
* Function Name: R_CMT0_Start
* Description  : This function starts the CMT0 channel counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CMT0_Start(void)
{
    /* Enable CMI0 interrupt in ICU */
    IEN(CMT0,CMI0) = 1U;

    /* Start CMT0 count */
    CMT.CMSTR0.BIT.STR0 = 1U;
}
/***********************************************************************************************************************
* Function Name: R_CMT0_Stop
* Description  : This function stops the CMT0 channel counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CMT0_Stop(void)
{
    /* Disable CMI0 interrupt in ICU */
    IEN(CMT0,CMI0) = 0U;

    /* Stop CMT0 count */
    CMT.CMSTR0.BIT.STR0 = 0U;
}
/***********************************************************************************************************************
* Function Name: R_CMT1_Create
* Description  : This function initializes the CMT1 channel.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CMT1_Create(void)
{
    /* Disable CMI interrupt */
    IEN(CMT1, CMI1) = 0U;

    /* Cancel CMT stop state in LPC */
    MSTP(CMT1) = 0U;

    /* Set control registers */
    CMT1.CMCR.WORD = _0003_CMT_CMCR_CKS_PCLK512 | _0040_CMT_CMCR_CMIE_ENABLE | _0080_CMT_CMCR_DEFAULT;
    CMT1.CMCOR = _5B8D_CMT1_CMCOR_VALUE;

    /* Set CMI1 priority level */
    IPR(CMT1,CMI1) = _0A_CMT_PRIORITY_LEVEL10;
}
/***********************************************************************************************************************
* Function Name: R_CMT1_Start
* Description  : This function starts the CMT1 channel counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CMT1_Start(void)
{
    /* Enable CMI1 interrupt in ICU */
    IEN(CMT1,CMI1) = 1U;

    /* Start CMT1 count */
    CMT.CMSTR0.BIT.STR1 = 1U;
}
/***********************************************************************************************************************
* Function Name: R_CMT1_Stop
* Description  : This function stops the CMT1 channel counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CMT1_Stop(void)
{
    /* Disable CMI1 interrupt in ICU */
    IEN(CMT1,CMI1) = 0U;

    /* Stop CMT1 count */
    CMT.CMSTR0.BIT.STR1 = 0U;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
