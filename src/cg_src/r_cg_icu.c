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
* File Name    : r_cg_icu.c
* Version      : Code Generator for RX64M V1.02.05.01 [11 Nov 2016]
* Device(s)    : R5F564MLDxFP
* Tool-Chain   : CCRX
* Description  : This file implements device driver for ICU module.
* Creation Date: 2018/11/02
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
#include "r_cg_icu.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_ICU_Create
* Description  : This function initializes ICU module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_Create(void)
{
    /* Disable IRQ interrupts */
    ICU.IER[0x08].BYTE = _00_ICU_IRQ0_DISABLE | _00_ICU_IRQ1_DISABLE | _00_ICU_IRQ2_DISABLE | _00_ICU_IRQ3_DISABLE |
                         _00_ICU_IRQ4_DISABLE | _00_ICU_IRQ5_DISABLE | _00_ICU_IRQ6_DISABLE | _00_ICU_IRQ7_DISABLE;
    ICU.IER[0x09].BYTE = _00_ICU_IRQ8_DISABLE | _00_ICU_IRQ9_DISABLE | _00_ICU_IRQ10_DISABLE | _00_ICU_IRQ11_DISABLE |
                         _00_ICU_IRQ12_DISABLE | _00_ICU_IRQ13_DISABLE | _00_ICU_IRQ14_DISABLE | _00_ICU_IRQ15_DISABLE;

    /* Disable group interrupts */
    IEN(ICU,GROUPBL0) = 0U;
    IEN(ICU,GROUPAL0) = 0U;

    /* Set IRQ0~7 digital filter */
    ICU.IRQFLTE0.BYTE = _20_ICU_IRQ5_FILTER_ENABLE | _40_ICU_IRQ6_FILTER_ENABLE | _80_ICU_IRQ7_FILTER_ENABLE;
    ICU.IRQFLTC0.WORD = _0C00_ICU_IRQ5_FILTER_PCLK_64 | _3000_ICU_IRQ6_FILTER_PCLK_64 | _C000_ICU_IRQ7_FILTER_PCLK_64;

    /* Set IRQ8~15 digital filter */
    ICU.IRQFLTE1.BYTE = _01_ICU_IRQ8_FILTER_ENABLE | _02_ICU_IRQ9_FILTER_ENABLE;
    ICU.IRQFLTC1.WORD = _0003_ICU_IRQ8_FILTER_PCLK_64 | _000C_ICU_IRQ9_FILTER_PCLK_64;

    /* Set IRQ settings */
    ICU.IRQCR[5].BYTE = _00_ICU_IRQ_EDGE_LOW_LEVEL;
    ICU.IRQCR[6].BYTE = _00_ICU_IRQ_EDGE_LOW_LEVEL;
    ICU.IRQCR[7].BYTE = _00_ICU_IRQ_EDGE_LOW_LEVEL;
    ICU.IRQCR[8].BYTE = _00_ICU_IRQ_EDGE_LOW_LEVEL;
    ICU.IRQCR[9].BYTE = _00_ICU_IRQ_EDGE_LOW_LEVEL;

    /* Set IRQ5 priority level */
    IPR(ICU,IRQ5) = _0F_ICU_PRIORITY_LEVEL15;

    /* Set IRQ6 priority level */
    IPR(ICU,IRQ6) = _0F_ICU_PRIORITY_LEVEL15;

    /* Set IRQ7 priority level */
    IPR(ICU,IRQ7) = _0F_ICU_PRIORITY_LEVEL15;

    /* Set IRQ8 priority level */
    IPR(ICU,IRQ8) = _0F_ICU_PRIORITY_LEVEL15;

    /* Set IRQ9 priority level */
    IPR(ICU,IRQ9) = _0F_ICU_PRIORITY_LEVEL15;

    /* Set Group BL0 priority level */
    IPR(ICU,GROUPBL0) = _0D_ICU_PRIORITY_LEVEL13;

    /* Set Group AL0 priority level */
    IPR(ICU,GROUPAL0) = _0E_ICU_PRIORITY_LEVEL14;

    /* Enable group BL0 interrupt */
    IEN(ICU,GROUPBL0) = 1U;

    /* Enable group AL0 interrupt */
    IEN(ICU,GROUPAL0) = 1U;

    /* Set IRQ5 pin */
    MPC.P15PFS.BYTE = 0x40U;
    PORT1.PDR.BYTE &= 0xDFU;
    PORT1.PMR.BYTE &= 0xDFU;

    /* Set IRQ6 pin */
    MPC.P16PFS.BYTE = 0x40U;
    PORT1.PDR.BYTE &= 0xBFU;
    PORT1.PMR.BYTE &= 0xBFU;

    /* Set IRQ7 pin */
    MPC.P17PFS.BYTE = 0x40U;
    PORT1.PDR.BYTE &= 0x7FU;
    PORT1.PMR.BYTE &= 0x7FU;

    /* Set IRQ8 pin */
    MPC.P20PFS.BYTE = 0x40U;
    PORT2.PDR.BYTE &= 0xFEU;
    PORT2.PMR.BYTE &= 0xFEU;

    /* Set IRQ9 pin */
    MPC.P21PFS.BYTE = 0x40U;
    PORT2.PDR.BYTE &= 0xFDU;
    PORT2.PMR.BYTE &= 0xFDU;

}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ5_Start
* Description  : This function enables IRQ5 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ5_Start(void)
{
    /* Enable IRQ5 interrupt */
    IEN(ICU,IRQ5) = 1U;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ5_Stop
* Description  : This function disables IRQ5 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ5_Stop(void)
{
    /* Disable IRQ5 interrupt */
    IEN(ICU,IRQ5) = 0U;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ6_Start
* Description  : This function enables IRQ6 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ6_Start(void)
{
    /* Enable IRQ6 interrupt */
    IEN(ICU,IRQ6) = 1U;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ6_Stop
* Description  : This function disables IRQ6 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ6_Stop(void)
{
    /* Disable IRQ6 interrupt */
    IEN(ICU,IRQ6) = 0U;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ7_Start
* Description  : This function enables IRQ7 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ7_Start(void)
{
    /* Enable IRQ7 interrupt */
    IEN(ICU,IRQ7) = 1U;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ7_Stop
* Description  : This function disables IRQ7 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ7_Stop(void)
{
    /* Disable IRQ7 interrupt */
    IEN(ICU,IRQ7) = 0U;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ8_Start
* Description  : This function enables IRQ8 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ8_Start(void)
{
    /* Enable IRQ8 interrupt */
    IEN(ICU,IRQ8) = 1U;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ8_Stop
* Description  : This function disables IRQ8 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ8_Stop(void)
{
    /* Disable IRQ8 interrupt */
    IEN(ICU,IRQ8) = 0U;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ9_Start
* Description  : This function enables IRQ9 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ9_Start(void)
{
    /* Enable IRQ9 interrupt */
    IEN(ICU,IRQ9) = 1U;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ9_Stop
* Description  : This function disables IRQ9 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ9_Stop(void)
{
    /* Disable IRQ9 interrupt */
    IEN(ICU,IRQ9) = 0U;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
