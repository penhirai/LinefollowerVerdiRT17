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
* File Name    : r_cg_icu_user.c
* Version      : Code Generator for RX64M V1.02.05.01 [11 Nov 2016]
* Device(s)    : R5F564MLDxFP
* Tool-Chain   : CCRX
* Description  : This file implements device driver for ICU module.
* Creation Date: 2018/10/22
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
#include "Switch.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: r_icu_irq5_interrupt
* Description  : This function is IRQ5 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#if FAST_INTERRUPT_VECTOR == VECT_ICU_IRQ5
#pragma interrupt r_icu_irq5_interrupt(vect=VECT(ICU,IRQ5),fint)
#else
#pragma interrupt r_icu_irq5_interrupt(vect=VECT(ICU,IRQ5))
#endif
static void r_icu_irq5_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
	R_ICU_IRQ5_Stop();
	SWT_CallDown();
	R_CMT1_Start();
    /* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_icu_irq6_interrupt
* Description  : This function is IRQ6 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#if FAST_INTERRUPT_VECTOR == VECT_ICU_IRQ6
#pragma interrupt r_icu_irq6_interrupt(vect=VECT(ICU,IRQ6),fint)
#else
#pragma interrupt r_icu_irq6_interrupt(vect=VECT(ICU,IRQ6))
#endif
static void r_icu_irq6_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
	R_ICU_IRQ6_Stop();
	SWT_CallRight();
	R_CMT1_Start();
    /* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_icu_irq7_interrupt
* Description  : This function is IRQ7 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#if FAST_INTERRUPT_VECTOR == VECT_ICU_IRQ7
#pragma interrupt r_icu_irq7_interrupt(vect=VECT(ICU,IRQ7),fint)
#else
#pragma interrupt r_icu_irq7_interrupt(vect=VECT(ICU,IRQ7))
#endif
static void r_icu_irq7_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
	R_ICU_IRQ7_Stop();
	SWT_CallUp();
	R_CMT1_Start();
    /* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_icu_irq8_interrupt
* Description  : This function is IRQ8 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#if FAST_INTERRUPT_VECTOR == VECT_ICU_IRQ8
#pragma interrupt r_icu_irq8_interrupt(vect=VECT(ICU,IRQ8),fint)
#else
#pragma interrupt r_icu_irq8_interrupt(vect=VECT(ICU,IRQ8))
#endif
static void r_icu_irq8_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
	R_ICU_IRQ8_Stop();
	SWT_CallLeft();
	R_CMT1_Start();
    /* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_icu_irq9_interrupt
* Description  : This function is IRQ9 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#if FAST_INTERRUPT_VECTOR == VECT_ICU_IRQ9
#pragma interrupt r_icu_irq9_interrupt(vect=VECT(ICU,IRQ9),fint)
#else
#pragma interrupt r_icu_irq9_interrupt(vect=VECT(ICU,IRQ9))
#endif
static void r_icu_irq9_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
	R_ICU_IRQ9_Stop();
	SWT_CallCenter();
	R_CMT1_Start();
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
