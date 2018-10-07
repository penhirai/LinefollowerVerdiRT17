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
* File Name    : r_cg_sci_user.c
* Version      : Code Generator for RX64M V1.02.05.01 [11 Nov 2016]
* Device(s)    : R5F564MLDxFP
* Tool-Chain   : CCRX
* Description  : This file implements device driver for SCI module.
* Creation Date: 2018/10/08
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
#include "r_cg_sci.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
extern uint8_t * gp_sci2_tx_address;                /* SCI2 send buffer address */
extern uint16_t  g_sci2_tx_count;                   /* SCI2 send data number */
extern uint8_t * gp_sci2_rx_address;                /* SCI2 receive buffer address */
extern uint16_t  g_sci2_rx_count;                   /* SCI2 receive data number */
extern uint16_t  g_sci2_rx_length;                  /* SCI2 receive data length */
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: r_sci2_transmit_interrupt
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#if FAST_INTERRUPT_VECTOR == VECT_SCI2_TXI2
#pragma interrupt r_sci2_transmit_interrupt(vect=VECT(SCI2,TXI2),fint)
#else
#pragma interrupt r_sci2_transmit_interrupt(vect=VECT(SCI2,TXI2))
#endif
static void r_sci2_transmit_interrupt(void)
{
    if (g_sci2_tx_count > 0U)
    {
        SCI2.TDR = *gp_sci2_tx_address;
        gp_sci2_tx_address++;
        g_sci2_tx_count--;
    }
    else
    {
        SCI2.SCR.BIT.TIE = 0U;
        SCI2.SCR.BIT.TEIE = 1U;
    }
}

/***********************************************************************************************************************
* Function Name: r_sci2_transmitend_interrupt
* Description  : This function is TEI2 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void r_sci2_transmitend_interrupt(void)
{
    MPC.P50PFS.BYTE = 0x00U;
    PORT5.PMR.BYTE &= 0xFEU;
    SCI2.SCR.BIT.TIE = 0U;
    SCI2.SCR.BIT.TE = 0U;
    SCI2.SCR.BIT.TEIE = 0U;

    r_sci2_callback_transmitend();
}
/***********************************************************************************************************************
* Function Name: r_sci2_receive_interrupt
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#if FAST_INTERRUPT_VECTOR == VECT_SCI2_RXI2
#pragma interrupt r_sci2_receive_interrupt(vect=VECT(SCI2,RXI2),fint)
#else
#pragma interrupt r_sci2_receive_interrupt(vect=VECT(SCI2,RXI2))
#endif
static void r_sci2_receive_interrupt(void)
{
    if (g_sci2_rx_length > g_sci2_rx_count)
    {
        *gp_sci2_rx_address = SCI2.RDR;
        gp_sci2_rx_address++;
        g_sci2_rx_count++;

        if (g_sci2_rx_length <= g_sci2_rx_count)
        {
            r_sci2_callback_receiveend();
        }
    }
}
/***********************************************************************************************************************
* Function Name: r_sci2_receiveerror_interrupt
* Description  : This function is ERI2 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void r_sci2_receiveerror_interrupt(void)
{
    uint8_t err_type;

    /* Clear overrun, framing and parity error flags */
    err_type = SCI2.SSR.BYTE;
    err_type &= 0xC7U;
    err_type |= 0xC0U;
    SCI2.SSR.BYTE = err_type;
}
/***********************************************************************************************************************
* Function Name: r_sci2_callback_transmitend
* Description  : This function is a callback function when SCI2 finishes transmission.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void r_sci2_callback_transmitend(void)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_sci2_callback_receiveend
* Description  : This function is a callback function when SCI2 finishes reception.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void r_sci2_callback_receiveend(void)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
