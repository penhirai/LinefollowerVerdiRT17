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
* File Name    : r_cg_sci.c
* Version      : Code Generator for RX64M V1.02.05.01 [11 Nov 2016]
* Device(s)    : R5F564MLDxFP
* Tool-Chain   : CCRX
* Description  : This file implements device driver for SCI module.
* Creation Date: 2018/10/24
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
uint8_t * gp_sci2_tx_address;               /* SCI2 transmit buffer address */
uint16_t  g_sci2_tx_count;                  /* SCI2 transmit data number */
uint8_t * gp_sci2_rx_address;               /* SCI2 receive buffer address */
uint16_t  g_sci2_rx_count;                  /* SCI2 receive data number */
uint16_t  g_sci2_rx_length;                 /* SCI2 receive data length */
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_SCI2_Create
* Description  : This function initializes SCI2.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_SCI2_Create(void)
{
    /* Cancel SCI2 module stop state */
    MSTP(SCI2) = 0U;

    /* Set interrupt priority */
    IPR(SCI2, RXI2) = _0D_SCI_PRIORITY_LEVEL13;
    IPR(SCI2, TXI2) = _0D_SCI_PRIORITY_LEVEL13;

    /* Clear the control register */
    SCI2.SCR.BYTE = 0x00U;

    /* Set clock enable */
    SCI2.SCR.BYTE = _00_SCI_INTERNAL_SCK_UNUSED;

    /* Clear the SIMR1.IICM, SPMR.CKPH, and CKPOL bit, and set SPMR */
    SCI2.SIMR1.BIT.IICM = 0U;
    SCI2.SPMR.BYTE = _00_SCI_RTS | _00_SCI_CLOCK_NOT_INVERTED | _00_SCI_CLOCK_NOT_DELAYED;

    /* Set control registers */
    SCI2.SMR.BYTE = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_EVEN | _00_SCI_PARITY_DISABLE | 
                    _00_SCI_DATA_LENGTH_8 | _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_MODE;
    SCI2.SCMR.BYTE = _00_SCI_SERIAL_MODE | _00_SCI_DATA_INVERT_NONE | _00_SCI_DATA_LSB_FIRST | 
                     _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
    SCI2.SEMR.BYTE = _00_SCI_LOW_LEVEL_START_BIT | _00_SCI_NOISE_FILTER_DISABLE | _10_SCI_8_BASE_CLOCK | 
                     _40_SCI_BAUDRATE_DOUBLE | _00_SCI_BIT_MODULATION_DISABLE;

    /* Set bitrate */
    SCI2.BRR = 0x20U;

    /* Set RXD2 pin */
    MPC.P52PFS.BYTE = 0x0AU;
    PORT5.PMR.BYTE |= 0x04U;

    /* Set TXD2 pin */
    PORT5.PODR.BYTE |= 0x01U;
    MPC.P50PFS.BYTE = 0x0AU;
    PORT5.PMR.BYTE |= 0x01U;
    PORT5.PDR.BYTE |= 0x01U;
}
/***********************************************************************************************************************
* Function Name: R_SCI2_Start
* Description  : This function starts SCI2.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_SCI2_Start(void)
{
    /* Clear interrupt flag */
    IR(SCI2,TXI2) = 0U;
    IR(SCI2,RXI2) = 0U;

    /* Enable SCI interrupt */
    IEN(SCI2,TXI2) = 1U;
    ICU.GENBL0.BIT.EN4 = 1U;
    IEN(SCI2,RXI2) = 1U;
    ICU.GENBL0.BIT.EN5 = 1U;
}
/***********************************************************************************************************************
* Function Name: R_SCI2_Stop
* Description  : This function stops SCI2.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_SCI2_Stop(void)
{
    PORT5.PMR.BYTE &= 0xFEU;
    SCI2.SCR.BIT.TE = 0U;      /* Disable serial transmit */
    SCI2.SCR.BIT.RE = 0U;      /* Disable serial receive */
    SCI2.SCR.BIT.TIE = 0U;     /* Disable TXI interrupt */
    SCI2.SCR.BIT.RIE = 0U;     /* Disable RXI and ERI interrupt */

    /* Disable SCI interrupt */
    IR(SCI2,TXI2) = 0U;
    IEN(SCI2,TXI2) = 0U;
    ICU.GENBL0.BIT.EN4 = 0U;
    IR(SCI2,RXI2) = 0U;
    IEN(SCI2,RXI2) = 0U;
    ICU.GENBL0.BIT.EN5 = 0U;
}
/***********************************************************************************************************************
* Function Name: R_SCI2_Serial_Receive
* Description  : This function receives SCI2 data.
* Arguments    : rx_buf -
*                    receive buffer pointer (Not used when receive data handled by DTC or DMAC)
*                rx_num -
*                    buffer size (Not used when receive data handled by DTC or DMAC)
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_SCI2_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
    MD_STATUS status = MD_OK;

    if (rx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        g_sci2_rx_count = 0U;
        g_sci2_rx_length = rx_num;
        gp_sci2_rx_address = rx_buf;
        SCI2.SCR.BIT.RIE = 1U;
        SCI2.SCR.BIT.RE = 1U;
    }

    return (status);
}
/***********************************************************************************************************************
* Function Name: R_SCI2_Serial_Send
* Description  : This function transmits SCI2 data.
* Arguments    : tx_buf -
*                    transfer buffer pointer (Not used when transmit data handled by DTC)
*                tx_num -
*                    buffer size (Not used when transmit data handled by DTC or DMAC)
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_SCI2_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
    MD_STATUS status = MD_OK;

    if (tx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        gp_sci2_tx_address = tx_buf;
        g_sci2_tx_count = tx_num;
        MPC.P50PFS.BYTE = 0x0AU;
        PORT5.PMR.BYTE |= 0x01U;
        SCI2.SCR.BIT.TIE = 1U;
        SCI2.SCR.BIT.TE = 1U;
    }

    return (status);
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
