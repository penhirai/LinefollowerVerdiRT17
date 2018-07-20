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
* File Name    : r_cg_mtu3.c
* Version      : Code Generator for RX64M V1.02.05.01 [11 Nov 2016]
* Device(s)    : R5F564MLDxFP
* Tool-Chain   : CCRX
* Description  : This file implements device driver for MTU3 module.
* Creation Date: 2018/07/18
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
#include "r_cg_mtu3.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_MTU3_Create
* Description  : This function initializes the MTU3 Unit0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MTU3_Create(void)
{
    /* Cancel MTU stop state in LPC */
    MSTP(MTU3) = 0U;

    /* Enable read/write to MTU registers */
    MTU.TRWERA.BYTE = _01_MTU_RWE_ENABLE;
    MTU.TRWERB.BYTE = _01_MTU_RWE_ENABLE;

    /* Stop all channels */
    MTU.TSTRA.BYTE = _00_MTU_CST0_OFF | _00_MTU_CST1_OFF | _00_MTU_CST2_OFF | _00_MTU_CST3_OFF | _00_MTU_CST4_OFF | 
                     _00_MTU_CST8_OFF;
    MTU5.TSTR.BYTE = _00_MTU_CSTW5_OFF | _00_MTU_CSTV5_OFF | _00_MTU_CSTU5_OFF;
    MTU.TSTRB.BYTE = _00_MTU_CST6_OFF | _00_MTU_CST7_OFF;

    /* Channel 0 is used as PWM1 mode */
    MTU0.TCR.BYTE = _00_MTU_PCLK_1 | _00_MTU_CKEG_RISE | _40_MTU_CKCL_B;
    MTU0.TCR2.BYTE = _00_MTU_PCLK_1;
    MTU.TSYRA.BIT.SYNC0 = 0U;
    MTU0.TMDR1.BYTE = _02_MTU_PWM1;
    MTU0.TIORH.BYTE = _00_MTU_IOA_DISABLE;
    MTU0.TIORL.BYTE = _00_MTU_IOC_DISABLE;
    MTU0.TGRA = _7530_TGRA_VALUE;
    MTU0.TGRB = _EA5F_TGRB_VALUE;
    MTU0.TGRC = _0064_TGRC_VALUE;
    MTU0.TGRD = _EA5F_TGRD_VALUE;
    MTU0.TGRE = _0064_TGRE_VALUE;
    MTU0.TGRF = _0064_TGRF_VALUE;
    MTU0.TIER.BYTE = _00_MTU_TGIEA_DISABLE | _00_MTU_TGIEB_DISABLE | _00_MTU_TGIEC_DISABLE | _00_MTU_TGIED_DISABLE | 
                     _00_MTU_TCIEV_DISABLE | _00_MTU_TTGE_DISABLE;
    MTU0.TIER2.BYTE = _00_MTU_TGIEE_DISABLE | _00_MTU_TGIEF_DISABLE | _00_MTU_TTGE2_DISABLE;

    /* Channel 1 is used as phase counting mode 1 */
    MTU1.TMDR1.BYTE = _04_MTU_COT1;
    MTU.TSYRA.BIT.SYNC1 = 0U;
    MTU1.TCR.BYTE = _00_MTU_CKCL_DIS;
    MTU1.TIOR.BYTE = _00_MTU_IOB_DISABLE | _00_MTU_IOA_DISABLE;
    MTU1.TIER.BYTE = _00_MTU_TGIEA_DISABLE | _00_MTU_TGIEB_DISABLE | _00_MTU_TCIEU_DISABLE | _00_MTU_TCIEV_DISABLE | 
                     _00_MTU_TTGE_DISABLE;
    MTU1.TGRA = _0063_TGRA_VALUE;
    MTU1.TGRB = _0063_TGRB_VALUE;

    /* Channel 2 is used as phase counting mode 1 */
    MTU2.TMDR1.BYTE = _04_MTU_COT1;
    MTU.TSYRA.BIT.SYNC2 = 0U;
    MTU2.TCR.BYTE = _00_MTU_CKCL_DIS;
    MTU2.TIOR.BYTE = _00_MTU_IOB_DISABLE | _00_MTU_IOA_DISABLE;
    MTU2.TIER.BYTE = _00_MTU_TGIEA_DISABLE | _00_MTU_TGIEB_DISABLE | _00_MTU_TCIEU_DISABLE | _00_MTU_TCIEV_DISABLE | 
                     _00_MTU_TTGE_DISABLE;
    MTU2.TGRA = _0063_TGRA_VALUE;
    MTU2.TGRB = _0063_TGRB_VALUE;

    /* Channel 3 is used as PWM1 mode */
    MTU3.TCR.BYTE = _00_MTU_PCLK_1 | _00_MTU_CKEG_RISE | _C0_MTU_CKCL_D;
    MTU3.TCR2.BYTE = _00_MTU_PCLK_1;
    MTU.TSYRA.BIT.SYNC3 = 0U;
    MTU3.TMDR1.BYTE = _02_MTU_PWM1;
    MTU3.TIORH.BYTE = _00_MTU_IOA_DISABLE;
    MTU3.TIORL.BYTE = _00_MTU_IOC_DISABLE;
    MTU3.TGRA = _0064_TGRA_VALUE;
    MTU3.TGRB = _0064_TGRB_VALUE;
    MTU3.TGRC = _0064_TGRC_VALUE;
    MTU3.TGRD = _0257_TGRD_VALUE;
    MTU3.TIER.BYTE = _00_MTU_TGIEA_DISABLE | _00_MTU_TGIEB_DISABLE | _00_MTU_TGIEC_DISABLE | _00_MTU_TGIED_DISABLE | 
                     _00_MTU_TCIEV_DISABLE | _00_MTU_TTGE_DISABLE;

    /* Channel 4 is used as PWM1 mode */
    MTU4.TCR.BYTE = _00_MTU_PCLK_1 | _00_MTU_CKEG_RISE | _C0_MTU_CKCL_D;
    MTU4.TCR2.BYTE = _00_MTU_PCLK_1;
    MTU.TSYRA.BIT.SYNC4 = 0U;
    MTU4.TMDR1.BYTE = _02_MTU_PWM1;
    MTU4.TCNT = 0x0000U;
    MTU4.TADCR.WORD = _0000_MTU_UT4AE_DISABLE | _0000_MTU_UT4BE_DISABLE | _0000_MTU_BF_DISABLE;
    MTU4.TIORH.BYTE = _00_MTU_IOA_DISABLE;
    MTU4.TIORL.BYTE = _00_MTU_IOC_DISABLE;
    MTU4.TGRA = _0064_TGRA_VALUE;
    MTU4.TGRB = _0257_TGRB_VALUE;
    MTU4.TGRC = _0064_TGRC_VALUE;
    MTU4.TGRD = _0257_TGRD_VALUE;
    MTU4.TIER.BYTE = _00_MTU_TGIEA_DISABLE | _00_MTU_TGIEB_DISABLE | _00_MTU_TGIEC_DISABLE | _00_MTU_TGIED_DISABLE | 
                     _00_MTU_TCIEV_DISABLE | _00_MTU_TTGE_DISABLE;
    
    /* Disable read/write to MTU registers */
    MTU.TRWERA.BYTE = _00_MTU_RWE_DISABLE;
    MTU.TRWERB.BYTE = _00_MTU_RWE_DISABLE;

}
/***********************************************************************************************************************
* Function Name: R_MTU3_C0_Start
* Description  : This function starts MTU unit0 channel 0 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MTU3_C0_Start(void)
{
    MTU.TSTRA.BYTE |= _01_MTU_CST0_ON;
}
/***********************************************************************************************************************
* Function Name: R_MTU3_C0_Stop
* Description  : This function stops MTU0 channel 0 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MTU3_C0_Stop(void)
{
    MTU.TSTRA.BIT.CST0 = 0U;
}
/***********************************************************************************************************************
* Function Name: R_MTU3_C1_Start
* Description  : This function starts MTU unit0 channel 1 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MTU3_C1_Start(void)
{
    MTU.TSTRA.BYTE |= _02_MTU_CST1_ON;
}
/***********************************************************************************************************************
* Function Name: R_MTU3_C1_Stop
* Description  : This function stops MTU0 channel 1 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MTU3_C1_Stop(void)
{
    MTU.TSTRA.BIT.CST1 = 0U;
}
/***********************************************************************************************************************
* Function Name: R_MTU3_C2_Start
* Description  : This function starts MTU unit0 channel 2 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MTU3_C2_Start(void)
{
    MTU.TSTRA.BYTE |= _04_MTU_CST2_ON;
}
/***********************************************************************************************************************
* Function Name: R_MTU3_C2_Stop
* Description  : This function stops MTU0 channel 2 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MTU3_C2_Stop(void)
{
    MTU.TSTRA.BIT.CST2 = 0U;
}
/***********************************************************************************************************************
* Function Name: R_MTU3_C3_Start
* Description  : This function starts MTU unit0 channel 3 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MTU3_C3_Start(void)
{
    MTU.TSTRA.BYTE |= _40_MTU_CST3_ON;
}
/***********************************************************************************************************************
* Function Name: R_MTU3_C3_Stop
* Description  : This function stops MTU0 channel 3 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MTU3_C3_Stop(void)
{
    MTU.TSTRA.BIT.CST3 = 0U;
}
/***********************************************************************************************************************
* Function Name: R_MTU3_C4_Start
* Description  : This function starts MTU unit0 channel 4 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MTU3_C4_Start(void)
{
    MTU.TSTRA.BYTE |= _80_MTU_CST4_ON;
}
/***********************************************************************************************************************
* Function Name: R_MTU3_C4_Stop
* Description  : This function stops MTU0 channel 4 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MTU3_C4_Stop(void)
{
    MTU.TSTRA.BIT.CST4 = 0U;
}

/* Start user code for adding. Do not edit comment generated here */
void R_MTU0_SetTGRA(uint16_t tgrx)
{
	MTU0.TGRA = tgrx;
}

uint16_t R_MTU0_GetTGRA(void)
{
	return MTU0.TGRA;
}

uint16_t R_MTU0_GetTGRB(void)
{
	return MTU0.TGRB;
}

void R_MTU0_SetTGRC(uint16_t tgrx)
{
	MTU0.TGRC = tgrx;
}

uint16_t R_MTU0_GetTGRC(void)
{
	return MTU0.TGRC;
}

uint16_t R_MTU0_GetTGRD(void)
{
	return MTU0.TGRD;
}


uint16_t R_MTU1_GetTcnt(void)
{
	return MTU1.TCNT;
}

uint16_t R_MTU2_GetTcnt(void)
{
	return MTU2.TCNT;
}


void R_MTU3_SetTGRA(uint16_t tgrx)
{
	MTU3.TGRA = tgrx;
}

uint16_t R_MTU3_GetTGRA(void)
{
	return MTU3.TGRA;
}

uint16_t R_MTU3_GetTGRB(void)
{
	return MTU3.TGRB;
}

void R_MTU3_SetTGRC(uint16_t tgrx)
{
	MTU3.TGRC = tgrx;
}

uint16_t R_MTU3_GetTGRC(void)
{
	return MTU3.TGRC;
}

uint16_t R_MTU3_GetTGRD(void)
{
	return MTU3.TGRD;
}


void R_MTU4_SetTGRA(uint16_t tgrx)
{
	MTU4.TGRA = tgrx;
}

uint16_t R_MTU4_GetTGRA(void)
{
	return MTU4.TGRA;
}

uint16_t R_MTU4_GetTGRB(void)
{
	return MTU4.TGRB;
}


void R_MTU4_SetTGRC(uint16_t tgrx)
{
	MTU4.TGRC = tgrx;
}

uint16_t R_MTU4_GetTGRC(void)
{
	return MTU4.TGRC;
}

uint16_t R_MTU4_GetTGRD(void)
{
	return MTU4.TGRD;
}

/* End user code. Do not edit comment generated here */
