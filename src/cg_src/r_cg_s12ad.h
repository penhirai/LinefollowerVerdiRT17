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
* File Name    : r_cg_s12ad.h
* Version      : Code Generator for RX64M V1.02.05.01 [11 Nov 2016]
* Device(s)    : R5F564MLDxFP
* Tool-Chain   : CCRX
* Description  : This file implements device driver for S12AD module.
* Creation Date: 2018/11/02
***********************************************************************************************************************/
#ifndef S12AD_H
#define S12AD_H

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/*
    A/D control register (ADCSR)
*/
/* Group B scan end interrupt enable (GBADIE) */
#define _0000_AD_GBADI_DISABLE               (0x0000U) /* Disables S12GBADI interrupt generation upon group B scan
                                                          completion */
#define _0040_AD_GBADI_ENABLE                (0x0040U) /* Enables S12GBADI interrupt generation upon group B scan
                                                          completion */
/* Double trigger mode select (DBLE) */
#define _0000_AD_DBLTRIGGER_DISABLE          (0x0000U) /* Disable double trigger mode */
#define _0080_AD_DBLTRIGGER_ENABLE           (0x0080U) /* Enable double trigger mode */
/* Trigger select (EXTRG) */
#define _0000_AD_SYNC_TRIGGER                (0x0000U) /* A/D conversion started by synchronous trigger */
#define _0100_AD_ASYNC_TRIGGER               (0x0100U) /* A/D conversion started by asynchronous trigger */
/* Trigger start enable (TRGE) */
#define _0000_AD_SYNCASYNCTRG_DISABLE        (0x0000U) /* A/D conversion synchronous or asynchronous trigger disable */
#define _0200_AD_SYNCASYNCTRG_ENABLE         (0x0200U) /* A/D conversion synchronous or asynchronous trigger enable */
/* Scan end interrupt enable (ADIE) */
#define _0000_AD_SCAN_END_INTERRUPT_DISABLE  (0x0000U) /* Disable S12ADI0 interrupt generation upon scan completion */
#define _1000_AD_SCAN_END_INTERRUPT_ENABLE   (0x1000U) /* Enable S12ADI0 interrupt generation upon scan completion */
/* Scan mode select (ADCS) */
#define _0000_AD_SINGLE_SCAN_MODE            (0x0000U) /* Single scan mode */
#define _2000_AD_GROUP_SCAN_MODE             (0x2000U) /* Group scan mode */
#define _4000_AD_CONTINUOUS_SCAN_MODE        (0x4000U) /* Continuous scan mode */
/* A/D conversion start (ADST) */
#define _0000_AD_CONVERSION_STOP             (0x0000U) /* Stop A/D conversion */
#define _8000_AD_CONVERSION_START            (0x8000U) /* Start A/D conversion */

/*
    A/D converted value addition count select register (ADADC)
*/
/* Addition Count Select (ADC[1:0]) */
#define _00_AD_1_TIME_CONVERSION             (0x00U) /* 1-time conversion */
#define _01_AD_2_TIME_CONVERSION             (0x01U) /* 2-time conversion */
#define _02_AD_3_TIME_CONVERSION             (0x02U) /* 3-time conversion */
#define _03_AD_4_TIME_CONVERSION             (0x03U) /* 4-time conversion */
/* Average Mode Enable bit (AVEE) */
#define _00_AD_ADDITION_MODE                 (0x00U) /* Addition mode */
#define _80_AD_AVERAGE_MODE                  (0x80U) /* Average mode */

/*
    A/D control extended register (ADCER)
*/
/* A/D Conversion Accuracy Specify (ADPRC) */
#define _0000_AD_RESOLUTION_12BIT            (0x0000U) /* 12-bit resolution */
#define _0002_AD_RESOLUTION_10BIT            (0x0002U) /* 10-bit resolution */
#define _0004_AD_RESOLUTION_8BIT             (0x0004U) /* 8-bit resolution */
/* Automatic clearing enable (ACE) */
#define _0000_AD_AUTO_CLEARING_DISABLE       (0x0000U) /* Disable auto clearing */
#define _0020_AD_AUTO_CLEARING_ENABLE        (0x0020U) /* Enable auto clearing */
/* A/D Self-diagnosis selection (DIAGVAL) */
#define _0000_AD_SELFTDIAGST_DISABLE         (0x0000U) /* Disable self-diagnosis */
#define _0100_AD_SELFTDIAGST_VREFH0_0        (0x0100U) /* Self-diagnosis using a voltage of 0V */
#define _0200_AD_SELFTDIAGST_VREFH0_HALF     (0x0200U) /* Self-diagnosis using a voltage of VREFH0_1/2 */
#define _0300_AD_SELFTDIAGST_VREFH0          (0x0300U) /* Self-diagnosis using a voltage of VREFH0_1 */
#define _0100_AD_SELFTDIAGST_AVCC1_0         (0x0100U) /* Self-diagnosis using a voltage of 0V */
#define _0200_AD_SELFTDIAGST_AVCC1_HALF      (0x0200U) /* Self-diagnosis using a voltage of AVCC1_1/2*/
#define _0300_AD_SELFTDIAGST_AVCC1           (0x0300U) /* Self-diagnosis using a voltage of AVCC1_1*/

/* A/D Self-diagnostic mode selection (DIAGLD) */
#define _0000_AD_SELFTDIAGST_ROTATION        (0x0000U) /* Rotation mode for self-diagnosis voltage */
#define _0400_AD_SELFTDIAGST_FIX             (0x0400U) /* Fixed mode for self-diagnosis voltage */
/* A/D Self-diagnostic enable (DIAGM) */
#define _0000_AD_SELFTDIAGST_DISABLE         (0x0000U) /* 12-bit self-diagnosis disable */
#define _0800_AD_SELFTDIAGST_ENABLE          (0x0800U) /* 12-bit self-diagnosis enable */
/* A/D data register format selection (ADRFMT) */
#define _0000_AD_RIGHT_ALIGNMENT             (0x0000U) /* Right-alignment for data register format */
#define _8000_AD_LEFT_ALIGNMENT              (0x8000U) /* Left-alignment for data register format */

/*
    A/D start trigger select register (ADSTRGR)
*/
/* A/D conversion start trigger select for group B (TRSB) */
#define _0001_AD_TRSB_TRGA0N                 (0x0001U) /* Compare match with or input capture to MTU0.TGRA */
#define _0002_AD_TRSB_TRGA1N                 (0x0002U) /* Compare match with or input capture to MTU1.TGRA */
#define _0003_AD_TRSB_TRGA2N                 (0x0003U) /* Compare match with or input capture to MTU2.TGRA */
#define _0004_AD_TRSB_TRGA3N                 (0x0004U) /* Compare match with or input capture to MTU3.TGRA */
#define _0005_AD_TRSB_TRGA4N                 (0x0005U) /* Compare match with or input capture to MTU4.TGRA,or an
                                                          underflow of MTU4.TCNT (in the trough) in complementary
                                                          PWM mode */
#define _0006_AD_TRSB_TRGA6N                 (0x0006U) /* Compare match with or input capture to MTU6.TGRA */
#define _0007_AD_TRSB_TRGA7N                 (0x0007U) /* Compare match with or input capture to MTU7.TGRA,or an
                                                          underflow of MTU7.TCNT (in the trough) in complementary
                                                          PWM mode */
#define _0008_AD_TRSB_TRG0N                  (0x0008U) /* Compare match with MTU0.TGRE */
#define _0009_AD_TRSB_TRG4AN                 (0x0009U) /* Compare match between MTU4.TADCORA and MTU4.TCNT */
#define _000A_AD_TRSB_TRG4BN                 (0x000AU) /* Compare match between MTU4.TADCORB and MTU4.TCNT */
#define _000B_AD_TRSB_TRG4BN_TRG4AN          (0x000BU) /* Compare match between MTU4.TADCORA and MTU4.TCNT, or
                                                          between MTU4.TADCORB and MTU4.TCNT */
#define _000C_AD_TRSB_TRG4ABN                (0x000CU) /* Compare match between MTU4.TADCORA and MTU4.TCNT, and
                                                          between MTU4.TADCORB and MTU4.TCNT (when interrupt skipping
                                                          function 2 is in use) */
#define _000D_AD_TRSB_TRG7AN                 (0x000DU) /* Compare match between MTU7.TADCORA and MTU7.TCNT */
#define _000E_AD_TRSB_TRG7BN                 (0x000EU) /* Compare match between MTU7.TADCORB and MTU7.TCNT */
#define _000F_AD_TRSB_TRG7AN_TRG7BN          (0x000FU) /* Compare match between MTU7.TADCORA and MTU7.TCNT, or between
                                                          MTU7.TADCORB and MTU7.TCNT */
#define _0010_AD_TRSB_TRG7ABN                (0x0010U) /* Compare match between MTU7.TADCORA and MTU7.TCNT, and between
                                                          MTU7.TADCORB and MTU7.TCNT (when interrupt skipping function
                                                          2 is in use) */
#define _0011_AD_TRSB_GTADTRA0N              (0x0011U) /* Compare match with GPT0.GTADTRA */
#define _0012_AD_TRSB_GTADTRB0N              (0x0012U) /* Compare match with GPT0.GTADTRB */
#define _0013_AD_TRSB_GTADTRA1N              (0x0013U) /* Compare match with GPT1.GTADTRA */
#define _0014_AD_TRSB_GTADTRB1N              (0x0014U) /* Compare match with GPT1.GTADTRB */
#define _0015_AD_TRSB_GTADTRA2N              (0x0015U) /* Compare match with GPT2.GTADTRA */
#define _0016_AD_TRSB_GTADTRB2N              (0x0016U) /* Compare match with GPT2.GTADTRB */
#define _0017_AD_TRSB_GTADTRA3N              (0x0017U) /* Compare match with GPT3.GTADTRA */
#define _0018_AD_TRSB_GTADTRB3N              (0x0018U) /* Compare match with GPT3.GTADTRB */
#define _0019_AD_TRSB_GTADTRA0N_GTADTRB0N    (0x0019U) /* Compare match with GPT0.GTADTRA or with GPT0.GTADTRB */
#define _001A_AD_TRSB_GTADTRA1N_GTADTRB1N    (0x001AU) /* Compare match with GPT1.GTADTRA or with GPT1.GTADTRB */
#define _001B_AD_TRSB_GTADTRA2N_GTADTRB2N    (0x001BU) /* Compare match with GPT2.GTADTRA or with GPT2.GTADTRB*/
#define _001C_AD_TRSB_GTADTRA3N_GTADTRB3N    (0x001CU) /* Compare match with GPT3.GTADTRA or with GPT3.GTADTRB */
#define _001D_AD_TRSB_TMTRG0AN_0             (0x001DU) /* Compare match between TMR0.TCORA0 and TMR0.TCNT0(unit0,ch0) */
#define _001E_AD_TRSB_TMTRG0AN_1             (0x001EU) /* Compare match between TMR2.TCORA0 and TMR2.TCNT0(unit1,ch0) */
#define _001F_AD_TRSB_TPTRGAN                (0x001FU) /* Compare match with or input capture to TPUn.TGRAn(n = 0to5) */
#define _0020_AD_TRSB_TPTRG0AN               (0x0020U) /* Compare match with or input capture to TPU0.TGRA0 */
#define _0030_AD_TRSB_ELCTRG0N_ELCTRG1N      (0x0030U) /* Trigger from ELC */
/* A/D conversion start trigger select for group A (TRSA) */
#define _0000_AD_TRSA_ADTRG                  (0x0000U) /* Input pin for the trigger */
#define _0100_AD_TRSA_TRGA0N                 (0x0100U) /* Compare match with or input capture to MTU0.TGRA */
#define _0200_AD_TRSA_TRGA1N                 (0x0200U) /* Compare match with or input capture to MTU1.TGRA */
#define _0300_AD_TRSA_TRGA2N                 (0x0300U) /* Compare match with or input capture to MTU2.TGRA */
#define _0400_AD_TRSA_TRGA3N                 (0x0400U) /* Compare match with or input capture to MTU3.TGRA */
#define _0500_AD_TRSA_TRGA4N                 (0x0500U) /* Compare match with or input capture to MTU4.TGRA or, in
                                                          complementary PWM mode,an underflow of MTU4.TCNT
                                                          (in the trough)*/
#define _0600_AD_TRSA_TRGA6N                 (0x0600U) /* Compare match with or input capture to MTU6.TGRA */
#define _0700_AD_TRSA_TRGA7N                 (0x0700U) /* Compare match with or input capture to MTU7.TGRA or, in
                                                          complementary PWM mode,an underflow of MTU7.TCNT
                                                          (in the trough)*/
#define _0800_AD_TRSA_TRG0N                  (0x0800U) /* Compare match with MTU0.TGRE */
#define _0900_AD_TRSA_TRG4AN                 (0x0900U) /* Compare match between MTU4.TADCORA and MTU4.TCNT */
#define _0A00_AD_TRSA_TRG4BN                 (0x0A00U) /* Compare match between MTU4.TADCORB and MTU4.TCNT */
#define _0B00_AD_TRSA_TRG4BN_TRG4AN          (0x0B00U) /* Compare match between MTU4.TADCORA and MTU4.TCNT, or between
                                                          MTU4.TADCORB and MTU4.TCNT */
#define _0C00_AD_TRSA_TRG4ABN                (0x0C00U) /* Compare match between MTU4.TADCORA and MTU4.TCNT, and between
                                                          MTU4.TADCORB and MTU4.TCNT (when interrupt skipping function
                                                          2 is in use) */
#define _0D00_AD_TRSA_TRG7AN                 (0x0D00U) /* Compare match between MTU7.TADCORA and MTU7.TCNT */
#define _0E00_AD_TRSA_TRG7BN                 (0x0E00U) /* Compare match between MTU7.TADCORB and MTU7.TCNT */
#define _0F00_AD_TRSA_TRG7AN_TRG7BN          (0x0F00U) /* Compare match between MTU7.TADCORA and MTU7.TCNT, or between
                                                          MTU7.TADCORB and MTU7.TCNT */
#define _1000_AD_TRSA_TRG7ABN                (0x1000U) /* Compare match between MTU7.TADCORA and MTU7.TCNT, and between
                                                          MTU7.TADCORB and MTU7.TCNT (when interrupt skipping function
                                                          2 is in use) */
#define _1100_AD_TRSA_GTADTRA0N              (0x1100U) /* Compare match with GPT0.GTADTRA */
#define _1200_AD_TRSA_GTADTRB0N              (0x1200U) /* Compare match with GPT0.GTADTRB */
#define _1300_AD_TRSA_GTADTRA1N              (0x1300U) /* Compare match with GPT1.GTADTRA */
#define _1400_AD_TRSA_GTADTRB1N              (0x1400U) /* Compare match with GPT1.GTADTRB */
#define _1500_AD_TRSA_GTADTRA2N              (0x1500U) /* Compare match with GPT2.GTADTRA */
#define _1600_AD_TRSA_GTADTRB2N              (0x1600U) /* Compare match with GPT2.GTADTRB */
#define _1700_AD_TRSA_GTADTRA3N              (0x1700U) /* Compare match with GPT3.GTADTRA */
#define _1800_AD_TRSA_GTADTRB3N              (0x1800U) /* Compare match with GPT3.GTADTRB */
#define _1900_AD_TRSA_GTADTRA0N_GTADTRB0N    (0x1900U) /* Compare match with GPT0.GTADTRA or with GPT0.GTADTRB */
#define _1A00_AD_TRSA_GTADTRA1N_GTADTRB1N    (0x1A00U) /* Compare match with GPT1.GTADTRA or with GPT1.GTADTRB */
#define _1B00_AD_TRSA_GTADTRA2N_GTADTRB2N    (0x1B00U) /* Compare match with GPT2.GTADTRA or with GPT2.GTADTRB */
#define _1C00_AD_TRSA_GTADTRA3N_GTADTRB3N    (0x1C00U) /* Compare match with GPT3.GTADTRA or with GPT3.GTADTRB */
#define _1D00_AD_TRSA_TMTRG0AN_0             (0x1D00U) /* Compare match between TMR0.TCORA0 and TMR0.TCNT0(unit0,ch0) */
#define _1E00_AD_TRSA_TMTRG0AN_1             (0x1E00U) /* Compare match between TMR2.TCORA0 and TMR2.TCNT0(unit1,ch0) */
#define _1F00_AD_TRSA_TPTRGAN                (0x1F00U) /* Compare match with or input capture to TPUn.TGRAn(n=0 to 5) */
#define _2000_AD_TRSA_TPTRG0AN               (0x2000U) /* Compare match with or input capture to TPU0.TGRAn */
#define _3000_AD_TRSA_ELCTRG0N_ELCTRG1N      (0x3000U) /* Trigger from ELC */

/*
    A/D converted extended input control register (ADEXICR)
*/
/* Temperature sensor output A/D conversion value addition mode selection (TSSAD) */
#define _0000_AD_TEMP_ADDITION_DISABLE       (0x0000U) /* Temperature sensor output A/D converted value addition/average
                                                          mode disabled */
#define _0001_AD_TEMP_ADDITION_ENABLE        (0x0001U) /* Temperature sensor output A/D converted value addition/average
                                                          mode enabled */
/* Internal reference voltage A/D conversion value addition mode select (OCSAD) */
#define _0000_AD_IREF_ADDITION_DISABLE       (0x0000U) /* Internal ref. voltage A/D converted value addition/average 
                                                          mode disabled */
#define _0002_AD_IREF_ADDITION_ENABLE        (0x0002U) /* Internal ref. voltage A/D converted value addition/average
                                                           mode enabled */
/* Temperature sensor output A/D conversion select (TSSA) */
#define _0000_AD_TEMP_GROUPA_DISABLE         (0x0000U) /* A/D conversion of temperature sensor output is disabled in 
                                                          group A  */
#define _0100_AD_TEMP_GROUPA_ENABLE          (0x0100U) /* A/D conversion of temperature sensor output is enabled in 
                                                          group A  */
/* Temperature sensor output A/D conversion select (TSSB) */
#define _0000_AD_TEMP_GROUPB_DISABLE         (0x0000U) /* A/D conversion of temperature sensor output is disabled in 
                                                          group B  */
#define _0400_AD_TEMP_GROUPB_ENABLE          (0x0400U) /* A/D conversion of temperature sensor output is enabled in 
                                                          group B  */
/* Internal reference voltage A/D conversion select (OCSA) */
#define _0000_AD_IREF_GROUPA_DISABLE         (0x0000U) /* A/D conversion of internal reference voltage is disabled in 
                                                          group A  */
#define _0200_AD_IREF_GROUPA_ENABLE          (0x0200U) /* A/D conversion of internal reference voltage is enabled in 
                                                          group A  */
/* Internal reference voltage A/D conversion select (OCSB) */
#define _0000_AD_IREF_GROUPB_DISABLE         (0x0000U) /* A/D conversion of internal reference voltage is disabled in 
                                                          group B  */
#define _0800_AD_IREF_GROUPB_ENABLE          (0x0800U) /* A/D conversion of internal reference voltage is enabled in 
                                                          group B  */
/* Extended analog input selection (EXSEL) */
#define _0000_AD_EXTNANEX1_IN_DISABLE        (0x0000U) /* Extended analog input disable */
#define _2000_AD_EXTNANEX1_IN_ENABLE         (0x2000U) /* Extended analog input enable */
/* Extended analog output control (EXOEN) */
#define _0000_AD_EXTNANEX0_OUT_DISABLE       (0x0000U) /* Extended analog output disable */
#define _8000_AD_EXTNANEX0_IN_ENABLE         (0x8000U) /* Extended analog output enable */

/*
    A/D Group Scan Priority Control Register (ADGSPCR)
*/
/* Group-A Priority Control Setting (PGS) */
#define _0000_AD_GPAPRIORITY_DISABLE         (0x0000U) /* Operation is without group A priority control */
#define _0001_AD_GPAPRIORITY_ENABLE          (0x0001U) /* Operation is with group A priority control */
/* Group B Restart Setting (GBRSCN) */
#define _0000_AD_GPBRESTART_DISABLE          (0x0000U) /* Group B not restart after discontinued due to Group A
                                                          priority */
#define _0002_AD_GPBRESTART_ENABLE           (0x0002U) /* Group B restart after discontinued due to Group A priority */
/* Group B Single Cycle Scan Continuous Start (GBRP) */
#define _0000_AD_GPBSCSCS_DISABLE            (0x0000U) /* Single cycle scan for group B not continuously activated */
#define _8000_AD_GPBSCSCS_ENABLE             (0x8000U) /* Single cycle scan for group B is continuously activated */

/* 
    A/D Compare Control Register (ADCMPCR)
*/
/* Window Function Setting (WCMPE) */
#define _00_AD_WINDOWFUNCTION_DISABLE        (0x00U) /* Window function disabled */ 
#define _40_AD_WINDOWFUNCTION_ENABLE         (0x40U) /* Window function enabled */
/* Compare Interrupt Enable (CMPIE) */
#define _00_AD_COMPARISON_INTERRUPT_DISABLE  (0x00U) /* S12CMPI interrupt is disabled */ 
#define _80_AD_COMPARISON_INTERRUPT_ENABLE   (0x80U) /* S12CMPI interrupt is enabled */

/* 
    A/D Compare Channel Select Extended Register (ADCMPANSER)
*/
/* Temperature Sensor Output Compare Select(CMPSTS) */
#define _00_AD_TEMP_COMPARE_DISABLE          (0x00U) /* Temperature sensor output is not a target for comparison */ 
#define _01_AD_TEMP_COMPARE_ENABLE           (0x01U) /* Temperature sensor output is a target for comparison */
/* Internal Reference Voltage Compare Select (CMPSOC) */
#define _00_AD_IREF_COMPARE_DISABLE          (0x00U) /* Internal reference voltage is not a target for comparison */ 
#define _02_AD_IREF_COMPARE_ENABLE           (0x02U) /* Internal reference voltage is a target for comparison */

/* 
    A/D Compare Level Extended Register (ADCMPLER)
*/
/* Temperature Sensor Output Compare Level Select(CMPLTS) */
#define _00_AD_TEMP_AD_TEMP_COMPARELEVEL     (0x00U) /* AD-converted value < ADCMPDR0 register value or A/D-converted
                                                      value > ADCMPDR1 register value */ 
#define _01_AD_TEMP_AD_TEMP_COMPARELEVEL     (0x01U) /* ADCMPDR0 register value < A/D-converted value < ADCMPDR1
                                                      register value */
/* Internal Reference Voltage Compare Level Select (CMPLOC) */
#define _00_AD_TEMP_AD_IREF_COMPARELEVEL     (0x00U) /* AD-converted value < ADCMPDR0 register value or A/D-converted
                                                      value > ADCMPDR1 register value */ 
#define _02_AD_TEMP_AD_IREF_COMPARELEVEL     (0x02U) /* ADCMPDR0 register value < A/D-converted value < ADCMPDR1
                                                      register value */

/*
    Interrupt Source Priority Register n (IPRn)
*/
/* Interrupt Priority Level Select (IPR[3:0]) */
#define _00_AD_PRIORITY_LEVEL0               (0x00U) /* Level 0 (interrupt disabled) */
#define _01_AD_PRIORITY_LEVEL1               (0x01U) /* Level 1 */
#define _02_AD_PRIORITY_LEVEL2               (0x02U) /* Level 2 */
#define _03_AD_PRIORITY_LEVEL3               (0x03U) /* Level 3 */
#define _04_AD_PRIORITY_LEVEL4               (0x04U) /* Level 4 */
#define _05_AD_PRIORITY_LEVEL5               (0x05U) /* Level 5 */
#define _06_AD_PRIORITY_LEVEL6               (0x06U) /* Level 6 */
#define _07_AD_PRIORITY_LEVEL7               (0x07U) /* Level 7 */
#define _08_AD_PRIORITY_LEVEL8               (0x08U) /* Level 8 */
#define _09_AD_PRIORITY_LEVEL9               (0x09U) /* Level 9 */
#define _0A_AD_PRIORITY_LEVEL10              (0x0AU) /* Level 10 */
#define _0B_AD_PRIORITY_LEVEL11              (0x0BU) /* Level 11 */
#define _0C_AD_PRIORITY_LEVEL12              (0x0CU) /* Level 12 */
#define _0D_AD_PRIORITY_LEVEL13              (0x0DU) /* Level 13 */
#define _0E_AD_PRIORITY_LEVEL14              (0x0EU) /* Level 14 */
#define _0F_AD_PRIORITY_LEVEL15              (0x0FU) /* Level 15 (highest) */

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define _003F_AD0_CHANNEL_SELECT_A0          (0x003FU) /* Channel select register A0 value */
#define _0000_AD0_ADDAVG_CHANNEL_SELECT0     (0x0000U) /* Converted value add/average function select register 0 */
#define _00_AD0_DISCONECT_SETTING            (0x00U) /* Disconnection detection assist setting */
#define _0000_AD0_COMPARECHANNEL_SELECT0     (0x0000U) /* Compare channel select register 0 value */
#define _0000_AD0_COMPARELEVEL_SELECT0       (0x0000U) /* Compare level register 0 value */
#define _3C_AD0_SAMPLING_STATE_0             (0x3CU) /* AN000 sampling state register value */
#define _3C_AD0_SAMPLING_STATE_1             (0x3CU) /* AN001 sampling state register value */
#define _3C_AD0_SAMPLING_STATE_2             (0x3CU) /* AN002 sampling state register value */
#define _3C_AD0_SAMPLING_STATE_3             (0x3CU) /* AN003 sampling state register value */
#define _3C_AD0_SAMPLING_STATE_4             (0x3CU) /* AN004 sampling state register value */
#define _3C_AD0_SAMPLING_STATE_5             (0x3CU) /* AN005 sampling state register value */

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
typedef enum
{
    ADCHANNEL0, ADCHANNEL1, ADCHANNEL2, ADCHANNEL3, ADCHANNEL4, ADCHANNEL5, ADCHANNEL6,
    ADCHANNEL7, ADCHANNEL8, ADCHANNEL9, ADCHANNEL10, ADCHANNEL11, ADCHANNEL12,
    ADCHANNEL13, ADCHANNEL14, ADCHANNEL15, ADCHANNEL16, ADCHANNEL17, ADCHANNEL18,
    ADCHANNEL19, ADCHANNEL20, ADSELFDIAGNOSIS, ADTEMPSENSOR, ADINTERREFVOLT,ADDATADUPLICATION,
    ADDATADUPLICATIONA, ADDATADUPLICATIONB
} ad_channel_t;

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void R_S12AD0_Create(void);
void R_S12AD0_Start(void);
void R_S12AD0_Stop(void);
void R_S12AD0_Get_ValueResult(ad_channel_t channel, uint16_t * const buffer);
void R_S12AD0_Set_CompareValue(uint16_t  reg_value0,uint16_t  reg_value1);

/* Start user code for function. Do not edit comment generated here */
void R_S12AD0_WaitAdcEnd(void);
/* End user code. Do not edit comment generated here */
#endif
