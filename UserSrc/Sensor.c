/*
 * Sensor.c
 *
 *  Created on: 2018/07/12
 *      Author: shuuichi
 */

#include "Sensor.h"
#include "Log.h"
#include "r_cg_s12ad.h"
#include "TaskTimer.h"
#include "r_cg_rspi.h"

#define GYRO_ADDR_WHO_AM_I 0x75
#define GYRO_ADDR_

static LOG_StrSensorData st_SensorData;

static void st_ReadAllAnalog(void);
static void st_ReadDarkSensor(void);
static void st_ReadLightSensor(void);

static void st_InitGyro(void);


void SSR_Init(void)
{
	st_SensorData.LeftMarker  = 0;
	st_SensorData.LeftCenter  = 0;
	st_SensorData.RightCenter = 0;
	st_SensorData.RightMarker = 0;
	st_SensorData.Potentio    = 0;
	st_SensorData.Power       = 0;
	st_SensorData.Gyro        = 0;

	TSK_Start(TSK_TASKEACH0_SENSOR);
}


LOG_StrSensorData *SSR_GetSensor(void)
{
	R_S12AD0_Start();
	R_S12AD0_WaitAdcEnd();

	st_ReadAllAnalog();

	return st_SensorData;
}


void SSR_TaskCalcSensor(void)
{
	R_S12AD0_Start();
	R_S12AD0_WaitAdcEnd();

	st_ReadAllAnalog();

}

static void st_ReadAllAnalog(void)
{
	ad_channel_t adc;

	adc = ADCHANNEL0;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.RightMarker);

	adc = ADCHANNEL1;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.LeftMarker);

	adc = ADCHANNEL2;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.Power);

	adc = ADCHANNEL3;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.LeftCenter);

	adc = ADCHANNEL4;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.Potentio);

	adc = ADCHANNEL5;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.RightCenter);
}

static void st_ReadDarkSensor(void)
{

}
static void st_ReadLightSensor(void)
{

}


static void st_InitGyro(void)
{
	uint8_t tx[10];
	uint8_t rx[10];

	R_RSPI0_Start();

	R_RSPI0_Send_Receive(tx, sizeof(tx), rx);
}
