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

#define GYRO_BUFF_SIZE 3
#define GYRO_ADDR_WHO_AM_I 0x75
#define GYRO_ADDR_GYRO_Z 0x47


typedef enum enmComm
{
	READ = 0,
	WRITE
}EnmComm;

typedef union unComm
{
	uint8_t Buff[GYRO_BUFF_SIZE];
	struct
	{
		uint8_t Address;
		uint16_t Data;
	}Context;
}UnComm;

static LOG_StrSensorData st_SensorData;
static UnComm st_Tx;
static UnComm st_Rx;
//static uint8_t st_Tx[GYRO_BUFF_SIZE];
//static uint8_t st_Rx[GYRO_BUFF_SIZE];


static void st_ReadAllAnalog(void);
static void st_ReadDarkSensor(void);
static void st_ReadLightSensor(void);

static void st_InitGyro(void);
static void st_StartReadGyro(void);

void SSR_Init(void)
{
	st_SensorData.LeftMarker  = 0;
	st_SensorData.LeftCenter  = 0;
	st_SensorData.RightCenter = 0;
	st_SensorData.RightMarker = 0;
	st_SensorData.Potentio    = 0;
	st_SensorData.Power       = 0;
	st_SensorData.Gyro        = 0;

	st_InitGyro();

	TSK_Start(TSK_TASKEACH0_SENSOR);
}


LOG_StrSensorData *SSR_GetSensor(void)
{
	R_S12AD0_Start();
	R_S12AD0_WaitAdcEnd();

	st_ReadAllAnalog();

	return &st_SensorData;
}


void SSR_TaskStartReadGyro(void)
{
	st_StartReadGyro();
}


LOG_StrSensorData *SSR_TaskCalcSensor(void)
{
	R_S12AD0_Start();
	R_S12AD0_WaitAdcEnd();

	st_ReadAllAnalog();

	st_SensorData.Gyro = st_Rx.Context.Data;

	return &st_SensorData;
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
	EnmComm rwFlag = READ;

	for(int32_t i = 0; i < GYRO_BUFF_SIZE; ++i)
	{
		st_Tx.Buff[i] = 0;
		st_Tx.Buff[i] = 0;
	}

	st_Tx.Context.Address = rwFlag & GYRO_ADDR_WHO_AM_I;

	R_RSPI0_Send_Receive(st_Tx.Buff, GYRO_BUFF_SIZE, st_Rx.Buff);

	R_RSPI0_Start();
}

static void st_StartReadGyro(void)
{
	EnmComm rwFlag = READ;

	st_Tx.Context.Address = rwFlag & GYRO_ADDR_GYRO_Z;

	R_RSPI0_Send_Receive(st_Tx.Buff, GYRO_BUFF_SIZE, st_Rx.Buff);

	R_RSPI0_Start();
}
