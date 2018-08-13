/*
 * Sensor.c
 *
 *  Created on: 2018/07/12
 *      Author: shuuichi
 */

#include "Sensor.h"
//#include "Log.h"
#include "r_cg_s12ad.h"
#include "TaskTimer.h"
#include "r_cg_rspi.h"

#define LINESENSOR_BUFF_SIZE 2
#define SENSOR_BUFF_SIZE 10
#define GYRO_BUFF_SIZE 3
#define GYRO_ADDR_WHO_AM_I 0x75
#define GYRO_ADDR_GYRO_Z 0x47

typedef struct strLineSensor
{
	int16_t LeftMarker;
	int16_t LeftCenter;
	int16_t RightCenter;
	int16_t RightMarker;
}StrLineSensor;

typedef enum enmLine
{
	DARK = 0,
	LIGHT,
	CALC
}EnmLine;

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

static StrLineSensor st_LineSensor[LINESENSOR_BUFF_SIZE];
static SSR_StrSensorDataArray st_SensorData;
//static SSR_StrSensorData st_SensorData[SENSOR_BUFF_SIZE];
static UnComm st_Tx;
static UnComm st_Rx;
//static uint8_t st_Tx[GYRO_BUFF_SIZE];
//static uint8_t st_Rx[GYRO_BUFF_SIZE];


static void st_ReadLineSensor(EnmLine kind);
static void st_ReadOtherSensor(void);
static void st_ReadAllAnalog(void);
static void st_ReadDarkSensor(void);
static void st_ReadLightSensor(void);
static void st_CalcLineSensor(void);

static void st_InitGyro(void);
static void st_StartReadGyro(void);

void SSR_Init(void)
{
	for(int32_t i; i < SSR_SENSOR_BUFF_SIZE; ++i)
	{
		st_SensorData.ArrayTemp[i].LeftMarker  = 0;
		st_SensorData.ArrayTemp[i].LeftCenter  = 0;
		st_SensorData.ArrayTemp[i].RightCenter = 0;
		st_SensorData.ArrayTemp[i].RightMarker = 0;
		st_SensorData.ArrayTemp[i].Potentio    = 0;
		st_SensorData.ArrayTemp[i].Power       = 0;
		st_SensorData.ArrayTemp[i].Gyro        = 0;
	}

	st_SensorData.Result.LeftMarker  = 0;
	st_SensorData.Result.LeftCenter  = 0;
	st_SensorData.Result.RightCenter = 0;
	st_SensorData.Result.RightMarker = 0;
	st_SensorData.Result.Potentio    = 0;
	st_SensorData.Result.Power       = 0;
	st_SensorData.Result.Gyro        = 0;

	st_SensorData.Index = 0;

	st_InitGyro();

	TSK_Start(TSK_TASKEACH0_SENSOR);
}


void SSR_TaskStartSensorGate(void)
{

}

void SSR_TaskStopsensorGate(void)
{

}


void SSR_GetAnalogSensor(void)
{
//	R_S12AD0_Start();
//	R_S12AD0_WaitAdcEnd();
//
//	st_ReadAllAnalog();

	st_ReadDarkSensor();

	st_ReadOtherSensor();
//	return &st_SensorData;
}


void SSR_TaskStartReadGyro(void)
{
	st_StartReadGyro();
}


void SSR_TaskCalcSensor(void)
{
//	R_S12AD0_Start();
//	R_S12AD0_WaitAdcEnd();
//
//	st_ReadAllAnalog();

	st_ReadLightSensor();
	st_CalcLineSensor();

	st_SensorData.ArrayTemp[st_SensorData.Index].Gyro = st_Rx.Context.Data;

	// execute filter
	st_SensorData.Result.LeftMarker  = st_SensorData.ArrayTemp[st_SensorData.Index].LeftMarker;
	st_SensorData.Result.LeftCenter  = st_SensorData.ArrayTemp[st_SensorData.Index].LeftCenter;
	st_SensorData.Result.RightCenter = st_SensorData.ArrayTemp[st_SensorData.Index].RightCenter;
	st_SensorData.Result.RightMarker = st_SensorData.ArrayTemp[st_SensorData.Index].RightMarker;
	st_SensorData.Result.Potentio    = st_SensorData.ArrayTemp[st_SensorData.Index].Potentio;
	st_SensorData.Result.Power       = st_SensorData.ArrayTemp[st_SensorData.Index].Power;
	st_SensorData.Result.Gyro        = st_SensorData.ArrayTemp[st_SensorData.Index].Gyro;

	++st_SensorData.Index;
	st_SensorData.Index %= SSR_SENSOR_BUFF_SIZE;



//	return &st_SensorData;
}


static void st_ReadLineSensor(EnmLine kind)
{
	ad_channel_t adc;

	R_S12AD0_Start();
	R_S12AD0_WaitAdcEnd();

	adc = ADCHANNEL0;
	R_S12AD0_Get_ValueResult(adc, &st_LineSensor[kind].RightMarker);

	adc = ADCHANNEL1;
	R_S12AD0_Get_ValueResult(adc, &st_LineSensor[kind].LeftMarker);

	adc = ADCHANNEL3;
	R_S12AD0_Get_ValueResult(adc, &st_LineSensor[kind].LeftCenter);

	adc = ADCHANNEL5;
	R_S12AD0_Get_ValueResult(adc, &st_LineSensor[kind].RightCenter);
}

static void st_ReadOtherSensor(void)
{
	ad_channel_t adc;

	R_S12AD0_Start();
	R_S12AD0_WaitAdcEnd();

	adc = ADCHANNEL2;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.ArrayTemp[st_SensorData.Index].Power);

	adc = ADCHANNEL4;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.ArrayTemp[st_SensorData.Index].Potentio);
}


static void st_ReadAllAnalog(void)
{
	ad_channel_t adc;

	R_S12AD0_Start();
	R_S12AD0_WaitAdcEnd();

	adc = ADCHANNEL0;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.ArrayTemp[st_SensorData.Index].RightMarker);

	adc = ADCHANNEL1;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.ArrayTemp[st_SensorData.Index].LeftMarker);

	adc = ADCHANNEL2;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.ArrayTemp[st_SensorData.Index].Power);

	adc = ADCHANNEL3;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.ArrayTemp[st_SensorData.Index].LeftCenter);

	adc = ADCHANNEL4;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.ArrayTemp[st_SensorData.Index].Potentio);

	adc = ADCHANNEL5;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.ArrayTemp[st_SensorData.Index].RightCenter);
}


static void st_ReadDarkSensor(void)
{
	EnmLine kind = DARK;

	st_ReadLineSensor(kind);
}

static void st_ReadLightSensor(void)
{
	EnmLine kind = LIGHT;

	st_ReadLineSensor(kind);
}


static void st_CalcLineSensor(void)
{
	EnmLine dark = DARK;
	EnmLine light = LIGHT;
	EnmLine calc = CALC;

	st_LineSensor[calc].LeftMarker  = st_LineSensor[light].LeftMarker  - st_LineSensor[dark].LeftMarker;
	st_LineSensor[calc].LeftCenter  = st_LineSensor[light].LeftCenter  - st_LineSensor[dark].LeftCenter;
	st_LineSensor[calc].RightCenter = st_LineSensor[light].RightCenter - st_LineSensor[dark].RightCenter;
	st_LineSensor[calc].RightMarker = st_LineSensor[light].RightMarker - st_LineSensor[dark].RightMarker;
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

	st_Tx.Context.Address = rwFlag | GYRO_ADDR_GYRO_Z;

	R_RSPI0_Send_Receive(st_Tx.Buff, GYRO_BUFF_SIZE, st_Rx.Buff);

	R_RSPI0_Start();
}
