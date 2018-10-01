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
#include <r_cg_port.h>

#define SEND_BUF_SIZE 20

#define LINESENSOR_BUFF_SIZE 2
#define SENSOR_BUFF_SIZE 10
#define GYRO_BUFF_SIZE 1

#define GYRO_ADDR_GYRO_CONFIG 0x1B
#define GYRO_ADDR_GYRO_Z 0x47
//#define GYRO_ADDR_GYRO_Z 0x41
#define GYRO_ADDR_SIG_PATH_RESET 0x68
#define GYRO_ADDR_PWR_MGMT_1 0x6B
#define GYRO_ADDR_PWR_MGMT_2 0x6C
#define GYRO_ADDR_WHO_AM_I 0x75

#define GYRO_DATA_FULLSCALE_250  0x00
#define GYRO_DATA_FULLSCALE_500  0x08
#define GYRO_DATA_FULLSCALE_1000 0x10
#define GYRO_DATA_FULLSCALE_2000 0x18

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
	WRITE = 0,
	READ
}EnmComm;


typedef struct strComm
{
	uint32_t Buff;
	uint8_t Address;
	int16_t Data;
}StrComm;

/*
typedef union unComm
{
	uint32_t Buff;//[GYRO_BUFF_SIZE];
	struct
	{
		uint8_t dummy;
		int16_t Data;
		uint8_t Address;
	}Context;
}UnComm;
*/


static uint8_t st_SendBuf[SEND_BUF_SIZE];

static StrLineSensor st_LineSensor[LINESENSOR_BUFF_SIZE];
static SSR_StrSensorDataArray st_SensorData;
//static SSR_StrSensorData st_SensorData[SENSOR_BUFF_SIZE];
static StrComm st_Tx;
static StrComm st_Rx;
//static uint8_t st_Tx[GYRO_BUFF_SIZE];
//static uint8_t st_Rx[GYRO_BUFF_SIZE];

static void st_SetSensorGateOn(void);
static void st_SetSensorGateOff(void);
static void st_ReadLineSensor(EnmLine kind);
static void st_ReadOtherSensor(void);
static void st_ReadAllAnalog(void);
static void st_ReadDarkSensor(void);
static void st_ReadLightSensor(void);
static void st_CalcLineSensor(void);

static void st_InitGyro(void);
static void st_StartReadGyro(void);
static void st_CommunicateGyro(EnmComm rwFlag, uint8_t address, uint8_t data);
static void st_ParseGyro(void);

void SSR_Init(void)
{
	st_SetSensorGateOff();

	for(int32_t i = 0; i < SSR_SENSOR_BUFF_SIZE; ++i)
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
	st_SetSensorGateOn();
}

void SSR_TaskStopSensorGate(void)
{
	st_SetSensorGateOff();
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

	st_ParseGyro();
	st_SensorData.ArrayTemp[st_SensorData.Index].Gyro = st_Rx.Data;//st_Rx.Context.Data;

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


void SSR_PrintAllSensor(void)
{
	sprintf(st_SendBuf, "L M:%d, L C:%d,R C:%d, R M:%d, Pow:%d, Pot:%d, Gyro:%d \r\n", st_SensorData.Result.LeftMarker, st_SensorData.Result.LeftCenter, st_SensorData.Result.RightCenter, st_SensorData.Result.RightMarker, st_SensorData.Result.Power, st_SensorData.Result.Potentio, st_SensorData.Result.Gyro);

	R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));
}


static void st_SetSensorGateOn(void)
{
	R_PORT_EnmPort state = R_PORT_LOW;

	R_PORT_SetP14(state);
}

static void st_SetSensorGateOff(void)
{
	R_PORT_EnmPort state = R_PORT_HIGH;

	R_PORT_SetP14(state);
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
	uint8_t data = 0x00;

	/*
	for(int32_t i = 0; i < GYRO_BUFF_SIZE; ++i)
	{
		st_Tx.Buff[i] = 0;
		st_Rx.Buff[i] = 0;
	}
	*/
	st_Tx.Buff = 0;
	st_Rx.Buff = 0;

	R_RSPI0_Start();

	// MPU6500: 0x70
	rwFlag = READ;
	st_CommunicateGyro(rwFlag, GYRO_ADDR_WHO_AM_I, 0x00);

	//
	rwFlag = WRITE;
	data = 0x00 | GYRO_DATA_FULLSCALE_2000;
	st_CommunicateGyro(rwFlag, GYRO_ADDR_GYRO_CONFIG, data);

	rwFlag = READ;
	st_CommunicateGyro(rwFlag, GYRO_ADDR_GYRO_CONFIG, 0x00);

	// 加速度，GYRO Enable
	rwFlag = WRITE;
	data = 0x00;
	st_CommunicateGyro(rwFlag, GYRO_ADDR_PWR_MGMT_2, data);

	rwFlag = READ;
	st_CommunicateGyro(rwFlag, GYRO_ADDR_PWR_MGMT_2, 0x00);
//	rwFlag = WRITE;
//	st_CommunicateGyro(rwFlag, GYRO_ADDR_PWR_MGMT_1, 0x81);

//	rwFlag = WRITE;
//	st_CommunicateGyro(rwFlag, GYRO_ADDR_SIG_PATH_RESET, 0x03);

	/*
	st_Tx.Context.Address = rwFlag & GYRO_ADDR_WHO_AM_I;

	R_RSPI0_Send_Receive(st_Tx.Buff, GYRO_BUFF_SIZE, st_Rx.Buff);

	R_RSPI0_Start();
	*/
}

static void st_StartReadGyro(void)
{

	EnmComm rwFlag = READ;

	st_CommunicateGyro(rwFlag, GYRO_ADDR_GYRO_Z, 0x00);
	/*
	st_Tx.Context.Address = rwFlag | GYRO_ADDR_GYRO_Z;

	R_RSPI0_Send_Receive(st_Tx.Buff, GYRO_BUFF_SIZE, st_Rx.Buff);

	R_RSPI0_Start();
	*/
}


static void st_CommunicateGyro(EnmComm rwFlag, uint8_t address, uint8_t data)
{
	st_Tx.Address = (rwFlag << 7) | address;
	st_Tx.Data = data;

	st_Tx.Buff = (st_Tx.Address << 16) | (st_Tx.Data);

	R_RSPI0_Send_Receive(&st_Tx.Buff, GYRO_BUFF_SIZE, &st_Rx.Buff);

//	R_RSPI0_Start();
}

static void st_ParseGyro(void)
{
	uint16_t temp;

	temp = st_Rx.Buff & 0xFFFF;
	st_Rx.Data = temp;
}
