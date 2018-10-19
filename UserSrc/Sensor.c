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
#include "Switch.h"

#define SEND_BUF_SIZE 20

#define LINESENSOR_BUFF_SIZE 3
#define SENSOR_BUFF_SIZE 10
#define GYRO_BUFF_SIZE 1

#define LINESENSOR_MAX_DEFAULT 4000
#define LINESENSOR_MIN_DEFAULT 0

#define NOT_SENSOR_CALIB 0
#define SENSOR_CALIB_DONE 1

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

typedef struct strLineSensorCoefficient
{
	float32_t LeftMarker;
	float32_t LeftCenter;
	float32_t RightCenter;
	float32_t RightMarker;
}StrLineSensorCoefficient;

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


#define MARKER_HIGH_THRESHOLD 800
#define MARKER_LOW_THRESHOLD 500

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
static StrLineSensorCoefficient st_LineCoefficient;
static StrLineSensor st_InstantMax;
static StrLineSensor st_InstantMin;
static int16_t st_SensorCalibMax;
static int16_t st_SensorCalibMin;
static uint8_t st_SensorCalibFlag;

static void st_SetSensorGateOn(void);
static void st_SetSensorGateOff(void);
static void st_ReadLineSensor(EnmLine kind);
static void st_ReadOtherSensor(void);
static void st_ReadAllAnalog(void);
static void st_ReadDarkSensor(void);
static void st_ReadLightSensor(void);
static void st_CalcLineSensor(void);
static SSR_EnmMarkerState st_CalcMarkerSensor(SSR_EnmMarkerState nowState, int16_t markerData);

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

	st_SensorData.MarkerState.Left  = LOW_STATE;
	st_SensorData.MarkerState.Right = LOW_STATE;


	st_SensorCalibMax = LINESENSOR_MAX_DEFAULT;
	st_SensorCalibMin = LINESENSOR_MIN_DEFAULT;

	st_InstantMax.LeftMarker  = st_SensorCalibMin;
	st_InstantMax.LeftCenter  = st_SensorCalibMin;
	st_InstantMax.RightCenter = st_SensorCalibMin;
	st_InstantMax.RightMarker = st_SensorCalibMin;

	st_InstantMin.LeftMarker  = st_SensorCalibMax;
	st_InstantMin.LeftCenter  = st_SensorCalibMax;
	st_InstantMin.RightCenter = st_SensorCalibMax;
	st_InstantMin.RightMarker = st_SensorCalibMax;

	st_LineCoefficient.LeftMarker  = 2.0;
	st_LineCoefficient.LeftCenter  = 2.0;
	st_LineCoefficient.RightCenter = 2.0;
	st_LineCoefficient.RightMarker = 2.0;

	st_SensorCalibFlag = NOT_SENSOR_CALIB;

	st_SensorData.SensorTheta = 0.0;
	st_SensorData.BodyOmega = 0.0;

	st_InitGyro();

	TSK_Start(TSK_TASKEACH0_SENSOR);
	TSK_Start(TSK_TASK1_SENSOR_FILTER);
}


void SSR_TaskStartSensorGate(void)
{
	st_SetSensorGateOn();
}

void SSR_TaskStopSensorGate(void)
{
	st_SetSensorGateOff();
}


void SSR_TaskGetAnalogSensor(void)
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
	EnmLine kind = CALC;

//	R_S12AD0_Start();
//	R_S12AD0_WaitAdcEnd();
//
//	st_ReadAllAnalog();

	st_ReadLightSensor();
	st_CalcLineSensor();

	st_ParseGyro();
	st_SensorData.ArrayTemp[st_SensorData.Index].Gyro = st_Rx.Data;//st_Rx.Context.Data;

	// input data to array
	st_SensorData.ArrayTemp[st_SensorData.Index].LeftMarker  = st_LineSensor[kind].LeftMarker;
	st_SensorData.ArrayTemp[st_SensorData.Index].LeftCenter  = st_LineSensor[kind].LeftCenter;
	st_SensorData.ArrayTemp[st_SensorData.Index].RightCenter = st_LineSensor[kind].RightCenter;
	st_SensorData.ArrayTemp[st_SensorData.Index].RightMarker = st_LineSensor[kind].RightMarker;

	/*
	// execute filter
	st_SensorData.Result.LeftMarker  = st_SensorData.ArrayTemp[st_SensorData.Index].LeftMarker;
	st_SensorData.Result.LeftCenter  = st_SensorData.ArrayTemp[st_SensorData.Index].LeftCenter;
	st_SensorData.Result.RightCenter = st_SensorData.ArrayTemp[st_SensorData.Index].RightCenter;
	st_SensorData.Result.RightMarker = st_SensorData.ArrayTemp[st_SensorData.Index].RightMarker;
	st_SensorData.Result.Potentio    = st_SensorData.ArrayTemp[st_SensorData.Index].Potentio;
	st_SensorData.Result.Power       = st_SensorData.ArrayTemp[st_SensorData.Index].Power;
	st_SensorData.Result.Gyro        = st_SensorData.ArrayTemp[st_SensorData.Index].Gyro;
	*/

	++st_SensorData.Index;
	st_SensorData.Index %= SSR_SENSOR_BUFF_SIZE;



//	return &st_SensorData;
}

void SSR_TaskCalcFilter(void)
{
	// execute filter
	st_SensorData.Result.LeftMarker  = st_SensorData.ArrayTemp[st_SensorData.Index].LeftMarker;
	st_SensorData.Result.LeftCenter  = st_SensorData.ArrayTemp[st_SensorData.Index].LeftCenter;
	st_SensorData.Result.RightCenter = st_SensorData.ArrayTemp[st_SensorData.Index].RightCenter;
	st_SensorData.Result.RightMarker = st_SensorData.ArrayTemp[st_SensorData.Index].RightMarker;
	st_SensorData.Result.Potentio    = st_SensorData.ArrayTemp[st_SensorData.Index].Potentio;
	st_SensorData.Result.Power       = st_SensorData.ArrayTemp[st_SensorData.Index].Power;
	st_SensorData.Result.Gyro        = st_SensorData.ArrayTemp[st_SensorData.Index].Gyro;

	if(st_SensorCalibFlag == SENSOR_CALIB_DONE)
	{
		st_SensorData.Result.LeftMarker = (int16_t)(st_LineCoefficient.LeftMarker * (float32_t)(st_SensorData.Result.LeftMarker - st_InstantMin.LeftMarker));
	}

	if(st_SensorCalibFlag == SENSOR_CALIB_DONE)
	{
		st_SensorData.Result.LeftCenter = (int16_t)(st_LineCoefficient.LeftCenter * (float32_t)(st_SensorData.Result.LeftCenter - st_InstantMin.LeftCenter));
	}

	if(st_SensorCalibFlag == SENSOR_CALIB_DONE)
	{
		st_SensorData.Result.RightCenter = (int16_t)(st_LineCoefficient.RightCenter * (float32_t)(st_SensorData.Result.RightCenter - st_InstantMin.RightCenter));
	}

	if(st_SensorCalibFlag == SENSOR_CALIB_DONE)
	{
		st_SensorData.Result.RightMarker = (int16_t)(st_LineCoefficient.RightMarker * (float32_t)(st_SensorData.Result.RightMarker - st_InstantMin.RightMarker));
	}
}


void SSR_TaskJudgeMarkerSensor(void)
{
	st_SensorData.MarkerState.Left  = st_CalcMarkerSensor(st_SensorData.MarkerState.Left,  st_SensorData.Result.LeftMarker);
	st_SensorData.MarkerState.Right = st_CalcMarkerSensor(st_SensorData.MarkerState.Right, st_SensorData.Result.RightMarker);
}


void SSR_CalibSensor(void)
{
	SWT_EnmDecision decision = SWT_DECISION_FALSE;

	while(1)
	{
		// 左マーカーセンサ
		if(st_InstantMax.LeftMarker < st_SensorData.Result.LeftMarker)
		{
			st_InstantMax.LeftMarker = st_SensorData.Result.LeftMarker;
		}
		if(st_InstantMin.LeftMarker > st_SensorData.Result.LeftMarker)
		{
			st_InstantMin.LeftMarker = st_SensorData.Result.LeftMarker;
		}

		// 左ラインセンサ
		if(st_InstantMax.LeftCenter < st_SensorData.Result.LeftCenter)
		{
			st_InstantMax.LeftCenter = st_SensorData.Result.LeftCenter;
		}
		if(st_InstantMin.LeftCenter > st_SensorData.Result.LeftCenter)
		{
			st_InstantMin.LeftCenter = st_SensorData.Result.LeftCenter;
		}

		// 右ラインセンサ
		if(st_InstantMax.RightCenter < st_SensorData.Result.RightCenter)
		{
			st_InstantMax.RightCenter = st_SensorData.Result.RightCenter;
		}
		if(st_InstantMin.RightCenter > st_SensorData.Result.RightCenter)
		{
			st_InstantMin.RightCenter = st_SensorData.Result.RightCenter;
		}

		// 右マーカーセンサ
		if(st_InstantMax.RightMarker < st_SensorData.Result.RightMarker)
		{
			st_InstantMax.RightMarker = st_SensorData.Result.RightMarker;
		}
		if(st_InstantMin.RightMarker > st_SensorData.Result.RightMarker)
		{
			st_InstantMin.RightMarker = st_SensorData.Result.RightMarker;
		}

		// 校正モード終了
		decision = SWT_GetCenterDecision();
		if(decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	st_LineCoefficient.LeftMarker  = ((float32_t)st_SensorCalibMax - (float32_t)st_SensorCalibMin)
									/ ((float32_t)st_InstantMax.LeftMarker - (float32_t)st_InstantMin.LeftMarker);
	st_LineCoefficient.LeftCenter  = ((float32_t)st_SensorCalibMax - (float32_t)st_SensorCalibMin)
										/ ((float32_t)st_InstantMax.LeftCenter - (float32_t)st_InstantMin.LeftCenter);
	st_LineCoefficient.RightCenter = ((float32_t)st_SensorCalibMax - (float32_t)st_SensorCalibMin)
										/ ((float32_t)st_InstantMax.RightCenter - (float32_t)st_InstantMin.RightCenter);
	st_LineCoefficient.RightMarker = ((float32_t)st_SensorCalibMax - (float32_t)st_SensorCalibMin)
										/ ((float32_t)st_InstantMax.RightMarker - (float32_t)st_InstantMin.RightMarker);
	st_SensorCalibFlag = SENSOR_CALIB_DONE;
}


/*
SSR_StrSensorData *SSR_GetSensorStructure(void)
{
	return &st_SensorData.Result;
}
*/


SSR_StrSensorData SSR_GetSensorData(void)
{
	return st_SensorData.Result;
}

/*
float32_t *SSR_GetPotentioData(void)
{
	return &st_SensorData.SensorTheta;
}


float32_t *SSR_GetGyroData(void)
{
	return &st_SensorData.BodyOmega;
}
*/


float32_t SSR_GetPotentioData(void)
{
	return st_SensorData.SensorTheta;
}


void SSR_SetPotentioData(float32_t sensorTheta)
{
	st_SensorData.SensorTheta = sensorTheta;
}

float32_t SSR_GetGyroData(void)
{
	return st_SensorData.BodyOmega;
}

void SSR_SetGyroData(float32_t bodyOmega)
{
	st_SensorData.BodyOmega = bodyOmega;
}


void SSR_PrintAllSensor(void)
{
	uint8_t buffSize;

	buffSize = sprintf(st_SendBuf, "L M:%d, L C:%d,R C:%d, R M:%d, Pow:%d, Pot:%d, Gyro:%d \r\n", st_SensorData.Result.LeftMarker, st_SensorData.Result.LeftCenter, st_SensorData.Result.RightCenter, st_SensorData.Result.RightMarker, st_SensorData.Result.Power, st_SensorData.Result.Potentio, st_SensorData.Result.Gyro);

	SCF_WriteData(st_SendBuf, buffSize);
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
	R_S12AD0_Get_ValueResult(adc, &st_LineSensor[kind].RightCenter);

	adc = ADCHANNEL5;
	R_S12AD0_Get_ValueResult(adc, &st_LineSensor[kind].LeftCenter);
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
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.ArrayTemp[st_SensorData.Index].RightCenter);

	adc = ADCHANNEL4;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.ArrayTemp[st_SensorData.Index].Potentio);

	adc = ADCHANNEL5;
	R_S12AD0_Get_ValueResult(adc, &st_SensorData.ArrayTemp[st_SensorData.Index].LeftCenter);
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

	st_LineSensor[calc].LeftMarker  = st_LineSensor[light].LeftMarker;//  - st_LineSensor[dark].LeftMarker;
	st_LineSensor[calc].LeftCenter  = st_LineSensor[light].LeftCenter;//  - st_LineSensor[dark].LeftCenter;
	st_LineSensor[calc].RightCenter = st_LineSensor[light].RightCenter;// - st_LineSensor[dark].RightCenter;
	st_LineSensor[calc].RightMarker = st_LineSensor[light].RightMarker;// - st_LineSensor[dark].RightMarker;
}


static SSR_EnmMarkerState st_CalcMarkerSensor(SSR_EnmMarkerState nowState, int16_t markerData)
{
	SSR_EnmMarkerState lowState = LOW_STATE;
	SSR_EnmMarkerState HighState = HIGH_STATE;
	SSR_EnmMarkerState ret;

	// ヒステリシス計算
	if(nowState == lowState)
	{
		if(markerData > MARKER_HIGH_THRESHOLD)
		{
			ret = HIGH_STATE;
		}
	}
	else if(nowState == HighState)
	{
		if(markerData < MARKER_LOW_THRESHOLD)
		{
			ret = LOW_STATE;
		}
	}

	return ret;
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
