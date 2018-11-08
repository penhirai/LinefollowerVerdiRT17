/*
 * ControlSensorAngle.c
 *
 *  Created on: 2018/10/14
 *      Author: shuuichi
 */

#include "ControlSensorAngle.h"
#include "Sensor.h"
#include "ControlStructure.h"
#include "FunctionTimer.h"
#include <r_cg_port.h>
#include "TaskTimer.h"

#define POTENTIO_CENTER (2020.0)
#define K_ANGLE			(90.0 / 1120.0)		// 90°:1120cnt 程度

typedef struct strTheta
{
	float32_t Target;
	float32_t Delta;
	float32_t Value;
	float32_t PastValue;
	float32_t Offset;
	CST_StrError Error;
	CST_StrGain Gain;
	float32_t DutyOffset;
	float32_t k_pot_to_theta;
	float32_t k_sensor_to_pot;
}StrTheta;

//static SSR_StrSensorData *st_SensorData;
static SSR_StrSensorData st_SensorData;
static StrTheta st_Theta;
//static float32_t *st_PotentioTheta;
static float32_t st_PotentioTheta;

static LOG_StrControlSensorHeader st_LogHeader;
static LOG_StrControlSensorArray  st_LogArray;


static void st_CalcSensorAngle(int16_t potentio);

static void st_DriveSensorMotor(float32_t input, float32_t offset);
//static void st_SetDriveCw(void);
//static void st_SetDriveCcw(void);

void CSA_Init(void)
{
	st_Theta.Target = POTENTIO_CENTER;
	st_Theta.Delta  = 0.0;
	st_Theta.Value  = 0.0;
	st_Theta.PastValue = 0.0;
	st_Theta.Offset = 0.0;

	st_Theta.Error.Sum  = 0.0;
	st_Theta.Error.Now  = 0.0;
	st_Theta.Error.Past = 0.0;
	st_Theta.Error.Factor.FF = 0.0;
	st_Theta.Error.Factor.P  = 0.0;
	st_Theta.Error.Factor.I  = 0.0;
	st_Theta.Error.Factor.D  = 0.0;

	st_Theta.Gain.Scale = 1.0;
	st_Theta.Gain.Factor.FF = 0.0;
	st_Theta.Gain.Factor.P  = 4.0;
	st_Theta.Gain.Factor.I  = 0.0;
	st_Theta.Gain.Factor.D  = 0.0;

	st_Theta.DutyOffset = 12.0;

	st_Theta.k_pot_to_theta = 1.0;//0.06597; // theta / potentio
	st_Theta.k_sensor_to_pot = 0.005; // 200 / 2800

	//st_SensorData = SSR_GetSensorStructure();
	//st_PotentioTheta = SSR_GetPotentioData();
}


void CSA_StartSensorMotor(void)
{
	R_PORT_EnmPort state = R_PORT_HIGH;

	R_PORT_SetPB4(state);
}


void CSA_StopSensorMotor(void)
{
	R_PORT_EnmPort state = R_PORT_LOW;

	R_PORT_SetPB4(state);
}

void CSA_StartSensorTask(void)
{
	TSK_Start(TSK_TASK2_CONTROL_SENSOR);
	FTR_StartSensorMotorTimer();
}


void CSA_ControlSensorTask(void)
{
	st_SensorData = SSR_GetSensorData();

	// ポテンショ角度計算
	// 念のため，角度情報はセンサ構造体に戻しておく
	st_CalcSensorAngle(st_SensorData.Potentio);
	st_Theta.Value = st_PotentioTheta;

	// センサ値からポテンショ角度計算
	st_Theta.Delta = st_Theta.k_sensor_to_pot * (st_SensorData.LeftCenter - st_SensorData.RightCenter);
	st_Theta.Delta *= st_Theta.k_pot_to_theta;

	// 目標角度計算
	st_Theta.Target = st_Theta.PastValue + st_Theta.Delta + st_Theta.Offset;

	// 偏差計算
	st_Theta.Error.Factor.P = st_Theta.Target - st_Theta.Value;

	// 偏差を合計
	st_Theta.Error.Sum = st_Theta.Gain.Factor.P * st_Theta.Error.Factor.P;

	//
	st_DriveSensorMotor(st_Theta.Error.Sum, st_Theta.DutyOffset);

	//
	SSR_SetPotentioData(st_PotentioTheta);

	// 現在のポテンショ値を記録
	st_Theta.PastValue = st_Theta.Value;
}


float32_t CSA_GetSensorTheta(void)
{
	float32_t thetaTemp;

	thetaTemp = POTENTIO_CENTER - st_Theta.Target;
	thetaTemp *= K_ANGLE;
	return thetaTemp;
}


LOG_StrControlSensorHeader *CSA_GetLogHeader(void)
{
	st_LogHeader.Gain = st_Theta.Gain;

	return &st_LogHeader;
}


LOG_StrControlSensorArray  *CSA_GetLogArray(void)
{
	float32_t angle;

	angle = CSA_GetSensorTheta();

	st_LogArray.LeftLineSensor  = st_SensorData.LeftCenter;
	st_LogArray.RightLineSensor = st_SensorData.RightCenter;
	st_LogArray.DiffLineSensor  = st_Theta.Delta;
	st_LogArray.Target          = st_Theta.Target;
	st_LogArray.SensorAngle     = angle;
	st_LogArray.ErrorP   = st_Theta.Error.Factor.P;
	st_LogArray.ErrorSum = st_Theta.Error.Sum;

	return &st_LogArray;
}


static void st_CalcSensorAngle(int16_t potentio)
{
	st_PotentioTheta = st_Theta.k_pot_to_theta * (float32_t)potentio;
}


static void st_DriveSensorMotor(float32_t input, float32_t offset)
{
	FTR_SetSensorMotorDuty(input, offset);
}
