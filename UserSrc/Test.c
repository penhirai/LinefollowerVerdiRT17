/*
 * Test.c
 *
 *  Created on: 2018/07/01
 *      Author: hirai
 */

#include "Test.h"
#include "Switch.h"
#include "FunctionTimer.h"
#include "Log.h"
#include "Led.h"
#include "sensor.h"
#include "SciFifo.h"
#include "TaskTimer.h"
#include "ControlVelocity.h"
#include "Buzzer.h"
#include "DriveAssert.h"
#include "ControlAngularVelocity.h"
#include "ControlSensorAngle.h"

#define MODE_LEVEL_MIN 0

typedef enum enmModeTestLevel
{
	CASE_SCI_TEST = MODE_LEVEL_MIN,
	CASE_IRQ_TEST,
	CASE_LED_TEST,
	CASE_PWM_TEST,
	CASE_BUZZER_TEST,
	CASE_INT_BUZZER_TEST,
	CASE_ENCODER_TEST,
	CASE_SENSOR_TEST,
	CASE_GYRO_TEST,
	CASE_CALIB_SENSOR_TEST,
	CASE_MARKER_TEST,
	CASE_STRAIGHT_TEST,
	CASE_ROTATION_TEST,
	CASE_ASSERT_TEST,
	CASE_LOG_TEST,
	MODE_LEVEL_MAX
}EnmModeTestLevel;


typedef enum enmLedSwtKindTest
{
	LED_SWT_CENTER = 0,
	LED_SWT_RIGHT,
	LED_SWT_LEFT,
	LED_SWT_UP,
	LED_SWT_DOWN
}EnmLedSwtKind;


#define SEND_BUF_SIZE 20

static SWT_StrSwitch *st_Swt;
static EnmModeTestLevel st_ModeLevel;
static SWT_EnmDecision st_Decision;
static uint8_t st_SendBuf[SEND_BUF_SIZE];
static uint8_t st_BufSize;


static void st_SciTest(void);
static void st_IrqTest(void);
static void st_LedTest(void);
static void st_PwmTest(void);
static void st_BuzzerTest(void);
static void st_IntBuzzerTest(void);
static void st_EncoderTest(void);
static void st_SensorTest(void);
static void st_GyroTest(void);
static void st_CalibSensorTest(void);
static void st_MarkerTest(void);
static void st_StraightTest(void);
static void st_RotationTest(void);
static void st_AssertTest(void);
static void st_LogTest(void);



void TST_TestMode(void)
{
	SWT_Init(SWT_RL_MIN, SWT_RL_MAX, MODE_LEVEL_MIN, MODE_LEVEL_MAX);
	st_ModeLevel = CASE_SCI_TEST;
	st_Decision = SWT_GetCenterDecision();;
	st_Swt = SWT_GetSwitch();

	//for(volatile i = 0; i < 100; ++i) ;

	while(1)
	{
		st_ModeLevel = (EnmModeTestLevel)st_Swt->UD_Dif;
		st_Decision = SWT_GetCenterDecision();

		LED_binaryOn(st_ModeLevel);

		switch(st_ModeLevel)
		{
			case CASE_SCI_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_SciTest();
				}
				break;
			case CASE_IRQ_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_IrqTest();
				}
				break;
			case CASE_LED_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_LedTest();
				}
				break;
			case CASE_PWM_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_PwmTest();
				}
				break;
			case CASE_BUZZER_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_BuzzerTest();
				}
				break;
			case CASE_INT_BUZZER_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_IntBuzzerTest();
				}
				break;
			case CASE_ENCODER_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_EncoderTest();
				}
				break;
			case CASE_SENSOR_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_SensorTest();
				}
				break;
			case CASE_GYRO_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_GyroTest();
				}
				break;
			case CASE_CALIB_SENSOR_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_CalibSensorTest();
				}
				break;
			case CASE_MARKER_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_MarkerTest();
				}
				break;
			case CASE_STRAIGHT_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_StraightTest();
				}
				break;
			case CASE_ROTATION_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_RotationTest();
				}
				break;
			case CASE_ASSERT_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_AssertTest();
				}
				break;
			case CASE_LOG_TEST:
				if(st_Decision == SWT_DECISION_TRUE)
				{
					st_LogTest();
				}
				break;
			default:
				break;
		}
	}
}


static void st_SciTest(void)
{
	st_Decision = SWT_DECISION_FALSE;

	//R_SCI2_Start();

	while(1)
	{
		st_BufSize = sprintf(st_SendBuf, "test  \r\n");
		SCF_WriteData(st_SendBuf, st_BufSize);


		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	//R_SCI2_Stop();
}


static void st_IrqTest(void)
{
	st_Decision = SWT_DECISION_FALSE;

	//R_SCI2_Start();

	while(1)
	{
		st_BufSize = sprintf(st_SendBuf, "C:%d, R:%d, L:%d, U:%d, D:%d\r\n", st_Swt->Center, st_Swt->Right, st_Swt->Left, st_Swt->Up, st_Swt->Down);
		SCF_WriteData(st_SendBuf, st_BufSize);
		//R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	//R_SCI2_Stop();
}


static void st_LedTest(void)
{
	EnmLedSwtKind ledKind;
	int32_t ledValue;

	st_Decision = SWT_DECISION_FALSE;

	//R_SCI2_Start();

	while(1)
	{
		ledKind = (EnmLedSwtKind)st_Swt->RL_Dif;

		switch(ledKind)
		{
			case LED_SWT_CENTER:
				ledValue = st_Swt->Center;
				break;
			case LED_SWT_RIGHT:
				ledValue = st_Swt->Right;
				break;
			case LED_SWT_LEFT:
				ledValue = st_Swt->Left;
				break;
			case LED_SWT_UP:
				ledValue = st_Swt->Up;
				break;
			case LED_SWT_DOWN:
				ledValue = st_Swt->Down;
				break;
			default:
				ledValue = st_Swt->Center;
				break;
		}

		LED_binaryOn(ledValue);

		st_BufSize = sprintf(st_SendBuf, "kind:%d, value:%d \r\n", ledKind, ledValue);
		SCF_WriteData(st_SendBuf, st_BufSize);
		//R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	//R_SCI2_Stop();
}


static void st_PwmTest(void)
{
	uint16_t rightMotorDuty;
	uint16_t leftMotorDuty;
	uint16_t sensorMotorDuty;

	SWT_Init(0, 9, 0, 9);

	st_Decision = SWT_DECISION_FALSE;

	//R_SCI2_Start();
	FTR_StartRightMotorTimer();
	FTR_StartLeftMotorTimer();
	FTR_StartSensorMotorTimer();

	CVL_StartDriveMotor();

	while(1)
	{
		rightMotorDuty = 10 * st_Swt->RL_Dif;
		leftMotorDuty  = 10 * st_Swt->UD_Dif;
		sensorMotorDuty = 10;

		FTR_SetTransitionRightMotorDuty(rightMotorDuty);
		FTR_SetTransitionLeftMotorDuty(leftMotorDuty);
		FTR_SetSensorMotorDuty(sensorMotorDuty, 0.0);

		st_BufSize = sprintf(st_SendBuf, "Right: %d, Left: %d sensor: %d \r\n", rightMotorDuty, leftMotorDuty, sensorMotorDuty);
		SCF_WriteData(st_SendBuf, st_BufSize);
		//R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	//R_SCI2_Stop();
}


static void st_BuzzerTest(void)
{
	uint16_t beepCount;

	SWT_Init(0, 9, 0, 9);

	st_Decision = SWT_DECISION_FALSE;

	//R_SCI2_Start();

	while(1)
	{
		beepCount = st_Swt->UD_Dif;
		if((beepCount % 2) == 0)
		{
			BZR_BuzzerOn();
		}
		else
		{
			BZR_BuzzerOff();
		}

		st_BufSize = sprintf(st_SendBuf, "Count: %d \r\n", beepCount);
		SCF_WriteData(st_SendBuf, st_BufSize);
		//R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	//R_SCI2_Stop();
}

static void st_IntBuzzerTest(void)
{
	uint16_t beepCount;

	SWT_Init(0, 9, 0, 9);

	st_Decision = SWT_DECISION_FALSE;

	//R_SCI2_Start();

	while(1)
	{
		beepCount = st_Swt->UD_Dif;
		if(st_Swt->RL_Dif > 0)
		{
			BZR_SetBeepCount((uint8_t)beepCount);
			st_Swt->RL_Dif = 0;
		}

		st_BufSize = sprintf(st_SendBuf, "Count: %d \r\n", beepCount);
		SCF_WriteData(st_SendBuf, st_BufSize);
		//R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	//R_SCI2_Stop();
}


static void st_EncoderTest(void)
{
	uint16_t rightEncoder;
	uint16_t leftEncoder;

	st_Decision = SWT_DECISION_FALSE;

	//R_SCI2_Start();
	FTR_StartRightEncoderTimer();
	FTR_StartLeftEncoderTimer();

	while(1)
	{
		rightEncoder = FTR_GetRightEncoderCount();
		leftEncoder = FTR_GetLeftEncoderCount();

		st_BufSize = sprintf(st_SendBuf, "Right: %d, Left: %d \r\n", rightEncoder, leftEncoder);
		//SCF_WriteData(st_SendBuf, st_BufSize);

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	//R_SCI2_Stop();
}


static void st_SensorTest(void)
{
	SSR_StrSensorData *sensor;

	st_Decision = SWT_DECISION_FALSE;

	//R_SCI2_Start();
	SSR_Init();

	while(1)
	{
		SSR_TaskGetAnalogSensor();

		SSR_PrintAllSensor();

		/*
		sensor = SSR_GetAnalogSensor();

		sprintf(st_SendBuf, "L M:%d, L C:%d,R C:%d, R M:%d, Pow:%d, Pot:%d \r\n", sensor->LeftMarker, sensor->LeftCenter, sensor->RightCenter, sensor->RightMarker, sensor->Power, sensor->Potentio);

		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));
		 */

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	//R_SCI2_Stop();
}


static void st_GyroTest(void)
{
	SSR_StrSensorData *sensor;
	uint32_t length;

	st_Decision = SWT_DECISION_FALSE;

	//R_SCI2_Start();
	//SSR_Init();

	while(1)
	{
		//LED_On(LED_2);
		//sensor = SSR_TaskCalcSensor();
		//SSR_TaskCalcSensor();
		//LED_Off(LED_2);

		SSR_PrintAllSensor();

		//length = sprintf(st_SendBuf, "test\r\n");

//		R_SCI2_Stop();
//		R_SCI2_Start();
	//	R_SCI2_Serial_Send(st_SendBuf, length);


		//LED_On(LED_1);
		//SSR_TaskStartReadGyro();
		//LED_Off(LED_1);

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	//R_SCI2_Stop();
}


static void st_CalibSensorTest(void)
{
	//st_Decision = SWT_DECISION_FALSE;

	//R_SCI2_Start();
	//SSR_Init();

	SSR_CalibSensor();

	/*
	while(1)
	{
		SSR_CalibSensor();


		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}

	}
	*/
}


static void st_MarkerTest(void)
{
	SSR_EnmMarkerState leftMarker;
	SSR_EnmMarkerState rightMarker;
	SSR_EnmMarkerKind kind;
	SSR_StrMarkerData markerData;

	st_Decision = SWT_DECISION_FALSE;

	SSR_CalibSensor();

	CVL_Init();
	TSK_Start(TSK_TASK3_CONTROL_VELOCITY);

	TSK_Start(TSK_TASK5_Judge_MARKER);

	while(1)
	{
		kind = SSR_LEFT_MARKER;
		leftMarker = SSR_GetMarkerState(kind);
		kind = SSR_RIGHT_MARKER;
		rightMarker = SSR_GetMarkerState(kind);

		markerData = SSR_GetCourceMarker();

		st_BufSize = sprintf(st_SendBuf, "L:%d, R:%d, c:%d, d:%f \r\n", leftMarker, rightMarker, markerData.MarkerKind, markerData.Distance);
		SCF_WriteData(st_SendBuf, st_BufSize);

		//SSR_PrintAllSensor();

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			//break;
		}
	}
}


static void st_StraightTest(void)
{
	float32_t target;
	float32_t velocity;
	float32_t leftDuty;
	float32_t rightDuty;
	float32_t errorNow;

	st_Decision = SWT_GetCenterDecision();

	CVL_StartDriveMotor();

	CVL_Init();
	TSK_Start(TSK_TASK3_CONTROL_VELOCITY);

	//R_SCI2_Start();

	CVL_SetTarget(3.5);

	while(1)
	{
		target = CVL_GetTarget();
		velocity = CVL_GetVelocity();
		leftDuty  = FTR_GetLeftMotorDuty();
		rightDuty = FTR_GetRightMotorDuty();
		errorNow = CVL_GetErrorNow();

		st_BufSize = sprintf(st_SendBuf, "tgt:%.3f, vec:%.3f, left:%.3f, right:%.3f \r\n", target, velocity, leftDuty, rightDuty);
		//st_BufSize = sprintf(st_SendBuf, "vec:%.3f, error:%.3f \r\n", velocity, errorNow);
		SCF_WriteData(st_SendBuf, st_BufSize);

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	//R_SCI2_Stop();
}


static void st_RotationTest(void)
{
	float32_t velocity;
	float32_t leftDuty;
	float32_t rightDuty;
	float32_t errorNow;
	float32_t radius;

	st_Decision = SWT_DECISION_FALSE;

	CVL_StartDriveMotor();

	CVL_Init();
	CAV_Init();
	TSK_Start(TSK_TASK3_CONTROL_VELOCITY);
	TSK_Start(TSK_TASK4_CONTROL_ANGULAR);

	CSA_StartSensorTask();

	CVL_SetTarget(0.5);
	//CAV_SetTarget(20.0);

	while(1)
	{
		velocity = CAV_GetVelocity();
		leftDuty  = FTR_GetLeftMotorDuty();
		rightDuty = FTR_GetRightMotorDuty();
		errorNow = CAV_GetErrorNow();
		radius = CAV_GetRadius();

		st_BufSize = sprintf(st_SendBuf, "vec:%.3f, left:%.3f, right:%.3f rad:%.3f\r\n", velocity, leftDuty, rightDuty, radius);
		SCF_WriteData(st_SendBuf, st_BufSize);

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	//R_SCI2_Stop();
}


static void st_AssertTest(void)
{
	DAS_EnmAssertFlag assertFlag;
	int32_t rlFlag;

	st_Decision = SWT_DECISION_FALSE;

	TSK_Start(TSK_TASKEACH1_ASSERT);
	DAS_Init();

	while(1)
	{
		assertFlag = DAS_GetAssertFlag();

		rlFlag = st_Swt->RL_Dif;
		if(rlFlag != 0)
		{
			DAS_ClearAssertFlag();
			while(rlFlag !=0 )
				rlFlag = st_Swt->RL_Dif;
		}

		st_BufSize = sprintf(st_SendBuf, "ass:%d \r\n", assertFlag);
		SCF_WriteData(st_SendBuf, st_BufSize);
		//R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}
}


static void st_LogTest(void)
{
	st_Decision = SWT_DECISION_FALSE;

	//R_SCI2_Start();

	while(1)
	{
		st_BufSize = sprintf(st_SendBuf, "test\r\n");
		SCF_WriteData(st_SendBuf, st_BufSize);
		//R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	//R_SCI2_Stop();
}
