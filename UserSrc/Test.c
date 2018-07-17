/*
 * Test.c
 *
 *  Created on: 2018/07/01
 *      Author: hirai
 */

#include "Test.h"
#include "Switch.h"
#include "FunctionTimer.h"

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
	CASE_STRAIGHT_TEST,
	CASE_ROTATION_TEST,
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


static void st_SciTest(void);
static void st_IrqTest(void);
static void st_LedTest(void);
static void st_PwmTest(void);
static void st_BuzzerTest(void);
static void st_IntBuzzerTest(void);
static void st_EncoderTest(void);
static void st_SensorTest(void);
static void st_GyroTest(void);
static void st_StraightTest(void);
static void st_RotationTest(void);
static void st_LogTest(void);



void TST_TestMode(void)
{
	SWT_Init(SWT_RL_MIN, SWT_RL_MAX, MODE_LEVEL_MIN, MODE_LEVEL_MAX);
	st_ModeLevel = CASE_SCI_TEST;
	st_Decision = SWT_DECISION_FALSE;

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

	R_SCI2_Start();

	while(1)
	{
		sprintf(st_SendBuf, "test\r\n");

		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	R_SCI2_Stop();
}


static void st_IrqTest(void)
{
	st_Decision = SWT_DECISION_FALSE;

	R_SCI2_Start();

	while(1)
	{
		sprintf(st_SendBuf, "C:%d, R:%d, L:%d, U:%d, D:%d\r\n", st_Swt->Center, st_Swt->Right, st_Swt->Left, st_Swt->Up, st_Swt->Down);
		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	R_SCI2_Stop();
}


static void st_LedTest(void)
{
	EnmLedSwtKind ledKind;
	int32_t ledValue;

	st_Decision = SWT_DECISION_FALSE;

	R_SCI2_Start();

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

		sprintf(st_SendBuf, "kind:%d, value:%d \r\n", ledKind, ledValue);

		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	R_SCI2_Stop();
}


static void st_PwmTest(void)
{
	uint16_t rightMotorDuty;
	uint16_t leftMotorDuty;

	SWT_Init(0, 9, 0, 9);

	st_Decision = SWT_DECISION_FALSE;

	R_SCI2_Start();
	FTR_StartRightMotorTimer();
	FTR_StartLeftMotorTimer();

	while(1)
	{
		rightMotorDuty = 10 * st_Swt->RL_Dif;
		leftMotorDuty  = 10 * st_Swt->UD_Dif;

		FTR_SetRightMotorDuty(rightMotorDuty);
		FTR_SetLeftMotorDuty(leftMotorDuty);

		sprintf(st_SendBuf, "Right: %d, Left: %d \r\n", rightMotorDuty, leftMotorDuty);

		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	R_SCI2_Stop();
}


static void st_BuzzerTest(void)
{
	uint16_t beepCount;

	SWT_Init(0, 9, 0, 9);

	st_Decision = SWT_DECISION_FALSE;

	R_SCI2_Start();

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

		sprintf(st_SendBuf, "Count: %d \r\n", beepCount);

		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	R_SCI2_Stop();
}

static void st_IntBuzzerTest(void)
{
	uint16_t beepCount;

	SWT_Init(0, 9, 0, 9);

	st_Decision = SWT_DECISION_FALSE;

	R_SCI2_Start();

	while(1)
	{
		beepCount = st_Swt->UD_Dif;
		if(st_Swt->RL_Dif > 0)
		{
			BZR_SetBeepCount((uint8_t)beepCount);
		}

		sprintf(st_SendBuf, "Count: %d \r\n", beepCount);

		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	R_SCI2_Stop();
}


static void st_EncoderTest(void)
{
	uint16_t rightEncoder;
	uint16_t leftEncoder;

	st_Decision = SWT_DECISION_FALSE;

	rightEncoder = FTR_GetRightEncoderCount();
	leftEncoder = FTR_GetLeftEncoderCount();

	R_SCI2_Start();
	FTR_StartRightEncoderTimer();
	FTR_StartLeftEncoderTimer();

	while(1)
	{
		sprintf(st_SendBuf, "Right: %d, Left: %d \r\n", rightEncoder, leftEncoder);

		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	R_SCI2_Stop();
}


static void st_SensorTest(void)
{
	LOG_StrSensorData *sensor;

	st_Decision = SWT_DECISION_FALSE;

	R_SCI2_Start();
	SSR_Init();

	while(1)
	{
		sensor = SSR_GetSensor();

		sprintf(st_SendBuf, "L M:%d, L C:%d,R C:%d, R M:%d, Pow:%d, Pot:%d \r\n", sensor->LeftMarker, sensor->LeftCenter, sensor->RightCenter, sensor->RightMarker, sensor->Power, sensor->Potentio);

		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	R_SCI2_Stop();
}


static void st_GyroTest(void)
{
	st_Decision = SWT_DECISION_FALSE;

	R_SCI2_Start();

	while(1)
	{
		sprintf(st_SendBuf, "test\r\n");

		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	R_SCI2_Stop();
}


static void st_StraightTest(void)
{
	st_Decision = SWT_DECISION_FALSE;

	R_SCI2_Start();

	while(1)
	{
		sprintf(st_SendBuf, "test\r\n");

		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	R_SCI2_Stop();
}


static void st_RotationTest(void)
{
	st_Decision = SWT_DECISION_FALSE;

	R_SCI2_Start();

	while(1)
	{
		sprintf(st_SendBuf, "test\r\n");

		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	R_SCI2_Stop();
}


static void st_LogTest(void)
{
	st_Decision = SWT_DECISION_FALSE;

	R_SCI2_Start();

	while(1)
	{
		sprintf(st_SendBuf, "test\r\n");

		R_SCI2_Serial_Send(st_SendBuf, sizeof(st_SendBuf));

		st_Decision = SWT_GetCenterDecision();
		if(st_Decision == SWT_DECISION_TRUE)
		{
			break;
		}
	}

	R_SCI2_Stop();
}
