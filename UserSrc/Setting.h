/*
 * Setting.h
 *
 *  Created on: 2018/10/18
 *      Author: shuuichi
 */

#ifndef SETTING_H_
#define SETTING_H_

// 主に制御系のdefine と構造体定義
#define PI						(3.1415926535)
#define PERIOD_INTERRUPT		(1.0 * 10e-3)				// 制御周期 [s]
#define PERIOD_INTERRUPT_INV	(1.0 / PERIOD_INTERRUPT)	//  逆数
#define GEAR_RATIO				(5.0)						// 65 / 13
#define D_TIRE					(21.8 * 10e-3 * 4.0)				// タイヤ直径 [m] なぜか4.0倍？
#define ENCODER_PULSE_MAX		(4096.0)					// cnt/回転
#define ENCODER_PULSE_MAX_INV	(1.0 / ENCODER_PULSE_MAX)	//  逆数

// 角速度系
#define K_ANGULAR_VELOCITY		(2000.0)					// [deg/s]
#define LENGTH_SENSOR			(65.0 * 10e-3)				// [m]


#endif /* SETTING_H_ */
