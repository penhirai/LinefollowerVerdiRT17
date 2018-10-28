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
#define PERIOD_INTERRUPT		(1.0 * 0.001)				// 制御周期 [s]
#define PERIOD_INTERRUPT_INV	(1.0 / PERIOD_INTERRUPT)	//  逆数
#define GEAR_RATIO				(5.0)						// 65 / 13
#define GEAR_RATIO_INV			(1.0 / GEAR_RATIO)			//  逆数
#define D_TIRE					(21.8 * 0.001)				// タイヤ直径 [m]
#define ENCODER_PULSE_MAX		(4096.0)					// cnt/回転
#define ENCODER_PULSE_MAX_INV	(1.0 / ENCODER_PULSE_MAX)	//  逆数

// 角速度系
#define K_GYRO					(-1.0 / 16.4)				// SOL 側実装のため，符号マイナス[(deg/s)/LSB]
#define K_ANGULAR_VELOCITY		(PI / 180.0)				// [deg/s] -> [rad/s]
#define K_ANGULAR_VELOCITY_INV	(1.0 / K_ANGULAR_VELOCITY)	// [rad/s] -> [deg/s]
#define LENGTH_SENSOR			(65.0 * 0.001)				// [m]


#endif /* SETTING_H_ */
