/*
 * common.h
 *
 *  Created on: 2017年6月14日
 *      Author: chen
 */

#ifndef COMMON_H_
#define COMMON_H_

/* XD5 Devices include. */
#include "blackbox.h"
#include "camera.h"
#include "hmi.h"
#include "imu.h"
#include "led.h"
#include "motor.h"
#include "oled.h"
#include "sbus.h"

/* XD5 Others include. */
#include "control.h"
#include "hmiHandle.h"
#include "imgprocess.h"

/* PID参数信息全局传递 */
extern carAngle_t Angle;            //PID控制器 - 平衡环参数
extern carSpeed_t Speed;            //PID控制器 - 速度环参数
extern carDirection_t Direction;    //PID控制器 - 转向环参数
extern carMotor_t Motor;            //PID控制器 - 电机输出参数
extern carFailsafe_t Failsafe;      //PID控制器 - 失控保护参数

/* 图像参数全局传递 */
extern imgProcess_t Process;
extern imgBrake_t Brake;
extern imgBarrier_t Barrier;
extern imgCircle_t Circle;
extern imgCross_t Cross;
extern imgSlope_t Slope;

extern float normpdf35[60];
extern float normpdf37[60];
extern float normpdf40[60];
extern float normpdf43[60];
extern float normpdf45[60];
extern float normpdf47[60];
extern float normpdf50[60];

/* 配置信息全局传递 */
extern camConf_t camera;
extern imuConf_t imuBias;

/* 遥控通道信息全局传递 */
extern sbusChannel_t rcInfo;
/* 传感器信息全局传递 */
extern imuData_t sensor;

#endif /* COMMON_H_ */
