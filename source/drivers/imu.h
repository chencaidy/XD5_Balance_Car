/*
 * sensor.h
 *
 *  Created on: 2017年2月3日
 *      Author: chen
 */

#ifndef USERS_DRIVERS_IMU_H_
#define USERS_DRIVERS_IMU_H_

#include "fsl_common.h"

/** 姿态参数结构体 */
typedef struct
{
    /* 姿态角 */
    float Pitch;
    float Roll;
    float Yaw;

    /* 陀螺仪值 */
    float GyroX;
    float GyroY;
    float GyroZ;

    /* 加速度值 */
    float AccelX;
    float AccelY;
    float AccelZ;

} imuData_t;

/** 校准信息配置 */
typedef struct
{
    long accel_bias[3];
    long gyro_bias[3];

} imuConf_t;

/** 公共调用函数 */
status_t IMU_Config(void);

status_t IMU_Process(imuData_t *ptr);
void IMU_HandleInput(char cmd);
void IMU_IRQ_Handle(void);
void IMU_PID_SetCallback(void *func);
void IMU_ToggleLED(void);

status_t IMU_Calibrate(imuConf_t *bias);
void IMU_SetBias(imuConf_t bias);

void Sensor_GetTimeStamp(long unsigned int *count);
status_t Sensor_I2C_WriteRegister(uint8_t slave_addr, uint8_t reg_addr, uint16_t len, const uint8_t *data_ptr);
status_t Sensor_I2C_ReadRegister(uint8_t slave_addr, uint8_t reg_addr, uint16_t len, uint8_t *data_ptr);

#endif /* USERS_DRIVERS_IMU_H_ */
