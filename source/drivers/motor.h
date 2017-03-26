/*
 * motor.h
 *
 *  Created on: 2017年1月26日
 *      Author: chen
 */

#ifndef SOURCE_DRIVERS_MOTOR_H_
#define SOURCE_DRIVERS_MOTOR_H_

#include <stdint.h>

/** 电机信息结构体 */
typedef struct
{
    int8_t pwmL;
    int8_t pwmR;

    int16_t cntL;
    int16_t cntR;
    int16_t cntAvg;

} mSpeed_t;

/** 公共调用函数 */
void Motor_Config(void);

void Motor_ChangeDuty(mSpeed_t speedInfo);
void Motor_GetCnt(mSpeed_t *speedInfo);

#endif /* SOURCE_DRIVERS_MOTOR_H_ */
