/*
 * control.h
 *
 *  Created on: 2017年3月19日
 *      Author: chen
 */

#ifndef USERS_CONTROL_H_
#define USERS_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    /* 直立环开关 */
    bool ON;
    /* 角度和陀螺仪偏差修正  */
    float A_Bias;
    float G_Bias;
    /* 平衡部分PD值 */
    float P;
    float D;

    float PWM;            //直立环最终PWM

} carAngle_t;

typedef struct
{
    /* 速度环开关 */
    bool ON;
    /* 速度定义 */
    int L_100ms;
    int R_100ms;
    float Avg_100ms;
    float Goal;
    float Goal_Init;
    /* PI控制 */
    float P;              //速度控制P参数
    float I;              //速度控制I参数
    float I_Error_Start;
    float I_Limit_PWM_max;
    float I_Limit_PWM_min;

    float PWM_Integral;   //速度赋给PWM的值，存储的积分
    float PWM_Per;        //速度每次增量值
    float PWM;            //速度环最终PWM

} carSpeed_t;

typedef struct
{
    /* 角度环开关 */
    bool ON;
    /* 陀螺仪偏差修正  */
    float G_Bias;
    /* PD控制 */
    float P;              //控制P参数
    float D;              //控制D参数

    float PWM_Per;        //方向每次增量值
    float PWM;            //方向环最终PWM

} carDirection_t;

typedef struct
{
    /* 死区设置  */
    float Dead_L;
    float Dead_R;
    /* 最终PWM输出  */
    float Final_PWM_L;
    float Final_PWM_R;

} carMotor_t;

void PidControllor_Init(void);
void PidControllor_Process(void);

void Angle_Control(float angle, float gyro);
void Speed_Control(int16_t cntL, int16_t cntR);
void Direction_Control(float offset, float gyro);
void Motor_Control(int8_t *pwmL, int8_t *pwmR);

#endif /* USERS_CONTROL_H_ */
