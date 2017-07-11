/*
 * control.c
 *
 *  Created on: 2017年3月19日
 *      Author: chen
 */


/* Includes ------------------------------------------------------------------*/
#include "control.h"

/* Definitions ---------------------------------------------------------------*/
#define PIT_PERIOD_CONSTANT    (5000U)    //PIT定时器触发（5ms）
#define SPEED_PERIOD_CONSTANT  (20U)      //速度控制周期（20 * 5 = 100ms）

/* Variables -----------------------------------------------------------------*/
carAngle_t Angle;
carSpeed_t Speed;
carDirection_t Direction;
carMotor_t Motor;
carFailsafe_t Failsafe;

/* Codes ---------------------------------------------------------------------*/

/**
  * @brief  控制器初始化
  * @retval none
  */
void PidControllor_Init(void)
{
    /* 开启控制器 */
    Angle.ON = true;
    Speed.ON = true;
    Direction.ON = true;

    /* 默认PID参数设置 */
    Motor.Dead_L = 2.f;
    Motor.Dead_R = 2.4f;

    Angle.A_Bias = -16.5f;
    Angle.G_Bias = 0.f;
    Angle.P = 9.0f;
    Angle.D = 0.028f;

    Speed.Goal = 0.f;
    Speed.P = 0.05f;
    Speed.I = 0.0075f;
    Speed.I_Error_Start = 2500;
    Speed.I_Limit_PWM_max = 35;
    Speed.I_Limit_PWM_min = -35;
    Speed.PWM_Integral = 0.f;

    Direction.G_Bias = 0.f;
    Direction.P = 0.95f;
    Direction.D = 0.055f;

    /* 默认失控保护参数 */
    Failsafe.ON = true;
    Failsafe.Pitch_Max = 25.f;
    Failsafe.Pitch_Min = -30.f;
}

/**
  * @brief  直立环控制器（建议周期：5ms）
  * @retval none
  */
void Angle_Control(float angle, float gyro)
{
    /* 直立环总开关 */
    if (Angle.ON == true)
    {
        Angle.PWM = (angle - Angle.A_Bias) * Angle.P
                + (gyro - Angle.G_Bias) * Angle.D;
    }
    else
    {
        Angle.PWM = 0;
    }
}

/**
  * @brief  速度环控制器（建议周期：100ms）
  * @retval none
  */
void Speed_Control(int16_t cntL, int16_t cntR)
{
    static uint8_t SpdPeriod = 0;

    static float Speed_Error = 0;
    static float Temp_Speed_P;
    static int Temp_L_100ms = 0, Temp_R_100ms = 0;

    /* 速度环总开关 */
    if (Speed.ON == true)
    {
        SpdPeriod++;
        Temp_L_100ms += cntL;
        Temp_R_100ms += cntR;
        if (SpdPeriod >= SPEED_PERIOD_CONSTANT)   //速度PID反馈调试，100ms一次计算
        {
            SpdPeriod = 0;
            Speed.L_100ms = Temp_L_100ms;
            Speed.R_100ms = Temp_R_100ms;
            Speed.Avg_100ms = (float) (Speed.L_100ms + Speed.R_100ms) / 2.f;
            Temp_L_100ms = 0;
            Temp_R_100ms = 0;

            /* 计算偏差量 */
            Speed_Error = Speed.Goal - Speed.Avg_100ms;

            /* I积分 */
            Speed.PWM_Integral += Speed_Error * Speed.I;
            /* PWM_Integral 限幅 */
            if (Speed.PWM_Integral > Speed.I_Limit_PWM_max)
                Speed.PWM_Integral = Speed.I_Limit_PWM_max;
            else if (Speed.PWM_Integral <= Speed.I_Limit_PWM_min)
                Speed.PWM_Integral = Speed.I_Limit_PWM_min;

            /* P限幅，防止车身过度倾斜 */
            if (Speed_Error > Speed.I_Error_Start)
                Temp_Speed_P = Speed.P * Speed.I_Error_Start;
            else if (Speed_Error < -Speed.I_Error_Start)
                Temp_Speed_P = Speed.P * -Speed.I_Error_Start;
            else
                Temp_Speed_P = Speed.P * Speed_Error;

            /* 平滑输出 */
            Speed.PWM_Per = (Temp_Speed_P + Speed.PWM_Integral - Speed.PWM)
                    / SPEED_PERIOD_CONSTANT;
        }

        Speed.PWM += Speed.PWM_Per;
    }
    else
    {
        SpdPeriod = 0;
        Temp_L_100ms = 0;
        Temp_R_100ms = 0;
        Speed.PWM_Integral = 0;
        Speed.PWM_Per = 0;
        Speed.PWM = 0;
    }
}

void Direction_Control(float turn, float offset, float gyro)
{
    /* 速度环总开关 */
    if (Direction.ON == true)
    {
        /* 增量式PD，转向值 + 中线偏移值 */
        Direction.PWM_Per = (turn + offset) * Direction.P
                + (gyro - Direction.G_Bias) * Direction.D - Direction.PWM;

        Direction.PWM += Direction.PWM_Per;
    }
    else
    {
        Direction.PWM_Per = 0;
        Direction.PWM = 0;
    }
}

/*
 * 电机pwm输出    //  10   15   20   25   30   35   40   45   50
 * 测得的速度      //2400 3000 4200 ---- ---- ---- ---- ---- ----
 */
void Motor_Control(int8_t *pwmL, int8_t *pwmR)
{
    /* 计算总输出 */
    Motor.Final_PWM_L = Angle.PWM - Speed.PWM + Direction.PWM;
    Motor.Final_PWM_R = Angle.PWM - Speed.PWM - Direction.PWM;

    /* 计算死区和限幅 */
    if (Motor.Final_PWM_L >= 0)
    {
        Motor.Final_PWM_L += Motor.Dead_L;   //加死区
        if (Motor.Final_PWM_L > 95)          //限幅
            Motor.Final_PWM_L = 95;
    }
    else
    {
        Motor.Final_PWM_L -= Motor.Dead_L;   //加死区
        if (Motor.Final_PWM_L < -95)         //限幅
            Motor.Final_PWM_L = -95;
    }

    if (Motor.Final_PWM_R >= 0)
    {
        Motor.Final_PWM_R += Motor.Dead_R;   //加死区
        if (Motor.Final_PWM_R >= 95)         //限幅
            Motor.Final_PWM_R = 95;
    }
    else
    {
        Motor.Final_PWM_R -= Motor.Dead_R;   //加死区
        if (Motor.Final_PWM_R <= -95)        //限幅
            Motor.Final_PWM_R = -95;
    }

    /* 输出最终PWM */
    *pwmL = (int8_t) Motor.Final_PWM_L;
    *pwmR = (int8_t) Motor.Final_PWM_R;
}

/**
  * @brief  失控保护控制器
  * @param  debug = 1时为调试模式，重启失控保护不会复位控制器
  * @retval none
  */
void Failsafe_Control(float angle, int debug)
{
    /* 失控保护总开关 */
    if (Failsafe.ON == true)
    {
        if (Failsafe.Protected == true)
            return;

        if (angle < Failsafe.Pitch_Min || angle > Failsafe.Pitch_Max)
        {
            Failsafe.Protected = true;
            Angle.ON = false;
            Speed.ON = false;
            Direction.ON = false;
        }
    }
    else if (debug)
    {
        if (Failsafe.Protected == false)
            return;
        /* Debug模式下不复位任何控制器，方便调参 */
        Failsafe.Protected = false;
    }
    else
    {
        if (Failsafe.Protected == false)
            return;
        Failsafe.Protected = false;
        Angle.ON = true;
        Speed.ON = true;
        Direction.ON = true;
    }
}
