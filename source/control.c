/*
 * control.c
 *
 *  Created on: 2017年3月19日
 *      Author: chen
 */


/* Includes ------------------------------------------------------------------*/
#include "control.h"

/* Definitions ---------------------------------------------------------------*/
#define SPEED_PERIOD_CONSTANT  (20U)      //速度控制周期

/* Variables -----------------------------------------------------------------*/
carAngle_t Angle;
carSpeed_t Speed;
carDirection_t Direction;
carMotor_t Motor;

/* Codes ---------------------------------------------------------------------*/
/**
  * @brief  直立环控制器（建议周期：5ms）
  * @retval none
  */
void Angle_Control(float angle, float gyro)
{
    Angle.PWM = (angle - Angle.Offset) * Angle.P + gyro * Angle.D;
}

/**
  * @brief  速度环控制器（建议周期：100ms）
  * @retval none
  */
void Speed_Control(int16_t cntL, int16_t cntR)
{
    static uint8_t Period = 0;      //速度控制周期变量

    static float Speed_Error = 0;
    static float Temp_Speed_P;
    static int Temp_L_100ms = 0, Temp_R_100ms = 0;

    Period++;
    Temp_L_100ms += cntL;
    Temp_R_100ms += cntR;
    if (Period >= SPEED_PERIOD_CONSTANT)   //速度PID反馈调试，100ms一次计算
    {
        Period = 0;
        Speed.L_100ms = Temp_L_100ms;
        Speed.R_100ms = Temp_R_100ms;
        Speed.Avg = (float) ((Speed.L_100ms + Speed.R_100ms) / 2.0);
        Temp_L_100ms = 0;
        Temp_R_100ms = 0;

        //速度控制
        Speed_Error = Speed.Goal - Speed.Avg;

        //IIIIIIIIIIIIIIIIIIIII积分IIIIIIIIIIIIIIIIIII
        Speed.PWM_Integral += Speed_Error * Speed.I;
        //PWM_Integral限幅
        if (Speed.PWM_Integral > Speed.I_Limit_PWM_max)   //限幅啊
            Speed.PWM_Integral = Speed.I_Limit_PWM_max;
        else if (Speed.PWM_Integral <= Speed.I_Limit_PWM_min)
            Speed.PWM_Integral = Speed.I_Limit_PWM_min;

        //PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
        //并不是普通限幅，用于还没快的时候
        if (Speed_Error > 0 - Speed.I_Error_Start && Speed_Error < Speed.I_Error_Start)
            Temp_Speed_P = Speed.P * Speed_Error;
        else if (Speed_Error < 0 - Speed.I_Error_Start)
            Temp_Speed_P = Speed.P * (0 - Speed.I_Error_Start);
        else if (Speed_Error > Speed.I_Error_Start)
            Temp_Speed_P = Speed.P * Speed.I_Error_Start;

        //输出
        Speed.PWM_Per = (Temp_Speed_P + Speed.PWM_Integral - Speed.PWM) / SPEED_PERIOD_CONSTANT;
    }

    Speed.PWM += Speed.PWM_Per;
}

void Direction_Control(float offset, float gyro)
{
    Direction.PWM_Per = Direction.P * (offset - 1000) + Direction.D * gyro
            - Direction.PWM;
    Direction.PWM += Direction.PWM_Per;
}

/*
 * 电机pwm输出    // 300  400  500  600  700  800  900 1000 1100
 * 测得的速度      //1130 2000 2750 3600 4350 5150 5870 6650 7450
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

void Controllor_Init(void)
{
    Motor.Dead_L = 2.f;
    Motor.Dead_R = 3.f;

    Angle.Offset = 31.f;
    Angle.P = 4.8f;
    Angle.D = 0.04f;

    Speed.Goal = 500;
    Speed.P = 0.04f;
    Speed.I = 0.0002f;

    Speed.I_Error_Start = 1200;
    Speed.I_Limit_PWM_max = 1500;
    Speed.I_Limit_PWM_min = -1500;
    Speed.PWM_Integral = 0;

    Direction.P = 0.1f;
    Direction.D = 0.2f;
}

void Controllor_Process(void)
{

}
