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
#define DIR_PERIOD_CONSTANT    (2U)       //速度控制周期（2 * 5 = 10ms）

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
    Angle.PWM = (angle - Angle.A_Bias) * Angle.P
            + (gyro - Angle.G_Bias) * Angle.D;
}

/**
  * @brief  速度环控制器（建议周期：100ms）
  * @retval none
  */
void Speed_Control(int16_t cntL, int16_t cntR)
{
    static uint8_t SpdPeriod = 0;      //速度控制周期变量

    static float Speed_Error = 0;
    static float Temp_Speed_P;
    static int Temp_L_100ms = 0, Temp_R_100ms = 0;

    SpdPeriod++;
    Temp_L_100ms += cntL;
    Temp_R_100ms += cntR;
    if (SpdPeriod >= SPEED_PERIOD_CONSTANT)   //速度PID反馈调试，100ms一次计算
    {
        SpdPeriod = 0;
        Speed.L_100ms = Temp_L_100ms;
        Speed.R_100ms = Temp_R_100ms;
        Speed.Avg_100ms = (float) ((Speed.L_100ms + Speed.R_100ms) / 2.f);
        Temp_L_100ms = 0;
        Temp_R_100ms = 0;

        //速度控制
        Speed_Error = Speed.Goal - Speed.Avg_100ms;

//#include "fsl_debug_console.h"
//        PRINTF("Speed: %d\r\n", (int)Speed.Avg_100ms);

        //IIIIIIIIIIIIIIIIIIIII积分IIIIIIIIIIIIIIIIIII
        Speed.PWM_Integral += Speed_Error * Speed.I;
        //PWM_Integral限幅
        if (Speed.PWM_Integral > Speed.I_Limit_PWM_max)   //限幅啊
            Speed.PWM_Integral = Speed.I_Limit_PWM_max;
        else if (Speed.PWM_Integral <= Speed.I_Limit_PWM_min)
            Speed.PWM_Integral = Speed.I_Limit_PWM_min;

        //PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
        //并不是普通限幅，用于还没快的时候
        if (Speed_Error > -Speed.I_Error_Start && Speed_Error < Speed.I_Error_Start)
            Temp_Speed_P = Speed.P * Speed_Error;
        else if (Speed_Error < -Speed.I_Error_Start)
            Temp_Speed_P = Speed.P * -Speed.I_Error_Start;
        else if (Speed_Error > Speed.I_Error_Start)
            Temp_Speed_P = Speed.P * Speed.I_Error_Start;

        //输出
        Speed.PWM_Per = (Temp_Speed_P + Speed.PWM_Integral - Speed.PWM) / SPEED_PERIOD_CONSTANT;
    }

    Speed.PWM += Speed.PWM_Per;
}

void Direction_Control(float offset, float gyro)
{
    static uint8_t DirPeriod = 0;      //方向控制周期变量

    DirPeriod++;
    if (DirPeriod >= DIR_PERIOD_CONSTANT)   //转向PID反馈调试，10ms一次计算
    {
        DirPeriod = 0;

        Direction.PWM_Per = (offset * Direction.P
                + (gyro - Direction.G_Bias) * Direction.D - Direction.PWM);

        Direction.PWM += Direction.PWM_Per;
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

void PidControllor_Init(void)
{
    /* 默认PID参数设置 */
    Motor.Dead_L = 0.f;
    Motor.Dead_R = 2.4f;

    Angle.A_Bias = 35.0f;
    Angle.G_Bias = 0.f;
    Angle.P = 10.0f;
    Angle.D = 0.024f;

    Speed.Goal = 0.f;
    Speed.P = 0.05f;
    Speed.I = 0.0075f;
    Speed.I_Error_Start = 1000;
    Speed.I_Limit_PWM_max = 35;
    Speed.I_Limit_PWM_min = -35;
    Speed.PWM_Integral = 0.f;

    Direction.G_Bias = 0.f;
    Direction.P = 0.75f;
    Direction.D = 0.08f;
}
