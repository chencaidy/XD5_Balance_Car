/*
 * motor.c
 *
 *  Created on: 2017年1月26日
 *      Author: chen
 */


/* Includes ------------------------------------------------------------------*/
#include "fsl_ftm.h"
#include "motor.h"

/* Definitions ---------------------------------------------------------------*/
#define BOARD_FTM_PWM        (FTM0)
#define BOARD_FTM_PWM_L0     (0U)
#define BOARD_FTM_PWM_L1     (1U)
#define BOARD_FTM_PWM_R0     (2U)
#define BOARD_FTM_PWM_R1     (3U)

#define BOARD_FTM_CNTL       (FTM2)
#define BOARD_FTM_CNTR       (FTM1)

#define FTM_SOURCE_CLOCK     CLOCK_GetFreq(kCLOCK_BusClk)

/* Variables -----------------------------------------------------------------*/
ftm_config_t ftmInfo;
ftm_chnl_pwm_signal_param_t pwmParam[4];
ftm_phase_params_t phaseParam;

/* Codes ---------------------------------------------------------------------*/
/**
  * @brief  电机初始化
  * @retval None
  */
void Motor_Config(void)
{
    /* 配置PWM参数，10Khz频率 */
    pwmParam[0].chnlNumber = (ftm_chnl_t)BOARD_FTM_PWM_L0;
    pwmParam[0].level = kFTM_LowTrue;
    pwmParam[0].dutyCyclePercent = 0U;
    pwmParam[0].firstEdgeDelayPercent = 0U;

    pwmParam[1].chnlNumber = (ftm_chnl_t)BOARD_FTM_PWM_L1;
    pwmParam[1].level = kFTM_LowTrue;
    pwmParam[1].dutyCyclePercent = 0U;
    pwmParam[1].firstEdgeDelayPercent = 0U;

    pwmParam[2].chnlNumber = (ftm_chnl_t)BOARD_FTM_PWM_R0;
    pwmParam[2].level = kFTM_LowTrue;
    pwmParam[2].dutyCyclePercent = 0U;
    pwmParam[2].firstEdgeDelayPercent = 0U;

    pwmParam[3].chnlNumber = (ftm_chnl_t)BOARD_FTM_PWM_R1;
    pwmParam[3].level = kFTM_LowTrue;
    pwmParam[3].dutyCyclePercent = 0U;
    pwmParam[3].firstEdgeDelayPercent = 0U;

    /* 初始化FTM - PWM输出模式 */
    FTM_GetDefaultConfig(&ftmInfo);
    FTM_Init(BOARD_FTM_PWM, &ftmInfo);

    FTM_SetupPwm(BOARD_FTM_PWM, pwmParam, 4U, kFTM_EdgeAlignedPwm, 10000U, FTM_SOURCE_CLOCK);
    FTM_StartTimer(BOARD_FTM_PWM, kFTM_SystemClock);

    /* 配置相位参数 */
    phaseParam.enablePhaseFilter = true;
    phaseParam.phaseFilterVal = 16;
    phaseParam.phasePolarity = kFTM_QuadPhaseNormal;

    /* 初始化FTM - 正交解码模式 */
    FTM_GetDefaultConfig(&ftmInfo);
    FTM_Init(BOARD_FTM_CNTL, &ftmInfo);
    FTM_Init(BOARD_FTM_CNTR, &ftmInfo);

    FTM_SetQuadDecoderModuloValue(BOARD_FTM_CNTL, 0U, 0xFFFF);
    FTM_SetQuadDecoderModuloValue(BOARD_FTM_CNTR, 0U, 0xFFFF);

    FTM_SetupQuadDecode(BOARD_FTM_CNTL, &phaseParam, &phaseParam, kFTM_QuadPhaseEncode);
    FTM_SetupQuadDecode(BOARD_FTM_CNTR, &phaseParam, &phaseParam, kFTM_QuadPhaseEncode);
}

/**
  * @brief  改变电机占空比
  * @param  [in] speedInfo： mSpeed_t结构体
  * @retval None
  */
void Motor_ChangeDuty(mSpeed_t speedInfo)
{
    if(speedInfo.pwmL >= 0)
    {
        FTM_UpdatePwmDutycycle(BOARD_FTM_PWM, (ftm_chnl_t)BOARD_FTM_PWM_L1, kFTM_EdgeAlignedPwm, speedInfo.pwmL);
        FTM_UpdatePwmDutycycle(BOARD_FTM_PWM, (ftm_chnl_t)BOARD_FTM_PWM_L0, kFTM_EdgeAlignedPwm, 0);
    }
    else
    {
        FTM_UpdatePwmDutycycle(BOARD_FTM_PWM, (ftm_chnl_t)BOARD_FTM_PWM_L1, kFTM_EdgeAlignedPwm, 0);
        FTM_UpdatePwmDutycycle(BOARD_FTM_PWM, (ftm_chnl_t)BOARD_FTM_PWM_L0, kFTM_EdgeAlignedPwm, -speedInfo.pwmL);
    }

    if(speedInfo.pwmR >= 0)
    {
        FTM_UpdatePwmDutycycle(BOARD_FTM_PWM, (ftm_chnl_t)BOARD_FTM_PWM_R1, kFTM_EdgeAlignedPwm, speedInfo.pwmR);
        FTM_UpdatePwmDutycycle(BOARD_FTM_PWM, (ftm_chnl_t)BOARD_FTM_PWM_R0, kFTM_EdgeAlignedPwm, 0);
    }
    else
    {
        FTM_UpdatePwmDutycycle(BOARD_FTM_PWM, (ftm_chnl_t)BOARD_FTM_PWM_R1, kFTM_EdgeAlignedPwm, 0);
        FTM_UpdatePwmDutycycle(BOARD_FTM_PWM, (ftm_chnl_t)BOARD_FTM_PWM_R0, kFTM_EdgeAlignedPwm, -speedInfo.pwmR);
    }

    /* Software trigger to update registers */
    FTM_SetSoftwareTrigger(BOARD_FTM_PWM, true);
}

/**
  * @brief  获取电机转速
  * @param  [in] speedInfo： mSpeed_t结构体
  * @retval None
  */
void Motor_GetCnt(mSpeed_t *speedInfo)
{
    speedInfo->cntL = FTM_GetQuadDecoderCounterValue(BOARD_FTM_CNTL);
    FTM_ClearQuadDecoderCounterValue(BOARD_FTM_CNTL);

    speedInfo->cntR = -FTM_GetQuadDecoderCounterValue(BOARD_FTM_CNTR);
    FTM_ClearQuadDecoderCounterValue(BOARD_FTM_CNTR);
}
