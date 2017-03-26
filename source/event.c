/*
 * event.c
 *
 *  Created on: 2017年2月24日
 *      Author: chen
 */


/* Includes ------------------------------------------------------------------*/
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"

#include "FreeRTOS.h"

#include "led.h"
#include "imu.h"
#include "camera.h"

#include "control.h"

/* Interrupts ----------------------------------------------------------------*/
/**
  * @brief  PORTB中断服务程序（MPU9250）
  * @retval none
  */
void PORTB_IRQHandler(void)
{
    traceISR_ENTER();

    /* 姿态传感器中断回调 */
    IMU_IRQ_Handle();

    /* 中断测试代码（LED指示） */
    static uint16_t i = 0;
    if (++i == 10)
    {
        i = 0;
        IMU_ToggleLED();
    }

    traceISR_EXIT();
}

/**
  * @brief  PORTC中断服务程序（摄像头）
  * @retval none
  */
void PORTC_IRQHandler(void)
{
    traceISR_ENTER();

    /* 摄像头中断回调 */
    CAM_ISR_Handle();

    /* 中断测试代码（LED指示） */
    static uint16_t i = 0;
    if (++i == 10)
    {
        i = 0;
        LED_Red_Toggle();
    }

    traceISR_EXIT();
}

/**
  * @brief  PIT0中断服务程序（PID控制器）
  * @retval none
  */
void PIT0_IRQHandler(void)
{
    traceISR_ENTER();

    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);

    /* PID处理 */
    PidControllor_Process();

    traceISR_EXIT();
}

void HardFault_Handler(void)
{
    /* HardFault指示（三灯全亮） */
    LED_Green(ON);
    LED_Yellow(ON);
    LED_Red(ON);

    while (1)
    {
    }
}
