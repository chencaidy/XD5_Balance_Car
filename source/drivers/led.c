/*
 * led.c
 *
 *  Created on: 2017年1月26日
 *      Author: chen
 */


/* Includes ------------------------------------------------------------------*/
#include "fsl_gpio.h"
#include "led.h"

/* Variables -----------------------------------------------------------------*/
gpio_pin_config_t led_config = {
    kGPIO_DigitalOutput, 0,
};

/* Codes ---------------------------------------------------------------------*/
/**
  * @brief  初始化LED
  * @retval None
  */
void LED_Config(void)
{
    GPIO_PinInit(GPIOE, 24, &led_config);       //红色LED
    GPIO_PinInit(GPIOE, 25, &led_config);       //黄色LED
    GPIO_PinInit(GPIOE, 26, &led_config);       //绿色LED

    LED_Red(OFF);
    LED_Yellow(OFF);
    LED_Green(OFF);
}

/**
  * @brief  红色状态灯开关
  * @param  status：用ON/OFF参数选项
  * @retval None
  */
inline void LED_Red(ledStatus_e status)
{
    GPIO_WritePinOutput(GPIOE, 24, status);
}

/**
  * @brief  黄色状态灯开关
  * @param  status：用ON/OFF参数选项
  * @retval None
  */
inline void LED_Yellow(ledStatus_e status)
{
    GPIO_WritePinOutput(GPIOE, 25, status);
}

/**
  * @brief  绿色状态灯开关
  * @param  status：用ON/OFF参数选项
  * @retval None
  */
inline void LED_Green(ledStatus_e status)
{
    GPIO_WritePinOutput(GPIOE, 26, status);
}

/**
  * @brief  红色状态灯翻转
  * @retval None
  */
inline void LED_Red_Toggle(void)
{
    GPIO_TogglePinsOutput(GPIOE, 1u << 24);
}

/**
  * @brief  黄色状态灯翻转
  * @retval None
  */
inline void LED_Yellow_Toggle(void)
{
    GPIO_TogglePinsOutput(GPIOE, 1u << 25);
}

/**
  * @brief  绿色灯状态翻转
  * @retval None
  */
inline void LED_Green_Toggle(void)
{
    GPIO_TogglePinsOutput(GPIOE, 1u << 26);
}
