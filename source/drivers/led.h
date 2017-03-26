/*
 * led.h
 *
 *  Created on: 2017年1月26日
 *      Author: chen
 */

#ifndef SOURCE_DRIVERS_LED_H_
#define SOURCE_DRIVERS_LED_H_

/** LED状态枚举 (ON/OFF) */
typedef enum {
    ON = 0,
    OFF = 1
} ledStatus_e;

/** 公共调用函数 */
void LED_Config(void);

void LED_Red(ledStatus_e status);
void LED_Yellow(ledStatus_e status);
void LED_Green(ledStatus_e status);

void LED_Red_Toggle(void);
void LED_Yellow_Toggle(void);
void LED_Green_Toggle(void);

#endif /* SOURCE_DRIVERS_LED_H_ */
