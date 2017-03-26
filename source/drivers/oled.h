/*
 * oled.h
 *
 *  Created on: 2017年3月2日
 *      Author: chen
 */

#ifndef USERS_DRIVERS_OLED_H_
#define USERS_DRIVERS_OLED_H_

#include "fsl_common.h"

status_t OLED_Config(void);

void OLED_Clear(uint8_t color);
void OLED_WriteStr(uint8_t x, uint8_t y, char *s, uint8_t size);
void OLED_Printf(uint8_t x, uint8_t y, char *fmt, ...);
void OLED_DrawImage_80x60(uint8_t bitmap[][80]);

void OLED_Refresh(void);

#endif /* USERS_DRIVERS_OLED_H_ */
