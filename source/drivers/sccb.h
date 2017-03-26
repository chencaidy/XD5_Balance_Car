/*
 * sccb.h
 *
 *  Created on: 2017年3月4日
 *      Author: chen
 */

#ifndef USERS_DRIVERS_SCCB_H_
#define USERS_DRIVERS_SCCB_H_

#include "fsl_common.h"

void SCCB_Config(void);

int SCCB_WriteReg(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);
int SCCB_ReadReg(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data_ptr);

#endif /* USERS_DRIVERS_SCCB_H_ */
