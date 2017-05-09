/*
 * hmi.h
 *
 *  Created on: 2017年5月9日
 *      Author: chen
 */

#ifndef DRIVERS_HMI_H_
#define DRIVERS_HMI_H_

#include "fsl_common.h"

/** 环形缓冲区数据结构 */
typedef struct {
    size_t rbCapacity;
    uint8_t  *rbHead;
    uint8_t  *rbTail;
    uint8_t  *rbBuff;
} rb_t;

status_t HMI_Config(void);

int8_t HMI_GetOnePacket(uint8_t *data, uint16_t *len);

void HMI_InsertData(char *fmt, ...);
status_t HMI_SendData(void);

status_t HMI_RxHandle(void);

#endif /* DRIVERS_HMI_H_ */
