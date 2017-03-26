/*
 * sbus.h
 *
 *  Created on: 2017年3月14日
 *      Author: chen
 */

#ifndef USERS_DRIVERS_SBUS_H_
#define USERS_DRIVERS_SBUS_H_

#include "fsl_common.h"

/* SBUS通道结构体 */
typedef struct
{
    uint16_t    ch[18];
    uint32_t    lostFrames;
    bool        failSafe;

} __attribute__ ((__packed__)) sbusChannel_t;

status_t Sbus_Config(void);
status_t Sbus_UpdateRC(sbusChannel_t *sbus);

#endif /* USERS_DRIVERS_SBUS_H_ */
