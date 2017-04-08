/*
 * blackbox.h
 *
 *  Created on: 2017年2月9日
 *      Author: chen
 */

#ifndef USERS_BLACKBOX_H_
#define USERS_BLACKBOX_H_

#include "fsl_common.h"

/** 缓冲区状态信息结构体 */
typedef struct {
    uint8_t *Pin;
    uint8_t *Pon;

    uint16_t PinRemain;
    uint16_t PonRemain;

    uint8_t  thisBuf;
    uint16_t thisSector;
    bool     isStart;

} blackbox_Info_t;

status_t Blackbox_Config(void);

status_t Blackbox_Process(void);

status_t Blackbox_Start(void);
status_t Blackbox_Stop(void);

status_t Blackbox_SYNC(void);
status_t Blackbox_DDR(void* DDR, uint32_t len);
status_t Blackbox_CIR(void* DIR, uint32_t len);

status_t Blackbox_WriteConf(char* file, void* conf, uint32_t len);
status_t Blackbox_ReadConf(char* file, void* conf, uint32_t len);

status_t Blackbox_Format(void);

#endif /* USERS_BLACKBOX_H_ */
