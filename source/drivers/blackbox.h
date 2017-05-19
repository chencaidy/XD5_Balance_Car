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
typedef enum
{
    INT8    = 0,
    UINT8   = 1,
    INT16   = 2,
    UINT16  = 3,
    INT32   = 4,
    UINT32  = 5,
    FLOAT   = 6,
    DOUBLE  = 7,

} DDR_Type_e;

/** 缓冲区状态信息结构体 */
typedef struct
{
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
status_t Blackbox_DDR(uint8_t ch, void* val, DDR_Type_e type);
status_t Blackbox_CIR(uint8_t* bitmap, uint32_t len);

status_t Blackbox_WriteConf(char* file, void* conf, uint32_t len);
status_t Blackbox_ReadConf(char* file, void* conf, uint32_t len);

status_t Blackbox_Format(void);
status_t Blackbox_Reset(void);

#endif /* USERS_BLACKBOX_H_ */
