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

/** 黑匣子文件结构 */
typedef struct {
    uint16_t  num;              //文件数，最大支持1023次数据记录
    uint16_t  fileEnd[1023];    //各文件结束扇区

} __attribute__ ((__packed__)) blackbox_fs_t;

///** 黑匣子数据包结构 */
//typedef struct {
//    uint16_t  Size;          //数据大小校验位
//
//
//    uint8_t   Image[600];    //图形数据
//
//} __attribute__ ((__packed__)) w25q_Blackbox_t;

status_t Blackbox_Config(void);

status_t Blackbox_Start(void);
status_t Blackbox_Stop(void);
status_t Blackbox_Format(void);
status_t Blackbox_Insert(void *buf, uint16_t len);
status_t Blackbox_Process(void);

status_t Blackbox_WriteConf(char* file, void* conf, uint32_t len);
status_t Blackbox_ReadConf(char* file, void* conf, uint32_t len);

#endif /* USERS_BLACKBOX_H_ */
