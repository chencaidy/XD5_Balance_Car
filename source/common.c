/*
 * common.c
 *
 *  Created on: 2017年6月14日
 *      Author: chen
 */

#include "common.h"

/* 创建摄像头缺省配置 */
camConf_t camera = {
    .FPS = 150,
    .CNST = 128,
    .AutoAEC = true,
    .AutoAGC = true,
    .AutoAWB = true,
};
