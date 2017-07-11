/*
 * imgprocess.h
 *
 *  Created on: 2017年5月19日
 *      Author: cheny
 */

#ifndef IMGPROCESS_H_
#define IMGPROCESS_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    bool ON;            /* 刹车总开关 */
    bool Flag;          /* 刹车线检测标志 */

    uint32_t Count;     /* 刹车线检测计数 */
    uint32_t Delay;     /* 刹车线检测间隔（毫秒） */
    uint32_t Scan_H;    /* 在哪行扫描刹车线 */
    uint32_t P_Limit;   /* 跳变点数判定阈值 */

    uint32_t StopDelay; /* 停车延时，防止停在刹车线上（毫秒） */

} imgBrake_t;

void img_find_middle(void);
void img_cross_search(void);
void img_circle_left_search(void);
void img_circle_right_search(void);
void img_smalls_search(void);
void img_barrier_search(void);

void img_brake_scan(void);

#endif /* IMGPROCESS_H_ */
