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

typedef struct
{
    bool ON;            /* 障碍总开关 */
    bool Flag;          /* 障碍检测标志 */

    uint32_t Delay;     /* 中线偏移时长（毫秒） */
    uint32_t Scan_H;    /* 在哪行扫描障碍 */

    float Offset_Goal_L;/* 中线预期左偏量 */
    float Offset_Goal_R;/* 中线预期右偏量 */
    float Offset;       /* 满足条件后，中线偏移量 */

    uint8_t Dir;        /* 障碍方向 */

} imgBarrier_t;

typedef struct
{

    bool ON;            /* 设置圆环总开关 */
    bool Flag;          /* 设置圆环标志位 */
    uint32_t Delay;     /* 设置圆环延时时间 */

    uint32_t Count;     /* 找到圆环个数 */
    uint8_t Dir[10];    /* 转弯方向队列 */
    uint8_t Limit;      /* 跳变容差阈值 */

} imgCircle_t;

typedef struct
{
    /*设置十字总开关*/
    bool ON;
    /*设置十字标志位*/
    bool Flag;
    /*设置十字延时时间*/
    uint32_t Delay;
    /*找到十字个数*/
    uint32_t Count;
    /*设置十字行数*/
    uint32_t Search_W;
    /*设置十字列数*/
    uint32_t Search_H;
}imgCross_t;

void img_find_middle(void);
void img_cross_search(void);
void img_circle_left_search(void);
void img_circle_right_search(void);
void img_smalls_search(void);

void img_brake_scan(void);
void img_barrier_scan(void);
void img_circle_scan(int debug);
void img_cross_scan(void);

#endif /* IMGPROCESS_H_ */
