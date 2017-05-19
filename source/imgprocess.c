#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "oled.h"
#include "sbus.h"
#include "camera.h"
#include "blackbox.h"

#define CAMERA_W        (80U)
#define CAMERA_H        (60U)
#define CAMERA_ML       (CAMERA_W/2-1)
#define CAMERA_MR       (CAMERA_W/2)
#define CAMERA_MT       (CAMERA_W-1)
#define CAMERA_HT       (CAMERA_H-1)        //预编译能节省CPU时间

extern int8_t offset;
extern uint8_t Pixmap[60][80];
extern sbusChannel_t rcInfo;

int MedianFilter(void *bitmap)
{
    unsigned char *data = NULL;

    int height = 60;
    int width = 80;
    data = bitmap;

    unsigned char pixel[9] = { 0 }; //滑动窗口的像素值，初始为0
    unsigned char mid; //中值
    unsigned char temp; //中间变量
    int flag;
    int m, i, j, x, h, w, y;

    for (j = 1; j < height - 1; j++)
    {
        for (i = 1; i < width - 1; i++)
        {
            m = 0;
            for (y = j - 1; y <= j + 1; y++)
                for (x = i - 1; x <= i + 1; x++)
                {
                    pixel[m] = data[y * width + x];
                    m = m + 1;
                }
            do
            {
                flag = 0;
                for (m = 0; m < 9; m++)
                {
                    if (pixel[m] < pixel[m + 1])
                    {
                        temp = pixel[m];
                        pixel[m] = pixel[m + 1];
                        pixel[m + 1] = temp;
                        flag = 1;
                    } //if
                } //for
            } while (flag == 1);
            mid = pixel[4];
            data[width * j + i] = mid;
        }
    }
    for (i = 0; i < height; i++)
    {
        for (j = 2; j < width - 1; j++)
        {
            m = 0;
            for (x = j - 2; x <= j + 2; x++)
                pixel[m++] = data[i * width + x];
            for (h = 0; h < 5; h++)
                for (w = h + 1; w < 5; w++)
                {
                    if (pixel[h] > pixel[w])
                    {
                        temp = pixel[w];
                        pixel[w] = pixel[h];
                        pixel[h] = temp;
                    }
                }
            data[i * width + j] = pixel[2];
        }
    }

    return 0;
}

void img_process(void)
{
    uint8_t left_border[80] =
    { 0 };
    uint8_t right_border[80] =
    { 0 };
    uint8_t middle_line[80] =
    { 0 };
    uint8_t h, w;        //当前帧 高,宽 计次
    uint8_t ave;         //当前中线的均值
    CAM_ImageExtract(Pixmap);
    MedianFilter(Pixmap);
    for (h = 0; h < 5; h++)
    {
        for (w = 40; w < CAMERA_W; w++)
        {
            right_border[h] = w;
            if (Pixmap[h][w] == 0)
                break;
        }

        for (w = 40; w >= 0; w--)
        {
            left_border[h] = w;
            if (Pixmap[h][w] == 0)
                break;
        }
        middle_line[h] = (left_border[h] + right_border[h]) / 2;
    }
    for (h = 5; h < CAMERA_H; h++)
    {
        for (w = middle_line[h - 1]; w < CAMERA_W; w++)
        {
            right_border[h] = w;
            if (Pixmap[h][w] == 0)
                break;
        }
        for (w = middle_line[h - 1]; w >= 0; w++)
        {
            left_border[h] = w;
            if (Pixmap[h][w] == 0)
                break;
        }
        middle_line[h] = (left_border[h] + right_border[h]) / 2;
    }

    for (h = 0; h < CAMERA_H; h++)
    {
        middle_line[h] = (right_border[h] + left_border[h]) / 2;
        ave = ave + middle_line[h] / 60;
    }
    offset = ave - 40;

    OLED_Printf(80, 4, "S1:%6d", ave);
}

void Algorithm_Bak(void)
{
    uint8_t h, w;        //当前帧 高,宽 计次
    uint8_t a = 0, a_max = 0;    //当前帧视距
    int8_t b = 0, c = 0;    //左线长度，右线长度

    CAM_ImageExtract(Pixmap);

    //获取前方视距，提供分段 PD依据
    for (w = 0; w < 10; w++)
    {
        for (h = 0; h < CAMERA_H; h++)       //左容差范围
        {
            if (Pixmap[CAMERA_HT - h][CAMERA_ML - w] == 0xFF)
            {
                a++;
            }
            else
            {
                break;
            }
        }
        if (a > a_max)
        {
            a_max = a;
        }
        a = 0;

        for (h = 0; h < CAMERA_H; h++)       //右容差范围
        {
            if (Pixmap[CAMERA_HT - h][CAMERA_MR + w] == 0xFF)
            {
                a++;
            }
            else
            {
                break;
            }
        }
        if (a > a_max)
        {
            a_max = a;
        }
        a = 0;
    }

    //获取左线长度
    for (w = 0; w < CAMERA_MR; w++)
    {
        if (Pixmap[CAMERA_HT - w][CAMERA_ML - w] == 0xFF)
        {
            b++;
        }
        else
        {
            break;
        }
    }
    //获取右线长度
    for (w = 0; w < CAMERA_MR; w++)
    {
        if (Pixmap[CAMERA_HT - w][CAMERA_MR + w] == 0xFF)
        {
            c++;
        }
        else
        {
            break;
        }
    }

    //超出视距校正
    if (b == 0 || c == 0)
    {
        for (w = 0; w < CAMERA_MR; w++)
        {
            if (Pixmap[CAMERA_HT][CAMERA_ML - w] == 0xFF)
            {
                b++;
            }
            if (Pixmap[CAMERA_HT][CAMERA_MR + w] == 0xFF)
            {
                c++;
            }
        }
    }

    //计算偏差
    offset = b - c;

    OLED_Printf(80, 4, "S1:%6d", a_max);
    OLED_Printf(80, 5, "S3:%6d", b);
    OLED_Printf(80, 6, "S5:%6d", c);

    if (rcInfo.ch[9] > 1024)
    {
        Blackbox_SYNC();
        Blackbox_CIR(CAM_GetBitmap(), 600);
    }
}
