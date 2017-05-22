#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <math.h>

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


static void img_drawline(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    float gradient;
    int8_t h, w;

    gradient = (y2 - y1) / (x2 - x1);

    for (w = x1; w <= x2; w++)
    {
        h = (int8_t) (gradient * (w - x1));
        Pixmap[h][w] = 0;
    }
}

static void myline1(int x1, int y1, int x2, int y2)
{
    /*变量定义开始（2007/10/16增加）*/
    int iTx; /*x轴终点的相对坐标xa或临时变量*/
    int iTy; /*y轴终点的相对坐标ya或临时变量*/
    int iDx; /*x轴方向的步长dx*/
    int iDy; /*y轴方向的步长dy*/
    int iFt; /*偏差Fm*/
    int iSt; /*记数循环数(dx+dy)S*/
    int iXt; /*x方向循环变量xm*/
    int iYt; /*y方向循环变量ym*/
    /*变量定义结束*/
    /*变量初始化开始*/
    /*如果是第三象限或第四象限则换成第一或第二象限*/
    if (y2 < y1)
    {
        iTx = x1;
        x1 = x2;
        x2 = iTx;
        iTy = y1;
        y1 = y2;
        y2 = iTy;
    }
    iTx = x2 - x1; /*取x轴的相对坐标*/
    iTy = y2 - y1; /*取y轴的相对坐标*/
    iDx = 1;
    iDy = 1;
    iFt = 0;
    iSt = iTx + iTy;
    if (iTx < 0)
        iSt = -1 * iTx + iTy;
    ; /*如果在第二象限，则x轴方向步长取负值*/
    iXt = 0;
    iYt = 0;
    /*变量初始化结束*/
    /*数据处理开始*/
    while (iSt > 0)
    {
        Pixmap[y1 + iYt][x1 + iXt] = 0;
        if (iTx >= 0) /*如果在第一象限*/
        {
            if (iFt < 0) /*如果偏差小于0*/
            {
                iYt += iDy; /*y方向走一步*/
                iFt += iTx;
            }
            else /*如果偏差大于或等于0*/
            {
                iXt += iDx; /*x方向走一步*/
                iFt -= iTy;
            }
        }
        else
        {
            if (iFt < 0) /*如果偏差小于0*/
            {
                iXt -= iDx; /*负x方向走一步*/
                iFt += iTy;
            }
            else /*如果偏差大于或等于0*/
            {
                iYt += iDy; /*y方向走一步*/
                iFt += iTx;
            }
        }
        iSt--;
    }
}

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

float normpdf[60] =
{ 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0002, 0.0005, 0.0015, 0.0038, 0.0087, 0.0180, 0.0332, 0.0547,
        0.0807, 0.1065, 0.1258, 0.1330, 0.1258, 0.1065, 0.0807, 0.0547, 0.0332,
        0.0180, 0.0087, 0.0038, 0.0015, 0.0005, 0.0002, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, };

void img_find_middle(void)
{
    uint8_t left_border[CAMERA_H] = {0};
    uint8_t right_border[CAMERA_H] = {0};
    uint8_t middle_line[CAMERA_H] = {0};
    int8_t h, w;        //当前帧 高,宽 计次
    float ave;         //当前中线的均值

//
//    img_cross_search();
//    img_cross_drawline();
//    MedianFilter(Pixmap);

    for (h = CAMERA_H - 1; h >55; h--)
    {
        for (w = 40; w < CAMERA_W; w++)
        {
            right_border[h] = w;
            if (Pixmap[h][w] == 0)
                break;
        }

        for (w = 39; w >= 0; w--)
        {
            left_border[h] = w;
            if (Pixmap[h][w] == 0)
                break;
        }
        middle_line[h] = (left_border[h] + right_border[h]) / 2;
    }
    for (h = 55; h >=0; h--)
    {
        for (w = middle_line[h + 1]; w < CAMERA_W; w++)
        {
            right_border[h] = w;
            if (Pixmap[h][w] == 0)
                break;
        }
        for (w = middle_line[h + 1]; w >= 0; w--)
        {
            left_border[h] = w;
            if (Pixmap[h][w] == 0)
                break;
        }
        middle_line[h] = (left_border[h] + right_border[h]) / 2;
    }

    for (h = 0; h < CAMERA_H; h++)
    {
        ave = ave + middle_line[h]*normpdf[h] ;
    }
//    ave = ave / 60 ;
    offset = (int8_t)(40 - ave);

    OLED_Printf(80, 4, "S1:%6d", offset);


    if (rcInfo.ch[9] > 1024)
    {
        Blackbox_SYNC();
        Blackbox_CIR(CAM_GetBitmap(), 600);
    }
}

void img_cross_search(void)
{
    uint8_t first_white_count = 0;
    uint8_t last_white_count = 0;
    int8_t h, w;
    uint8_t black[80] = { 0 };
    uint8_t start_y = 0, start_x = 0, end_y = 0, end_x = 0;

    for (h = 59; h >= 0; h--)
    {
        if (Pixmap[h][10] == 0)
            break;
        else
            first_white_count = first_white_count + 1;
    }

    for (h = 59; h >= 0; h--)
    {
        if (Pixmap[h][70] == 0)
            break;
        else
            last_white_count = last_white_count + 1;
    }

    if (first_white_count < 30 && last_white_count > 50)
    {
        for (w = 0; w < 80; w++)
        {
            for (h = 59; h >= 0; h--)
            {
                black[w] = h;
                if (Pixmap[h][w] == 0)
                    break;
            }
        }

        for (w = 10; w <= 70; w++)
        {
            if ((black[w + 1] - black[w]) < -5)
            {
                start_y = black[w];
                start_x = w;
                break;
            }
        }

        for ((w = start_x + 1); w <= 78; w++)
        {
            if (black[w + 1] - black[w] < 0)
            {
                end_y = black[w];
                end_x = w;
                break;
            }
            else
            {
                end_y = black[79];
                end_x = 79;
            }
        }
    }

    if (first_white_count > 50 && last_white_count < 30)
    {
        for (w = 0; w < 80; w++)
        {
            for (h = 59; h >= 0; h--)
            {
                black[w] = h;
                if (Pixmap[h][w] == 0)
                    break;
            }
        }

        for (w = 70; w >= 10; w--)
        {
            if ((black[w] - black[w - 1]) > 10)
            {
                start_y = black[w];
                start_x = w;
                break;
            }
        }

        for ((w = start_x - 1); w >=10; w--)
        {
            if (black[w] - black[w - 1] > 0)
            {
                end_y = black[w];
                end_x = w;
                break;
            }
            else
            {
                end_y = black[0];
                end_x = 0;
            }
        }
    }

    myline1(start_x, start_y, end_x, end_y);

}

//void Algorithm_Bak(void)
//{
//    uint8_t h, w;        //当前帧 高,宽 计次
//    uint8_t a = 0, a_max = 0;    //当前帧视距
//    int8_t b = 0, c = 0;    //左线长度，右线长度
//
//    CAM_ImageExtract(Pixmap);
//
//    //获取前方视距，提供分段 PD依据
//    for (w = 0; w < 10; w++)
//    {
//        for (h = 0; h < CAMERA_H; h++)       //左容差范围
//        {
//            if (Pixmap[CAMERA_HT - h][CAMERA_ML - w] == 0xFF)
//            {
//                a++;
//            }
//            else
//            {
//                break;
//            }
//        }
//        if (a > a_max)
//        {
//            a_max = a;
//        }
//        a = 0;
//
//        for (h = 0; h < CAMERA_H; h++)       //右容差范围
//        {
//            if (Pixmap[CAMERA_HT - h][CAMERA_MR + w] == 0xFF)
//            {
//                a++;
//            }
//            else
//            {
//                break;
//            }
//        }
//        if (a > a_max)
//        {
//            a_max = a;
//        }
//        a = 0;
//    }
//
//    //获取左线长度
//    for (w = 0; w < CAMERA_MR; w++)
//    {
//        if (Pixmap[CAMERA_HT - w][CAMERA_ML - w] == 0xFF)
//        {
//            b++;
//        }
//        else
//        {
//            break;
//        }
//    }
//    //获取右线长度
//    for (w = 0; w < CAMERA_MR; w++)
//    {
//        if (Pixmap[CAMERA_HT - w][CAMERA_MR + w] == 0xFF)
//        {
//            c++;
//        }
//        else
//        {
//            break;
//        }
//    }
//
//    //超出视距校正
//    if (b == 0 || c == 0)
//    {
//        for (w = 0; w < CAMERA_MR; w++)
//        {
//            if (Pixmap[CAMERA_HT][CAMERA_ML - w] == 0xFF)
//            {
//                b++;
//            }
//            if (Pixmap[CAMERA_HT][CAMERA_MR + w] == 0xFF)
//            {
//                c++;
//            }
//        }
//    }
//
//    //计算偏差
//    offset = b - c;
//
//    OLED_Printf(80, 4, "S1:%6d", a_max);
//    OLED_Printf(80, 5, "S3:%6d", b);
//    OLED_Printf(80, 6, "S5:%6d", c);
//
//    if (rcInfo.ch[9] > 1024)
//    {
//        Blackbox_SYNC();
//        Blackbox_CIR(CAM_GetBitmap(), 600);
//    }
//}
