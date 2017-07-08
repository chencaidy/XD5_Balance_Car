#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <math.h>

#include "common.h"

#define CAMERA_W        (80U)
#define CAMERA_H        (60U)
#define CAMERA_ML       (CAMERA_W/2-1)
#define CAMERA_MR       (CAMERA_W/2)
#define CAMERA_MT       (CAMERA_W-1)
#define CAMERA_HT       (CAMERA_H-1)        //预编译能节省CPU时间

#define BLACK           (0U)
#define WRITE           (255U)

extern int8_t offset;
extern uint8_t Pixmap[60][80];
extern sbusChannel_t rcInfo;


static void myline1(int x1, int y1, int x2, int y2)
{
    /*变量定义开始（2007/10/16增加）*/
    int iTx;        /*x轴终点的相对坐标xa或临时变量*/
    int iTy;        /*y轴终点的相对坐标ya或临时变量*/
    int iDx;        /*x轴方向的步长dx*/
    int iDy;        /*y轴方向的步长dy*/
    int iFt;        /*偏差Fm*/
    int iSt;        /*记数循环数(dx+dy)S*/
    int iXt;        /*x方向循环变量xm*/
    int iYt;        /*y方向循环变量ym*/

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
    iTx = x2 - x1;      /*取x轴的相对坐标*/
    iTy = y2 - y1;      /*取y轴的相对坐标*/
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
        if (iTx >= 0)           /*如果在第一象限*/
        {
            if (iFt < 0)        /*如果偏差小于0*/
            {
                iYt += iDy;     /*y方向走一步*/
                iFt += iTx;
            }
            else                /*如果偏差大于或等于0*/
            {
                iXt += iDx;     /*x方向走一步*/
                iFt -= iTy;
            }
        }
        else
        {
            if (iFt < 0)        /*如果偏差小于0*/
            {
                iXt -= iDx;     /*负x方向走一步*/
                iFt += iTy;
            }
            else                /*如果偏差大于或等于0*/
            {
                iYt += iDy;     /*y方向走一步*/
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

float normpdf30[60] =
{       0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0002, 0.0005, 0.0015, 0.0038, 0.0087, 0.0180, 0.0332, 0.0547,
        0.0807, 0.1065, 0.1258, 0.1330, 0.1258, 0.1065, 0.0807, 0.0547, 0.0332,
        0.0180, 0.0087, 0.0038, 0.0015, 0.0005, 0.0002, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 };         //系数为30

float normpdf15[60] =
{       0.0016,0.0027, 0.0045, 0.0071, 0.0108, 0.0158, 0.0222, 0.0299, 0.0388,
        0.0484,0.0579, 0.0666, 0.0737, 0.0782, 0.0798, 0.0782, 0.0737, 0.0666,
        0.0579,0.0484, 0.0388, 0.0299, 0.0222, 0.0158, 0.0108, 0.0071, 0.0045,
        0.0027,0.0016, 0.0009, 0.0005, 0.0002, 0.0001, 0.0001, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,0.0000};           //系数为15

float normpdf45[60] =
{       0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0001, 0.0001,
        0.0002, 0.0005, 0.0009, 0.0016, 0.0027, 0.0045, 0.0071, 0.0108, 0.0158,
        0.0222, 0.0299, 0.0388, 0.0484, 0.0579, 0.0666, 0.0737, 0.0782, 0.0798,
        0.0782, 0.0737, 0.0666, 0.0579, 0.0484, 0.0388, 0.0299, 0.0222, 0.0158,
        0.0108, 0.0071, 0.0045, 0.0027, 0.0016, 0.0009};          //系数为45

float normpdf50[60] =
{      0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
       0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
       0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
       0.0000, 0.0000, 0.0000, 0.0001, 0.0001, 0.0002, 0.0005, 0.0009, 0.0016,
       0.0027, 0.0045, 0.0071, 0.0108, 0.0158, 0.0222, 0.0299, 0.0388, 0.0484,
       0.0579, 0.0666, 0.0737, 0.0782, 0.0798, 0.0782, 0.0737, 0.0666, 0.0579,
       0.0484, 0.0388, 0.0299, 0.0222, 0.0158, 0.0108,
};                                                              //系数为50

float linspace[60] =
{2.00 ,1.98 ,1.97 ,1.95 ,1.93 ,1.92 ,1.90 ,1.88 ,1.86 ,1.85 ,1.83 ,1.81 ,1.80 ,1.78,
        1.76 ,1.75 ,1.73 ,1.71 ,1.69 ,1.68 ,1.66 ,1.64 ,1.63 ,1.61 ,1.59 ,1.58 ,1.56,
        1.54 ,1.53 ,1.51 ,1.49 ,1.47 ,1.46 ,1.44 ,1.42 ,1.41 ,1.39 ,1.37 ,1.36 ,1.34,
        1.32 ,1.31 ,1.29 ,1.27 ,1.25 ,1.24 ,1.22 ,1.20 ,1.19 ,1.17 ,1.15 ,1.14 ,1.12,
        1.10 ,1.08 ,1.07 ,1.05 ,1.03 ,1.02 ,1.00};                //一元修正函数

void img_find_middle(void)
{
#define SEARCH_WIDE (20U)   //中线搜寻范围 (±20)

    int w, h;
    float left[CAMERA_H] = { 0 };
    float right[CAMERA_H] = { 0 };
    float middle[CAMERA_H] = { 0 };
    float midOffset = 0;
    float ave = 0;;

    /* 计算第60行的中线偏移 */
    if (Pixmap[59][39] == BLACK)
    {
        for (w = 0; w < SEARCH_WIDE; w++)
        {
            if (Pixmap[59][40 + w] == WRITE)
            {
                midOffset = w;
                break;
            }
            if (Pixmap[59][39 - w] == WRITE)
            {
                midOffset = -w;
                break;
            }
        }
    }

    /* 寻找第60行的左线 */
    for (w = (39 + midOffset); w >= 0; w--)
    {
        left[59] = w;

        if (Pixmap[59][w] == BLACK)
            break;
    }
    /* 寻找第60行的右线 */
    for (w = (40 + midOffset); w < CAMERA_W; w++)
    {
        right[59] = w;

        if (Pixmap[59][w] == BLACK)
            break;
    }
    /* 计算第60行中线 */
    middle[59] = (left[59] + right[59]) / 2.f;

    /* 计算剩下59行中线 */
    for (h = 58; h >= 0; h--)
    {
        for (w = middle[h + 1]; w >= 0; w--)
        {
            left[h] = w;

            if (Pixmap[h][w] == BLACK)
                break;
        }

        for (w = middle[h + 1]; w < CAMERA_W; w++)
        {
            right[h] = w;

            if (Pixmap[h][w] == BLACK)
                break;
        }
        /* 如果左线等于右线则为无效行，取上一次数据 */
        if (left[h] == right[h])
            middle[h] = middle[h + 1];
        else
            middle[h] = (left[h] + right[h]) / 2.f;
    }

    /* 计算偏移量 */
    for (h = 0; h < CAMERA_H; h++)
    {
//        if (mline_len > 55)
//            ave += ((middle_line[h] - 39.5) * linspace[h]) * normpdf2[h];
//        else
        ave += (middle[h] - 39.5) * normpdf50[h];
    }

    offset = (int8_t) ave;

    OLED_Printf(80, 4, "S1:%6d", offset);
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
            first_white_count = first_white_count + 1;        //记录第10行白线长度
    }

    for (h = 59; h >= 0; h--)
    {
        if (Pixmap[h][70] == 0)
            break;
        else
            last_white_count = last_white_count + 1;         //记录第70行白线长度
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
                start_x = w;
                start_y = black[w];
                break;
            }
        }

        for ((w = start_x + 1); w <= 78; w++)
        {
            if (black[w + 1] - black[w] < 0)
            {
                end_x = w;
                end_y = black[w];
                break;
            }
            else
            {
                end_x = 79;
                end_y = black[79];
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
                start_x = w;
                start_y = black[w];
                break;
            }
        }

        for ((w = start_x - 1); w >=10; w--)
        {
            if (black[w] - black[w - 1] > 0)
            {
                end_x = w;
                end_y = black[w];
                break;
            }
            else
            {
                end_x = 0;
                end_y = black[0];
            }
        }
    }

    myline1(start_x, start_y, end_x, end_y);

}

void img_circle_search(void)
{
    int8_t h, w;
    uint16_t black_count = 0;
    uint8_t black[80] = { 0 };
    uint8_t start_y = 0, start_x = 0, end_y = 0, end_x = 0;
    uint8_t start_y_2 = 0, start_x_2 = 0, end_y_2 = 0, end_x_2 = 0;
    uint8_t all_white_count = 0;
    uint8_t new_circle_rate= 0;
    float circle_rate = 0;

    for(h = 25;h >= 10; h--)
    {
        for(w = 20; w <= 70; w++)
        {
            if(Pixmap[h][w] == 0)
                black_count = black_count + 1;
        }
    }
    circle_rate = black_count/750.f;                //计算15*50这个区间内的黑点占有率

    for(h = 59;h >= 0;h--)                          //寻找全白行
    {
        for(w = 0;w <= 79;w++)
        {
            if(Pixmap[h][w] == 0)
                break;
            if(w == 79)
                all_white_count = all_white_count + 1;
        }
    }


    for(w = 0;w <= 79;w++)                          //记录每一列的黑线长度
    {
        for(h = 59;h >= 0;h--)
        {
            black[w] = h;
            if(Pixmap[h][w] == 0)
                break;
        }
    }



    if  (all_white_count > 5)               //利用全白行和中心圆近似率来判断圆环
    {
        if(circle_rate >  0.5)
        {
            for(w = 0; w < 40; w++)
            {
                if  ((black[w + 1] - black[w]) < -5)
                {
                    start_x = w;
                    start_y = black[w];
                    break;
                }
                else
                {
                    start_x = 0;
                    start_y = 59;
                }
            }
            end_x = 40;
            end_y = black[40];
        }
    }
    new_circle_rate = (int8_t)(circle_rate * 10);
    myline1(start_x, start_y, end_x, end_y);
    OLED_Printf(80, 6, "S3:%6d", all_white_count);
    OLED_Printf(80, 7, "S4:%6d", new_circle_rate);
}

void img_smalls_search(void)
{
    uint8_t h, w;
    uint8_t a = 0, a_max=0, b = 0, c = 0;

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

    for(w = 0; w < 3; w++)
    {
        for (h = 0; h < 3; h++)
        {
            if (Pixmap[h][CAMERA_W - w] == 0x00)
                b++;
            if (Pixmap[h][w] == 0x00)
                c++;
        }
    }

    if(a_max < 58 && c < 8 && b < 8)
    {
        Direction.P = 0.78f;
    }
    else
    {
        Direction.P = 0.55f;
    }

    OLED_Printf(80, 5, "S2:%6d", a_max);
    OLED_Printf(80, 6, "S3:%6d", b);
    OLED_Printf(80, 7, "S4:%6d", c);
}
