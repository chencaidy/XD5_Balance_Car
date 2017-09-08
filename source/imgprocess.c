#include "imgprocess.h"
#include "common.h"
#include "fsl_debug_console.h"
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* 预编译部分图像位置，节省CPU时间 */
#define CAMERA_W        (80U)
#define CAMERA_H        (60U)
#define CAMERA_WT       (CAMERA_W-1)
#define CAMERA_HT       (CAMERA_H-1)
#define CAMERA_ML       (CAMERA_W/2-1)
#define CAMERA_MR       (CAMERA_W/2)

/* 图像黑白定义 */
#define BLACK           (0U)
#define WHITE           (255U)
/* 障碍左右定义 */
#define LEFT            (0U)
#define RIGHT           (1U)
/* 理想中线值 */
#define BLACK_CENTER       (39.5F)

extern uint8_t Pixmap[60][80];

float normpdf35[60] = {
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,

        0.0000, 0.0000, 0.0000, 0.3000, 0.6000,     //31-35
        0.0000, 0.1000, 0.0000, 0.0000, 0.0000,     //36-40
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //41-45
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //46-50
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //51-55
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000      //56-60
};      //系数为35

float normpdf37[60] = {
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,

        0.0000, 0.0000, 0.0000, 0.0000, 0.3000,     //31-35
        0.6000, 0.1000, 0.0000, 0.0000, 0.0000,     //36-40
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //41-45
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //46-50
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //51-55
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000      //56-60
};      //系数为35

float normpdf40[60] = {
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,

        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //31-35
        0.0000, 0.3000, 0.6000, 0.0000, 0.0000,     //36-40
        0.0000, 0.1000, 0.0000, 0.0000, 0.0000,     //41-45
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //46-50
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //51-55
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000      //56-60
};      //系数为40

float normpdf43[60] = {
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,

        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //31-35
        0.0000, 0.0000, 0.2000, 0.0000, 0.0000,     //36-40
        0.0000, 0.0000, 0.6000, 0.0000, 0.0000,     //41-45
        0.0000, 0.0000, 0.2000, 0.0000, 0.0000,     //46-50
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //51-55
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000      //56-60
};      //系数为45

float normpdf45[60] = {
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,

        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //31-35
        0.0000, 0.0000, 0.0000, 0.0000, 0.2000,     //36-40
        0.0000, 0.0000, 0.0000, 0.0000, 0.6000,     //41-45
        0.0000, 0.0000, 0.0000, 0.0000, 0.2000,     //46-50
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //51-55
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000      //56-60
};      //系数为45

float normpdf47[60] = {
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,

        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //31-35
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //36-40
        0.0000, 0.2000, 0.0000, 0.0000, 0.0000,     //41-45
        0.0000, 0.6000, 0.0000, 0.0000, 0.0000,     //46-50
        0.0000, 0.2000, 0.0000, 0.0000, 0.0000,     //51-55
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000      //56-60
};      //系数为47

float normpdf50[60] = {
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,

        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //31-35
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000,     //36-40
        0.0000, 0.0000, 0.0000, 0.0000, 0.2000,     //41-45
        0.0000, 0.0000, 0.0000, 0.0000, 0.6000,     //46-50
        0.0000, 0.0000, 0.0000, 0.0000, 0.2000,     //51-55
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000      //56-60
};      //系数为50



/* 起跑初始化检测结构体 */
imgProcess_t Process = {
    .Flag = true,
    .Delay = 1500,
    .Turn = 0,
    .normpdf = normpdf45,
};

/* 初始化刹车线检测结构体 */
imgBrake_t Brake = {
    .ON = true,
    .Flag = false,
    .Count = 0,
    .Delay = 5000,
    .Scan_H = 55,
    .P_Limit = 10,
    .StopDelay = 100,
};

/* 初始化障碍检测结构体 */
imgBarrier_t Barrier =
{
    .ON = true,
    .Flag = false,
    .Scan_H = 52,
    .Delay = 500,
    .Offset_Goal_L = 5.5,
    .Offset_Goal_R = -5.5,
};

/* 初始化圆环检测结构体 */
imgCircle_t Circle = {
    .ON = true,
    .Flag = false,
    .Delay = 1500,
    .Count = 0,
    .Dir = {0},
    .Limit = 15,
};

/* 初始化十字检测结构体 */
imgCross_t Cross={
    .ON = true,
    .Flag = false,
    .Delay = 500,
    .Count = 0,
    .Search_H = 40,
    .Search_W = 40,
};

/* 初始化坡道检测结构体 */
imgSlope_t Slope={

    .ON = false,
    .Flag = false,
    .Delay = 1000,
};

/**
  * @brief 画线算法（已测试）
  */
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

/**
  * @brief 中线算法（已测试）
  */
#define MID_SEARCH_WIDE (20U)   //中线搜寻范围 (±20)
void img_find_middle(void)
{
    int w, h;
    float left[CAMERA_H] = { 0 };
    float right[CAMERA_H] = { 0 };
    float middle[CAMERA_H] = { 0 };
    float midOffset = 0;
    float ave = 0;;

    /* 计算第60行的中线偏移 */
    if (Pixmap[59][39] == BLACK)
    {
        for (w = 0; w < MID_SEARCH_WIDE; w++)
        {
            if (Pixmap[59][40 + w] == WHITE)
            {
                midOffset = w;
                break;
            }
            if (Pixmap[59][39 - w] == WHITE)
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
        ave += (middle[h] - BLACK_CENTER) * Process.normpdf[h];
    }
    Process.Turn = ave;

    OLED_Printf(80, 2, "Turn:%4d", (int8_t) Process.Turn);
}

/**
  * @brief 起步时中线算法，防止在起跑线跑飞
  */
void img_find_middle_start(void)
{
    int w, h;
    float left[CAMERA_H] = { 0 };
    float right[CAMERA_H] = { 0 };
    float middle[CAMERA_H] = { 0 };
    float ave = 0;;

    for (h = 59; h >= 0; h--)
    {
        /* 寻找第H行的左线 */
        for (w = 39; w >= 0; w--)
        {
            left[h] = w;

            if (Pixmap[h][w] == BLACK)
                break;
        }
        /* 寻找第H行的右线 */
        for (w = 40; w <= 79; w++)
        {
            right[h] = w;

            if (Pixmap[h][w] == BLACK)
                break;
        }
        /* 计算第H行中线 */
        middle[h] = (left[h] + right[h]) / 2.f;
    }

    /* 计算偏移量 */
    for (h = 0; h < CAMERA_H; h++)
    {
        ave += (middle[h] - BLACK_CENTER) * Process.normpdf[h];
    }
    Process.Turn = ave;

    OLED_Printf(80, 2, "Turn:%4d", (int8_t) Process.Turn);
}

/**
  * @brief 十字补线算法
  */
void img_cross_search(void)
{
//    uint8_t first_white_count = 0;
//    uint8_t last_white_count = 0;
    int8_t h, w;
    uint8_t black[80] = { 0 };
    uint8_t start_y = 0, start_x = 0, end_y = 0, end_x = 0;
    uint8_t start_y_2 = 0, start_x_2 = 0, end_y_2 = 0, end_x_2 = 0;
    uint8_t all_white_count = 0;
    uint8_t mline_len = 0;

    for(h = 49;h >= 0;h--)                          //寻找全白行
       {
           for(w = 0;w <= 79;w++)
           {
               if(Pixmap[h][w] == BLACK)
                   break;
               if(w == 79)
                   all_white_count = all_white_count + 1;
           }
       }

    for(h =59;h >0;h--)
    {
        mline_len = h;
        if(Pixmap[h][40] == BLACK)
            break;
    }
    mline_len = 59 - h;                         //判断40行视距长度

    if (all_white_count >= 10 && mline_len > 50)
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

        for (w = 0; w <= 78; w++)
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

        for (w = 79; w >= 10; w--)
        {
            if ((black[w] - black[w - 1]) > 10)
            {
                start_x_2 = w;
                start_y_2 = black[w];
                break;
            }
        }

        for ((w = start_x_2 - 1); w >= 0; w--)
        {
            if (black[w] - black[w - 1] > 0)
            {
                end_x_2 = w;
                end_y_2 = black[w];
                break;
            }
            else
            {
                end_x_2 = 0;
                end_y_2 = black[0];
            }
        }
    }

//    for (h = 59; h >= 0; h--)
//    {
//        if (Pixmap[h][10] == 0)
//            break;
//        else
//            first_white_count = first_white_count + 1;        //记录第10行白线长度
//    }
//
//    for (h = 59; h >= 0; h--)
//    {
//        if (Pixmap[h][70] == 0)
//            break;
//        else
//            last_white_count = last_white_count + 1;         //记录第70行白线长度
//    }
//
//    if (first_white_count < 30 && last_white_count > 50)
//    {
//        for (w = 0; w < 80; w++)
//        {
//            for (h = 59; h >= 0; h--)
//            {
//                black[w] = h;
//                if (Pixmap[h][w] == 0)
//                    break;
//            }
//        }
//
//        for (w = 10; w <= 70; w++)
//        {
//            if ((black[w + 1] - black[w]) < -5)
//            {
//                start_x = w;
//                start_y = black[w];
//                break;
//            }
//        }
//
//        for ((w = start_x + 1); w <= 78; w++)
//        {
//            if (black[w + 1] - black[w] < 0)
//            {
//                end_x = w;
//                end_y = black[w];
//                break;
//            }
//            else
//            {
//                end_x = 79;
//                end_y = black[79];
//            }
//        }
//    }
//
//    if (first_white_count > 50 && last_white_count < 30)
//    {
//        for (w = 0; w < 80; w++)
//        {
//            for (h = 59; h >= 0; h--)
//            {
//                black[w] = h;
//                if (Pixmap[h][w] == 0)
//                    break;
//            }
//        }
//
//        for (w = 70; w >= 10; w--)
//        {
//            if ((black[w] - black[w - 1]) > 10)
//            {
//                start_x = w;
//                start_y = black[w];
//                break;
//            }
//        }
//
//        for ((w = start_x - 1); w >=10; w--)
//        {
//            if (black[w] - black[w - 1] > 0)
//            {
//                end_x = w;
//                end_y = black[w];
//                break;
//            }
//            else
//            {
//                end_x = 0;
//                end_y = black[0];
//            }
//        }
//    }

    myline1(start_x, start_y, end_x, end_y);
    myline1(start_x_2, start_y_2, end_x_2, end_y_2);
    OLED_Printf(80, 5, "S2:%6d", mline_len);
//    OLED_Printf(80, 6, "S3:%6d", all_white_count);
}

/**
  * @brief 圆环左补线（已测试）
  */
void img_circle_search(void)
{
    int8_t h, w;
    uint16_t black_count = 0;
    uint16_t small_black_count = 0;
    uint8_t black[80] = { 0 };
    uint8_t left_width[60] = {0};
    uint8_t right_width[60] = {0};
    uint8_t start_y = 0, start_x = 0, end_y = 0, end_x = 0;
    uint8_t all_white_count = 0;
//    uint8_t new_circle_rate= 0;
    float circle_rate = 0;
    float small_rate = 0;

    if (Circle.Dir[Circle.Count - 1] == LEFT)
    {
        for (h = 25; h >= 20; h--)
        {
            for (w = 27; w <= 52; w++)
            {
                if (Pixmap[h][w] == 0)
                    small_black_count = small_black_count + 1;
            }
        }
        small_rate = small_black_count / 150.f;            //计算15*50这个大区间内的黑点占有率

        if (small_rate > 0.5)
        {

            for (h = 25; h >= 10; h--)
            {
                for (w = 15; w <= 64; w++)
                {
                    if (Pixmap[h][w] == 0)
                        black_count = black_count + 1;
                }
            }
            circle_rate = black_count / 750.f;             //计算15*50这个大区间内的黑点占有率

            for (h = 15; h <= 59; h++)                          //寻找全白行
            {
                for (w = 0; w <= 79; w++)
                {
                    if (Pixmap[h][w] == 0)
                        break;
                    if (w == 79)
                        all_white_count = all_white_count + 1;
                }
            }

            for (w = 0; w <= 79; w++)                          //记录每一列的黑线长度
            {
                for (h = 59; h >= 20; h--)
                {
                    black[w] = h;
                    if (Pixmap[h][w] == 0)
                        break;
                }
            }

            for (h = 60; h >= 20; h--)                          //记录左边行的黑线宽度
            {
                for (w = 0; w <= 79; w++)
                {
                    left_width[h] = w;
                    if (Pixmap[h][w] == 0)
                        break;
                }
            }

            for (h = 59; h >= 20; h--)                          //记录右边行的黑线宽度
            {
                for (w = 70; w >= 0; w--)
                {
                    right_width[h] = 79 - w;
                    if (Pixmap[h][w] == 0)
                        break;
                }
            }


            if (circle_rate > 0.5) //利用全白行和中心圆近似率来判断圆环    左转
            {

                for (w = 79; w > 40; w--)
                {
                    if ((black[w] - black[w - 1]) > 5)
                    {
                        start_x = w;
                        start_y = black[w];
                        break;
                    }
                    else
                    {
                        start_x = 79;
                        start_y = 59;
                    }
                }

                for (h = 20; h < 58; h++)
                {
                    if ((right_width[h + 1] - right_width[h]) > 10 && right_width[h + 1]>= 75)
                    {
                        end_x = 80- right_width[h];
                        end_y = h;
                        break;
                    }
                }
            }
        }
    }
    else
    {
        for (h = 25; h >= 20; h--)
        {
            for (w = 27; w <= 52; w++)
            {
                if (Pixmap[h][w] == 0)
                    small_black_count = small_black_count + 1;
            }
        }
        small_rate = small_black_count / 150.f;            //计算15*50这个大区间内的黑点占有率

        if (small_rate > 0.5)
        {

            for (h = 25; h >= 10; h--)
            {
                for (w = 15; w <= 64; w++)
                {
                    if (Pixmap[h][w] == 0)
                        black_count = black_count + 1;
                }
            }
            circle_rate = black_count / 750.f;             //计算15*50这个大区间内的黑点占有率

            for (h = 15; h <= 59; h++)                          //寻找全白行
            {
                for (w = 0; w <= 79; w++)
                {
                    if (Pixmap[h][w] == 0)
                        break;
                    if (w == 79)
                        all_white_count = all_white_count + 1;
                }
            }

            for (w = 0; w <= 79; w++)                          //记录每一列的黑线长度
            {
                for (h = 59; h >= 20; h--)
                {
                    black[w] = h;
                    if (Pixmap[h][w] == 0)
                        break;
                }
            }

            for (h = 59; h >= 20; h--)                          //记录左边行的黑线宽度
            {
                for (w = 0; w <= 79; w++)
                {
                    left_width[h] = w;
                    if (Pixmap[h][w] == 0)
                        break;
                }
            }

        if (circle_rate > 0.5) //利用全白行和中心圆近似率来判断圆环         右转
        {
            for (w = 0; w < 40; w++)
            {
                if ((black[w + 1] - black[w]) < -5)
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
            for (h = 25; h < 58; h++)
            {
                if ((left_width[h + 1] - left_width[h]) > 10 && left_width[h+1] >=75)
                {
                    end_x = left_width[h];
                    end_y = h;
                    break;
                }
            }
        }
    }
  }
    myline1(start_x, start_y, end_x, end_y);
//    new_circle_rate = (int8_t)(circle_rate * 10);

//    OLED_Printf(80, 6, "S3:%6d", all_white_count);
//    OLED_Printf(80, 7, "S4:%6d", new_circle_rate);
}


/**
  * @brief 圆环右补线（已测试）
  */
//void img_circle_right_search(void)
//{
//    int8_t h, w;
//    uint16_t black_count = 0;
//    uint16_t small_black_count = 0;
//    uint8_t black[80] = { 0 };
//    uint8_t start_y = 0, start_x = 0, end_y = 0, end_x = 0;
//    uint8_t all_white_count = 0;
////    uint8_t new_circle_rate= 0;
//    float circle_rate = 0;
//    float small_rate = 0;
//
//
//    myline1(start_x, start_y, end_x, end_y);
////    new_circle_rate = (int8_t)(circle_rate * 10);
//
////    OLED_Printf(80, 6, "S3:%6d", all_white_count);
////    OLED_Printf(80, 7, "S4:%6d", new_circle_rate);
//}

/**
  * @brief 小S检测
  */
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

/**
  * @brief 刹车线检测（已测试）
  */
void img_brake_scan(void)
{
    int w, p_count = 0;
    static TickType_t LastTime;

    if (Brake.ON == true)
    {
        if (Brake.Flag == true)
        {
            /* FLAG已经设置，开始计时 */
            if (xTaskGetTickCount() > LastTime + Brake.Delay)
            {
                /* 时间条件满足，取消标志 */
                Brake.Flag = false;
            }
        }
        else
        {
            for (w = 0; w < (CAMERA_W - 1); w++)
            {
                if (Pixmap[Brake.Scan_H][w] != Pixmap[Brake.Scan_H][w + 1])
                    p_count++;
            }
            if (p_count > Brake.P_Limit)
            {
                /* 满足条件，设置FLAG */
                Brake.Flag = true;
                LastTime = xTaskGetTickCount();
                /* 检测次数 */
                Brake.Count++;
                PRINTF("--> FIND BRAKE LINE!\r\n");
            }
        }

        /* 进行条件判断 */
        if (Brake.Count == 1)
        {
            Brake.Scan_H = 35;
        }

        if (Brake.Count == 2)
        {
            /* 反正要停了，直接阻塞延时算了，当然是线程阻塞啦_(:3 」∠)_ */
            vTaskDelay(Brake.StopDelay);
            Angle.ON = false;
            Speed.ON = false;
            Direction.ON = false;
            Speed.Goal = 0;
            /* 清除计数 */
            Brake.Count = 0;
            Brake.Scan_H = 55;
        }

        OLED_Printf(80, 5, "BC:%6d", Brake.Count);
    }
}

/**
  * @brief 障碍检测（已测试）
  */
void img_barrier_scan(void)
{
    int w, p_count = 0;
    static TickType_t LastTime;
    uint8_t left_lenth = 0;
    uint8_t right_lenth = 0;

    if (Barrier.ON == true)
    {
        if (Barrier.Flag == true)
        {
            /* FLAG已经设置，开始计时 */
            if (xTaskGetTickCount() > LastTime + Barrier.Delay)
            {
                /* 时间条件满足，取消标志 */
                Barrier.Flag = false;
                /* 取消偏移量 */
                Barrier.Offset = 0;
            }
        }
        else
        {
            for (w = 0; w < (CAMERA_W - 1); w++)
            {
                if (Pixmap[Barrier.Scan_H][w] != Pixmap[Barrier.Scan_H][w + 1])
                    p_count++;
            }
            if (p_count == 4)
            {
                /* 判断障碍左右 */
                for (w = 39; w >= 0; w--)
                {
                    left_lenth++;
                    if (Pixmap[Barrier.Scan_H][w] == 0)
                        break;
                }
                for (w = 40; w < CAMERA_W; w++)
                {
                    right_lenth++;
                    if (Pixmap[Barrier.Scan_H][w] == 0)
                        break;
                }

                /* 左右等长，迷の条件，丑拒 */
                if (right_lenth == left_lenth)
                    return;

                /* 满足条件，设置FLAG */
                Barrier.Flag = true;
                LastTime = xTaskGetTickCount();

                if (right_lenth > left_lenth)
                {
                    /* 右边白线比左边长，障碍在左 */
                    Barrier.Dir = LEFT;
                    Barrier.Offset = Barrier.Offset_Goal_L;
                    PRINTF("--> FIND LEFT BARRIER!\r\n");
                }
                else
                {
                    /* 障碍在右 */
                    Barrier.Dir = RIGHT;
                    Barrier.Offset = Barrier.Offset_Goal_R;
                    PRINTF("--> FIND RIGHT BARRIER!\r\n");
                }
            }
        }
    }
}

/**
  * @brief 圆环检测（已测试）
  */
void img_circle_scan(void)
{
    int h, w;
    int left = 0, right = 0, middle = 0;
    uint8_t height[CAMERA_W] = { 0 };
    static TickType_t LastTime;

    if (Circle.ON == true)
    {
        if (Circle.Flag == true)
        {
            /* FLAG已经设置，开始计时 */
            if (xTaskGetTickCount() > LastTime + Circle.Delay)
            {
                /* 时间条件满足，取消标志 */
                Circle.Flag = false;
            }

        }
        else
        {
            /* 获取白线长度 */
            for (w = 0; w <= 79; w++)
            {
                for (h = 59; h >= 0; h--)
                {
                    if (Pixmap[h][w] == BLACK)
                        break;
                    else
                        height[w] = 60 - h;
                }
            }
            /* 搜寻左跳变 */
            for (w = 0; w <= 38; w++)
            {
                if (height[w + 1] - height[w] > Circle.Limit)
                {
                    left = height[w + 1];
                    break;
                }
                else
                {
                    left = -1;
                }
            }
            /* 搜寻右跳变 */
            for (w = 79; w >= 41; w--)
            {
                if (height[w - 1] - height[w] > Circle.Limit)
                {
                    right = height[w - 1];
                    break;
                }
                else
                {
                    right = -1;
                }
            }
            /* 计算中线 */
            middle = (height[39] + height[40]) / 2;

            /* 条件判断（高-低-高） */
            if (middle < left && middle < right)
            {
                /* 满足条件，设置FLAG */
                Circle.Flag = true;
                LastTime = xTaskGetTickCount();
                /* 检测次数 */
                Circle.Count++;
            }
        }

        OLED_Printf(80, 7, "CIR:%5d", Circle.Count);
    }
}



void img_cross_scan(void)
{
    static TickType_t LastTime;
    uint32_t white_count = 0 ;
    int h,w;

    if(Cross.ON == true)
    {
        if(Cross.Flag == true)
        {
            if (xTaskGetTickCount() > LastTime +Circle.Delay)
            {
                Cross.Flag = false;
            }
            img_cross_search();
        }
        else
        {
            for(w = 0; w<= 79; w++)
            {
                if(Pixmap[Cross.Search_H][w] == 0xFF)
                    white_count ++;
            }
            for(h =20;  h<= 59; h++)
            {
                if(Pixmap[h][Cross.Search_W] == 0xFF)
                    white_count ++;
            }

            if(white_count == 120)
            {
                Cross.Flag  = true;
                Cross.Count++;
                LastTime = xTaskGetTickCount();
            }
        }
        OLED_Printf(80, 6, "COS:%5d",Cross.Count);
    }
}

/**
  * @brief 起步图像算法延时定时器
  * @brief 原因：中线算法起步时与刹车线冲突，圆环检测与刹车线冲突。
  * @brief 解决方法：之前使用特殊中线算法，等待冲出起跑线后开始处理圆环。
  */
void img_start_timer(void)
{
    if (Process.Flag == false)
    {
        if (xTaskGetTickCount() > Process.LastTime + Process.Delay)
        {
            Process.Flag = true;
            Circle.Count = 0;
            Cross.Count = 0;
            PRINTF("--> Start ALL Type Image Process!\r\n");
        }
    }
}

/**
  * @brief 发车信号
  * @brief 注意：发车时必须在HMI消息处理中调用！
  * @brief 运行此函数开始重新倒计时。
  */
void img_start_signal(void)
{
    Process.LastTime = xTaskGetTickCount();
    Process.Flag = false;
}


void img_Slope_scan(void)
{
    static TickType_t LastTime;
    if(Slope.ON == true)
    {
        if(Slope.Flag == true)
        {
            if (xTaskGetTickCount() > LastTime +Slope.Delay)
            {
                Slope.Flag = false;
            }
            else
            {
                Motor.Final_PWM_L = Angle.PWM - 0.4*Speed.PWM + Direction.PWM;
                Motor.Final_PWM_R = Angle.PWM - 0.4*Speed.PWM - Direction.PWM;
            }
        }
        else
        {
            if(sensor.GyroX < -40)
                Slope.Flag = true;
                LastTime = xTaskGetTickCount();
        }
    }
}
