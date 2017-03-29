#include<stdio.h>
#include<stdlib.h>

int MedianFilter(void *bitmap) {

    unsigned char *data = NULL;

    int height = 60;
    int width = 80;
    data = bitmap;

    unsigned char pixel[9] = { 0 }; //滑动窗口的像素值，初始为0
    unsigned char mid; //中值
    unsigned char temp; //中间变量
    int flag;
    int m, i, j, x, h, w, y;
    for (j = 1; j < height - 1; j++) {
        for (i = 1; i < width - 1; i++) {
            m = 0;
            for (y = j - 1; y <= j + 1; y++)
                for (x = i - 1; x <= i + 1; x++) {
                    pixel[m] = data[y * width + x];
                    m = m + 1;
                }
            do {
                flag = 0;
                for (m = 0; m < 9; m++) {
                    if (pixel[m] < pixel[m + 1]) {
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
    for (i = 0; i < height; i++) {
        for (j = 2; j < width - 1; j++) {
            m = 0;
            for (x = j - 2; x <= j + 2; x++)
                pixel[m++] = data[i * width + x];
            for (h = 0; h < 5; h++)
                for (w = h + 1; w < 5; w++) {
                    if (pixel[h] > pixel[w]) {
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
