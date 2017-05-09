/*
 * hmi.c
 *
 *  Created on: 2017年5月9日
 *      Author: chen
 */


/* Includes ------------------------------------------------------------------*/
#include "hmi.h"

#include "fsl_debug_console.h"
#include "fsl_uart.h"
#include "fsl_uart_freertos.h"
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdarg.h>

/* Definitions ---------------------------------------------------------------*/
#define HMI_UART               (UART0)
#define HMI_UART_CLKSRC        (UART0_CLK_SRC)
#define HMI_UART_RX_TX_IRQn    (UART0_RX_TX_IRQn)
#define HMI_BAUD               (115200U)

#define MAX_TX_LEN         (512U)                   ///< 发送缓冲最大长度
#define MAX_RX_LEN         (128U)                   ///< 接收数据最大长度
#define RB_MAX_LEN         (MAX_RX_LEN * 2)         ///< 环形缓冲区最大长度
#define min(a, b)          (a) < (b) ? (a) : (b)    ///< 获取最小值

/* Variables -----------------------------------------------------------------*/
static struct _uart_handle t_handle;
static uart_rtos_handle_t handle;
static uart_rtos_config_t uart_config;

static uint8_t tbBuf[MAX_TX_LEN];           //发送缓冲区
static uint16_t tbOffset;                   //tbBuf数据偏移量
volatile uint32_t TxLostPackCnt=0;          //发送丢包计数

const uint8_t end[3]={0xFF, 0xFF, 0xFF};    //包尾

/**@name 串口环形缓冲区实现
* @{
*/
rb_t pRb;                                               ///< 环形缓冲区结构体变量
static uint8_t rbBuf[RB_MAX_LEN];                       ///< 环形缓冲区数据缓存区
static uint8_t g_Buffer[MAX_RX_LEN];

static void rbCreate(rb_t* rb)
{
    if(NULL == rb)
    {
        PRINTF("HMI: <ERROR> input rb is NULL. \r\n");
        return;
    }

    rb->rbHead = rb->rbBuff;
    rb->rbTail = rb->rbBuff;
}

static void rbDelete(rb_t* rb)
{
    if(NULL == rb)
    {
        PRINTF("HMI: <ERROR> input rb is NULL. \r\n");
        return;
    }

    rb->rbBuff = NULL;
    rb->rbHead = NULL;
    rb->rbTail = NULL;
    rb->rbCapacity = 0;
}

static int32_t rbCapacity(rb_t *rb)
{
    if(NULL == rb)
    {
        PRINTF("HMI: <ERROR> input rb is NULL. \r\n");
        return -1;
    }

    return rb->rbCapacity;
}

static int32_t rbCanRead(rb_t *rb)
{
    if(NULL == rb)
    {
        PRINTF("HMI: <ERROR> input rb is NULL. \r\n");
        return -1;
    }

    if (rb->rbHead == rb->rbTail)
    {
        return 0;
    }

    if (rb->rbHead < rb->rbTail)
    {
        return rb->rbTail - rb->rbHead;
    }

    return rbCapacity(rb) - (rb->rbHead - rb->rbTail);
}

static int32_t rbCanWrite(rb_t *rb)
{
    if(NULL == rb)
    {
        PRINTF("HMI: <ERROR> input rb is NULL. \r\n");
        return -1;
    }

    return rbCapacity(rb) - rbCanRead(rb);
}

static int32_t rbRead(rb_t *rb, void *data, size_t count)
{
    int copySz = 0;

    if(NULL == rb)
    {
        PRINTF("HMI: <ERROR> input rb is NULL. \r\n");
        return -1;
    }

    if(NULL == data)
    {
        PRINTF("HMI: <ERROR> input data is NULL. \r\n");
        return -1;
    }

    if (rb->rbHead < rb->rbTail)
    {
        copySz = min(count, rbCanRead(rb));
        memcpy(data, rb->rbHead, copySz);
        rb->rbHead += copySz;
        return copySz;
    }
    else
    {
        if (count < rbCapacity(rb)-(rb->rbHead - rb->rbBuff))
        {
            copySz = count;
            memcpy(data, rb->rbHead, copySz);
            rb->rbHead += copySz;
            return copySz;
        }
        else
        {
            copySz = rbCapacity(rb) - (rb->rbHead - rb->rbBuff);
            memcpy(data, rb->rbHead, copySz);
            rb->rbHead = rb->rbBuff;
            copySz += rbRead(rb, (char*)data+copySz, count-copySz);
            return copySz;
        }
    }
}

static int32_t rbWrite(rb_t *rb, const void *data, size_t count)
{
    int tailAvailSz = 0;

    if(NULL == rb)
    {
        PRINTF("HMI: <ERROR> rb is empty. \r\n");
        return -1;
    }

    if(NULL == data)
    {
        PRINTF("HMI: <ERROR> data is empty. \r\n");
        return -1;
    }

    if (count >= rbCanWrite(rb))
    {
        PRINTF("HMI: <ERROR> no memory %d. \r\n", rbCanWrite(rb));
        return -1;
    }

    if (rb->rbHead <= rb->rbTail)
    {
        tailAvailSz = rbCapacity(rb) - (rb->rbTail - rb->rbBuff);
        if (count <= tailAvailSz)
        {
            memcpy(rb->rbTail, data, count);
            rb->rbTail += count;
            if (rb->rbTail == rb->rbBuff+rbCapacity(rb))
            {
                rb->rbTail = rb->rbBuff;
            }
            return count;
        }
        else
        {
            memcpy(rb->rbTail, data, tailAvailSz);
            rb->rbTail = rb->rbBuff;

            return tailAvailSz + rbWrite(rb, (char*)data+tailAvailSz, count-tailAvailSz);
        }
    }
    else
    {
        memcpy(rb->rbTail, data, count);
        rb->rbTail += count;
        return count;
    }
}
/**@} */

/**@name 串口发送缓冲队列实现
* @{
*/
static inline void tbReset(void)
{
    tbOffset = 0;
}
/**@} */

/**
* @brief   串口初始化
* @param   [in] bps    : 波特率
* @return  none
*/
static status_t uart_Init(void)
{
    status_t status = kStatus_Success;

    NVIC_SetPriority(HMI_UART_RX_TX_IRQn, 5);

    /*
     * config.baudRate_Bps = 100000U;
     * config.parityMode = kUART_ParityEven;
     * config.stopBitCount = kUART_TwoStopBit;
     */
    uart_config.base = HMI_UART;
    uart_config.srcclk = CLOCK_GetFreq(HMI_UART_CLKSRC);
    uart_config.baudrate = HMI_BAUD;
    uart_config.buffer = g_Buffer,
    uart_config.buffer_size = sizeof(g_Buffer),

    status = UART_RTOS_Init(&handle, &t_handle, &uart_config);

    return status;
}

/**
* @brief    向环形缓冲区写入数据
* @param    [in] buf        : buf地址
* @param    [in] len        : 字节长度
* @return   正确 : 返回写入的数据长度
            失败 : -1
*/
static int32_t HMI_PutData(uint8_t *buf, uint32_t len)
{
    int32_t count = 0;

    if(NULL == buf)
    {
        PRINTF("HMI: <ERROR> PutData buf is empty. \r\n");
        return -1;
    }

    count = rbWrite(&pRb, buf, len);
    if(count != len)
    {
        PRINTF("HMI: <ERROR> Failed to rbWrite. \r\n");
        return -1;
    }

    return count;
}

/**
* @brief    从环形缓冲区中抓取一包数据
*
* @param    [out] data                : 输出数据地址
* @param    [out] len                 : 输出数据长度
*
* @return : 0,正确返回; -1,错误返回
*/
int8_t HMI_GetOnePacket(uint8_t *data, uint16_t *len)
{
    uint8_t ret = 0;
    uint8_t i = 0;
    uint8_t tmpData;
    uint8_t tmpLen = 0;
    static uint16_t protocolCount = 0;
    static uint8_t lastData = 0;
    static uint8_t lastlastData = 0;
    uint8_t *protocolBuff = data;

    if((NULL == data) ||(NULL == len))
    {
        PRINTF("HMI: <ERROR> GetOnePacket Error , Illegal Param. \r\n");
        return -1;
    }

    tmpLen = rbCanRead(&pRb);
    if(0 == tmpLen)
    {
        return -1;
    }

    for(i=0; i<tmpLen; i++)
    {
        ret = rbRead(&pRb, &tmpData, 1);
        if(0 != ret)
        {
            protocolBuff[protocolCount] = tmpData;
            protocolCount++;

            if((0xFF == lastlastData) && (0xFF == lastData) && (0xFF == tmpData))
            {
                protocolBuff[protocolCount] = '\0';
                *len = protocolCount;

                protocolCount = 0;
                lastData = 0;
                lastlastData = 0;

                return 0;
            }

            lastlastData = lastData;
            lastData = tmpData;
        }
    }

    return 1;
}

/**
* @brief   串口中断函数
* @return  none
*/
status_t HMI_RxHandle(void)
{
    status_t status = kStatus_Success;
    uint8_t rxData[2];
    size_t cnt;

    status = UART_RTOS_Receive(&handle, rxData, 1, &cnt);
    if(status == kStatus_Success)
        HMI_PutData(rxData, cnt);

    return status;
}

/**
* @brief   HMI初始化
* @return  none
*/
status_t HMI_Config(void)
{
    status_t status = kStatus_Success;

    status = uart_Init();

    /* 复位HMI屏 */
    HMI_InsertData("rest");
    HMI_SendData();

    rbDelete(&pRb);
    pRb.rbCapacity = RB_MAX_LEN;
    pRb.rbBuff = rbBuf;
    rbCreate(&pRb);

    tbReset();

    return status;
}

/**
* @brief   插入缓冲区数据
* @return  none
*/
void HMI_InsertData(char *fmt, ...)
{
    va_list argptr;
    char s_buf[64];
    uint8_t s_size;

    va_start(argptr, fmt);
    s_size = vsprintf(s_buf, fmt, argptr);
    va_end(argptr);

    if(tbOffset + s_size + 3 > MAX_TX_LEN)
    {
        PRINTF("HMI: <ERROR> Tx buffer overload. \r\n");
        return;
    }
    else
    {
        memcpy(tbBuf + tbOffset, s_buf, s_size);
        memcpy(tbBuf + tbOffset + s_size, end, 3);
        tbOffset += s_size + 3;
    }
}

/**
* @brief   发送缓冲区数据
* @return  none
*/
status_t HMI_SendData(void)
{
    status_t error = kStatus_Success;

    error = UART_RTOS_Send(&handle, tbBuf, tbOffset);
    if (error != kStatus_Success)
        return error;

    tbReset();

    return error;
}
