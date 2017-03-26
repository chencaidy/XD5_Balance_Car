/*
 * oled.c
 *
 *  Created on: 2017年3月2日
 *      Author: chen
 */


/* Includes ------------------------------------------------------------------*/
#include "oled.h"
#include "fonts.h"

#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_dspi.h"
#include "fsl_dspi_freertos.h"
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdarg.h>

/* Definitions ---------------------------------------------------------------*/
#define OLED_DSPI_MASTER_BASE        (SPI0_BASE)
#define OLED_DSPI_MASTER_IRQN        (SPI0_IRQn)
#define OLED_DSPI_MASTER_BASEADDR    ((SPI_Type *)OLED_DSPI_MASTER_BASE)

#define OLED_DSPI_MASTER_CLK_SRC     (DSPI0_CLK_SRC)

#define OLED_TRANSFER_BAUDRATE       (30000000U)     /*! Transfer baudrate - 30MHz */

#define OLED_DC_GPIO                 (GPIOA)
#define OLED_DC_GPIO_PIN             (12U)
#define OLED_RST_GPIO                (GPIOA)
#define OLED_RST_GPIO_PIN            (13U)

#define OLED_PIXEL_W        (128U)
#define OLED_PIXEL_H        (64U)

/* Variables -----------------------------------------------------------------*/
static dspi_transfer_t oled_masterXfer;
static dspi_rtos_handle_t oled_master_rtos_handle;
static dspi_master_config_t oled_masterConfig;

/* Define the init structure for the DC/RST pin */
static gpio_pin_config_t oled_gpio_config =
{ kGPIO_DigitalOutput, 1, };

static uint8_t GDRAM[OLED_PIXEL_H / 8][OLED_PIXEL_W];
static SemaphoreHandle_t dispReady;
static bool oled_isPowerOn = false;

static status_t OLED_isDetected(void)
{
    uint8_t rst;
    gpio_pin_config_t test_config =
    { kGPIO_DigitalInput, 0, };

    /* 设置 GPIO Output 模式并清零 */
    GPIO_PinInit(OLED_RST_GPIO, OLED_RST_GPIO_PIN, &oled_gpio_config);
    GPIO_ClearPinsOutput(OLED_RST_GPIO, 1U << OLED_RST_GPIO_PIN);
    /* 设置 GPIO Input 模式 */
    GPIO_PinInit(OLED_RST_GPIO, OLED_RST_GPIO_PIN, &test_config);
    /* 读取 GPIO 状态 */
    rst = GPIO_ReadPinInput(OLED_RST_GPIO, OLED_RST_GPIO_PIN);

    if (rst == 0)
        return kStatus_Fail;
    else
        return kStatus_Success;
}

static void OLED_GPIO_Config(void)
{
    /* 初始化DC GPIO. */
    GPIO_PinInit(OLED_DC_GPIO, OLED_DC_GPIO_PIN, &oled_gpio_config);
    /* 初始化RST GPIO. */
    GPIO_PinInit(OLED_RST_GPIO, OLED_RST_GPIO_PIN, &oled_gpio_config);
}

static status_t OLED_SPI_Config(void)
{
    uint32_t sourceClock;
    status_t status;

    NVIC_SetPriority(OLED_DSPI_MASTER_IRQN, 6);

    /*Master config*/
    oled_masterConfig.whichCtar = kDSPI_Ctar0;
    oled_masterConfig.ctarConfig.baudRate = OLED_TRANSFER_BAUDRATE;
    oled_masterConfig.ctarConfig.bitsPerFrame = 8;
    oled_masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
    oled_masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
    oled_masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    oled_masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 200;
    oled_masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 200;
    oled_masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 500;

    oled_masterConfig.whichPcs = kDSPI_Pcs0;
    oled_masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    oled_masterConfig.enableContinuousSCK = false;
    oled_masterConfig.enableRxFifoOverWrite = false;
    oled_masterConfig.enableModifiedTimingFormat = false;
    oled_masterConfig.samplePoint = kDSPI_SckToSin0Clock;

    sourceClock = CLOCK_GetFreq(OLED_DSPI_MASTER_CLK_SRC);
    status = DSPI_RTOS_Init(&oled_master_rtos_handle, OLED_DSPI_MASTER_BASEADDR,
            &oled_masterConfig, sourceClock);
    if (status != kStatus_Success)
    {
        PRINTF("DSPI master: error during initialization. \r\n");
        return status;
    }

    return status;
}

static status_t OLED_SPI_Transfer(void *SendBuffer, uint32_t size)
{
    status_t status;

    /*Start master transfer*/
    oled_masterXfer.txData = SendBuffer;
    oled_masterXfer.rxData = NULL;
    oled_masterXfer.dataSize = size;
    oled_masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    status = DSPI_RTOS_Transfer(&oled_master_rtos_handle, &oled_masterXfer);

    if (status != kStatus_Success)
    {
        PRINTF("DSPI master: error during transaction, %d", status);
        return status;
    }
    else
    {
        return kStatus_Success;
    }
}

static status_t OLED_SendCMD(uint8_t *regs, uint32_t size)
{
    status_t status;

    /* DC拉低，写命令 */
    GPIO_ClearPinsOutput(OLED_DC_GPIO, 1U << OLED_DC_GPIO_PIN);

    status = OLED_SPI_Transfer(regs, size);
    return status;
}

static status_t OLED_SendDATA(uint8_t *dats, uint32_t size)
{
    status_t status;

    /* DC置高，写数据 */
    GPIO_SetPinsOutput(OLED_DC_GPIO, 1U << OLED_DC_GPIO_PIN);

    status = OLED_SPI_Transfer(dats, size);
    return status;
}

static void OLED_WriteChar(uint8_t x, uint8_t y, char ch)
{
    ch -= 32;
    memcpy(&GDRAM[y][x], (void *) ASCII_5x8 + ch * 5, 5);
}

status_t OLED_Config(void)
{
    status_t status;

    /* 配置命令表 */
    uint8_t cmd_config[] =
    {
        (0xAE), /* 关闭OLED面板显示(休眠) */
        (0x00), /* 设置列地址低4bit */
        (0x10), /* 设置列地址高4bit */
        (0x40), /* 设置起始行地址（低5bit 0-63）， 硬件相关*/

        (0x81), /* 设置对比度命令(双字节命令），第1个字节是命令，第2个字节是对比度参数0-255 */
        (0xCF), /* 设置对比度参数 */

        (0xA0), /* A0 ：列地址0映射到SEG0; A1 ：列地址127映射到SEG0 */
        (0xA6), /* A6 : 设置正常显示模式; A7 : 设置为反显模式 */

        (0xA8), /* 设置COM路数 */
        (0x3F), /* 1 ->（63+1）路 */

        (0xD3), /* 设置显示偏移（双字节命令）*/
        (0x00), /* 无偏移 */

        (0xD5), /* 设置显示时钟分频系数/振荡频率 */
        (0xF0), /* 设置分频系数,高4bit是分频系数，低4bit是振荡频率 */

        (0xD9), /* 设置预充电周期 */
        (0xF1), /* [3:0],PHASE 1; [7:4],PHASE 2; */

        (0xDA), /* 设置COM脚硬件接线方式 */
        (0x12),

        (0xDB), /* 设置 vcomh 电压倍率 */
        (0x40), /* [6:4] 000 = 0.65 x VCC; 0.77 x VCC (RESET); 0.83 x VCC  */

        (0x8D), /* 设置充电泵（和下个命令结合使用） */
        (0x14), /* 0x14 使能充电泵， 0x10 是关闭 */
        (0xAF)  /* 打开OLED面板 */
    };

    status = OLED_isDetected();
    if (status != kStatus_Success)
    {
        PRINTF("OLED: Display device not found, task suspend.\r\n");
        return status;
    }

    /* 创建二值信号量 */
    dispReady = xSemaphoreCreateBinary();
    if (dispReady == NULL)
    {
        PRINTF("OLED: Semaphore create failed.\r\n");
        return kStatus_Fail;
    }

    /* 初始化外设 */
    OLED_GPIO_Config();
    status = OLED_SPI_Config();
    if (status != kStatus_Success)
        return status;

    /* 产生一个让LCD复位的低电平脉冲 */
    GPIO_ClearPinsOutput(OLED_RST_GPIO, 1U << OLED_RST_GPIO_PIN);
    vTaskDelay(1);
    GPIO_SetPinsOutput(OLED_RST_GPIO, 1U << OLED_RST_GPIO_PIN);
    vTaskDelay(5);

    /* 发送命令表 */
    status = OLED_SendCMD(cmd_config, sizeof(cmd_config));
    if (status != kStatus_Success)
        return status;

    /* 屏幕初始化完成标志 */
    oled_isPowerOn = true;

    /* 清屏 */
    OLED_Clear(0);

    PRINTF("OLED: Configuration with success, task run.\r\n");

    return status;
}

void OLED_Refresh(void)
{
    uint8_t i;
    uint8_t cmds[3];

    /* 检测是否插入屏幕 */
    if (oled_isPowerOn == false)
        return;

    /* 等待数据就绪信号 */
    if (xSemaphoreTake(dispReady, portMAX_DELAY) != pdTRUE)
        return;

    for (i = 0; i < 8; i++)
    {
        cmds[0] = 0xB0 + i;  // 设置页地址（0~7）
        cmds[1] = 0x00;      // 设置列地址的低地址
        cmds[2] = 0x10;      // 设置列地址的高地址

        OLED_SendCMD(cmds, sizeof(cmds));
        OLED_SendDATA((void *) GDRAM + OLED_PIXEL_W * i, OLED_PIXEL_W);
    }
}

void OLED_Clear(uint8_t color)
{
    uint8_t i;
    uint8_t cmds[3];

    /* 检测是否插入屏幕 */
    if (oled_isPowerOn == false)
        return;

    memset(GDRAM, color, sizeof(GDRAM));

    for (i = 0; i < 8; i++)
    {
        cmds[0] = 0xB0 + i;  // 设置页地址（0~7）
        cmds[1] = 0x00;      // 设置列地址的低地址
        cmds[2] = 0x10;      // 设置列地址的高地址

        OLED_SendCMD(cmds, sizeof(cmds));
        OLED_SendDATA((uint8_t *) GDRAM, sizeof(GDRAM));
    }
}

void OLED_WriteStr(uint8_t x, uint8_t y, char *s, uint8_t size)
{
    uint8_t i = 0;
    uint8_t sizeFix;

    /* 检测是否插入屏幕 */
    if (oled_isPowerOn == false)
        return;

    sizeFix = (OLED_PIXEL_W - x) / 5;
    sizeFix = size < sizeFix ? size : sizeFix;

    for (i = 0; i < sizeFix; i++)
        OLED_WriteChar(x + i * 5, y, s[i]);

    xSemaphoreGive(dispReady);     //置信号量，允许刷新
}

void OLED_Printf(uint8_t x, uint8_t y, char *fmt, ...)
{
    va_list argptr;
    char s_buf[25];
    uint8_t s_size;

    /* 检测是否插入屏幕 */
    if (oled_isPowerOn == false)
        return;

    va_start(argptr, fmt);
    s_size = vsprintf(s_buf, fmt, argptr);
    va_end(argptr);

    OLED_WriteStr(x, y, s_buf, s_size);
}

void OLED_DrawImage_80x60(uint8_t bitmap[][80])
{
    uint8_t row, column;
    uint8_t conv;

    /* 检测是否插入屏幕 */
    if (oled_isPowerOn == false)
        return;

    for (row = 0; row < 7; row++)
    {
        for (column = 0; column < 80; column++)
        {
            conv = 0;
            conv += 0x01 & bitmap[row * 8][column];
            conv += 0x02 & bitmap[row * 8 + 1][column];
            conv += 0x04 & bitmap[row * 8 + 2][column];
            conv += 0x08 & bitmap[row * 8 + 3][column];
            conv += 0x10 & bitmap[row * 8 + 4][column];
            conv += 0x20 & bitmap[row * 8 + 5][column];
            conv += 0x40 & bitmap[row * 8 + 6][column];
            conv += 0x80 & bitmap[row * 8 + 7][column];
            GDRAM[row][column] = conv;
        }
    }

    for (column = 0; column < 80; column++)
    {
        conv = 0;
        conv += 0x01 & bitmap[row * 8][column];
        conv += 0x02 & bitmap[row * 8 + 1][column];
        conv += 0x04 & bitmap[row * 8 + 2][column];
        conv += 0x08 & bitmap[row * 8 + 3][column];
        GDRAM[row][column] = conv;
    }

    xSemaphoreGive(dispReady);     //置信号量，允许刷新
}
