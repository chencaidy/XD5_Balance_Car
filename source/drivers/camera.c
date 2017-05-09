/*
 * camera.c
 *
 *  Created on: 2017年3月5日
 *      Author: chen
 */

/* Includes ------------------------------------------------------------------*/
#include "camera.h"
#include "sccb.h"

#include "FreeRTOS.h"
#include "task.h"

#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_edma.h"
#include "fsl_dma_manager.h"

/* Definitions ---------------------------------------------------------------*/
#define CAM_PCLK_PORT       (PORTC)
#define CAM_PCLK_GPIO       (GPIOC)
#define CAM_PCLK_GPIO_PIN   (17U)

#define CAM_VSYNC_PORT      (PORTC)
#define CAM_VSYNC_IRQN      (PORTC_IRQn)
#define CAM_VSYNC_GPIO      (GPIOC)
#define CAM_VSYNC_GPIO_PIN  (18U)

#define CAM_DATA_GPIO       (GPIOD)
#define CAM_DATA_OFFSET     (0U)

#define CAM_DMA_CHANNEL     (0U)
#define CAM_DMA_REQUEST     (kDmaRequestMux0PortC)

#define CAM_SCCB_ADDR       (0x42U)      /* OV7725 8bit addr is 0x42 */

#define CAM_RB_NUM          (4U)         /* 环形缓冲区长度 */

#define OV7725_W    (80)
#define OV7725_H    (60)

/* Private typedef -----------------------------------------------------------*/
struct ov7725_reg
{
    uint8_t addr;
    uint8_t val;
};

/* Prototypes ----------------------------------------------------------------*/
static void CAM_eDMA_Callback(edma_handle_t *handle, void *param,
        bool transferDone, uint32_t tcds);

/* Variables -----------------------------------------------------------------*/
static dmamanager_handle_t dmamanager_handle;
static edma_handle_t g_EDMA_Handle;
static edma_transfer_config_t transferConfig;

/* 环形缓冲区实现 */
static uint8_t rb[CAM_RB_NUM][OV7725_H * (OV7725_W / 8)];
static uint8_t *rb_thisBuf = NULL;
static uint8_t *rb_lastBuf = NULL;
static volatile uint8_t rb_num = 0;

static const struct ov7725_reg reg_tbl[] =
{
    {OV7725_COM4         , 0xC1},
    {OV7725_CLKRC        , 0x00},
    {OV7725_COM2         , 0x03},
    {OV7725_COM3         , 0xD0},
    {OV7725_COM7         , 0x40},
    {OV7725_COM8         , 0xC8},   //关闭AEC AWB AGC
    {OV7725_HSTART       , 0x3F},
    {OV7725_HSIZE        , 0x50},
    {OV7725_VSTRT        , 0x03},
    {OV7725_VSIZE        , 0x78},
    {OV7725_HREF         , 0x40},
    {OV7725_SCAL0        , 0x0A},
    {OV7725_AWB_Ctrl0    , 0xE0},
    {OV7725_DSPAuto      , 0xff},
    {OV7725_DSP_Ctrl2    , 0x0C},
    {OV7725_DSP_Ctrl3    , 0x00},
    {OV7725_DSP_Ctrl4    , 0x00},
    {OV7725_EXHCH        , 0x00},
    {OV7725_GAM1         , 0x0c},
    {OV7725_GAM2         , 0x16},
    {OV7725_GAM3         , 0x2a},
    {OV7725_GAM4         , 0x4e},
    {OV7725_GAM5         , 0x61},
    {OV7725_GAM6         , 0x6f},
    {OV7725_GAM7         , 0x7b},
    {OV7725_GAM8         , 0x86},
    {OV7725_GAM9         , 0x8e},
    {OV7725_GAM10        , 0x97},
    {OV7725_GAM11        , 0xa4},
    {OV7725_GAM12        , 0xaf},
    {OV7725_GAM13        , 0xc5},
    {OV7725_GAM14        , 0xd7},
    {OV7725_GAM15        , 0xe8},
    {OV7725_SLOP         , 0x20},
    {OV7725_LC_RADI      , 0x00},
    {OV7725_LC_COEF      , 0x13},
    {OV7725_LC_XC        , 0x08},
    {OV7725_LC_COEFB     , 0x14},
    {OV7725_LC_COEFR     , 0x17},
    {OV7725_LC_CTR       , 0x05},
    {OV7725_BDBase       , 0x99},
    {OV7725_BDMStep      , 0x03},
    {OV7725_SDE          , 0x04},
    {OV7725_BRIGHT       , 0x00},
    {OV7725_CNST         , 0x80},
    {OV7725_SIGN         , 0x06},
    {OV7725_UVADJ0       , 0x11},
    {OV7725_UVADJ1       , 0x02},
    {OV7725_HOutSize     , 0x14},
    {OV7725_VOutSize     , 0x1e},
};

/* User Codes ----------------------------------------------------------------*/
static void CAM_Buffer_Update(void)
{
    rb_num++;
    if (rb_num > CAM_RB_NUM - 1)
        rb_num = 0;

    rb_lastBuf = rb_thisBuf;
    rb_thisBuf = rb[rb_num];
}

static void CAM_Buffer_Config(void)
{
    memset(rb, 0x00, sizeof(rb));

    rb_lastBuf = NULL;
    rb_thisBuf = rb[0];
    rb_num = 0;
}

static void CAM_DMA_Config(void)
{
    /* Initialize DMAMGR */
    DMAMGR_Init(&dmamanager_handle, DMA0, 16, 0);
    /* Request a DMAMUX channel by static allocate mechanism */
    DMAMGR_RequestChannel(&dmamanager_handle, CAM_DMA_REQUEST, CAM_DMA_CHANNEL,
            &g_EDMA_Handle);

    /* Configure Callback. */
    EDMA_SetCallback(&g_EDMA_Handle, CAM_eDMA_Callback, NULL);

    /* Prepare transfer. */
    EDMA_PrepareTransfer(&transferConfig,
            (void *) &CAM_DATA_GPIO->PDIR + CAM_DATA_OFFSET, sizeof(uint8_t),
            rb_thisBuf, sizeof(uint8_t), sizeof(uint8_t),
            OV7725_H * (OV7725_W / 8), kEDMA_PeripheralToMemory);
}

static status_t CAM_SCCB_Config(void)
{
    status_t status;
    uint8_t i, PID, VER;

    /* 初始化 SCCB 总线 */
    SCCB_Config();

    /* 设备自检 */
    SCCB_ReadReg(CAM_SCCB_ADDR, OV7725_PID, &PID);
    SCCB_ReadReg(CAM_SCCB_ADDR, OV7725_VER, &VER);
    PRINTF("Camera: Chip PID is 0x%x, VER is 0x%x \r\n", PID, VER);
    if (PID == 0x77 && VER == 0x21)
        PRINTF("Camera: Found a OV7725 device. \r\n");
    else
        return kStatus_Fail;

    /* 复位 OV7725 */
    SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_COM7, 0x80);
    vTaskDelay(50);

    /* 配置寄存器 */
    for (i = 0; i < ARRAY_SIZE(reg_tbl); i++)
    {
        status = SCCB_WriteReg(CAM_SCCB_ADDR, reg_tbl[i].addr, reg_tbl[i].val);
        if (status != kStatus_Success)
        {
            PRINTF("Camera: Register 0x%x write failed \r\n", reg_tbl[i].addr);
            return kStatus_Fail;
        }
    }

    return kStatus_Success;
}

static void CAM_GPIO_Config(void)
{
    static gpio_pin_config_t cam_config =
    { kGPIO_DigitalInput, 1, };

    /* Init PCLK GPIO. */
    GPIO_PinInit(CAM_PCLK_GPIO, CAM_PCLK_GPIO_PIN, &cam_config);

    /* Init VSYNC GPIO. */
    NVIC_SetPriority(CAM_VSYNC_IRQN, 6);
    PORT_SetPinInterruptConfig(CAM_VSYNC_PORT, CAM_VSYNC_GPIO_PIN,
            kPORT_InterruptRisingEdge);
    GPIO_PinInit(CAM_VSYNC_GPIO, CAM_VSYNC_GPIO_PIN, &cam_config);
    GPIO_ClearPinsInterruptFlags(CAM_VSYNC_GPIO, 1U << CAM_VSYNC_GPIO_PIN);
    EnableIRQ(CAM_VSYNC_IRQN);
}

status_t CAM_Config(void)
{
    status_t status;

    status = CAM_SCCB_Config();
    if (status != kStatus_Success)
        return status;

    CAM_Buffer_Config();
    CAM_DMA_Config();
    CAM_GPIO_Config();

    return kStatus_Success;
}

uint8_t *CAM_GetBitmap(void)
{
    return rb_lastBuf;
}

void CAM_ImageExtract(void *pixmap)
{
    uint8_t color[2] =
    { 255, 0 }; //0 和 1 分别对应的颜色
    uint8_t *mdst = pixmap;
    uint8_t *msrc = rb_lastBuf;
    //注：山外的摄像头 0 表示 白色，1表示 黑色

    uint8_t tmpsrc;
    uint16_t strlen = OV7725_H * (OV7725_W / 8);

    if (msrc == NULL)
        return;

    while (strlen--)
    {
        tmpsrc = *msrc++;
        *mdst++ = color[(tmpsrc >> 7) & 0x01];
        *mdst++ = color[(tmpsrc >> 6) & 0x01];
        *mdst++ = color[(tmpsrc >> 5) & 0x01];
        *mdst++ = color[(tmpsrc >> 4) & 0x01];
        *mdst++ = color[(tmpsrc >> 3) & 0x01];
        *mdst++ = color[(tmpsrc >> 2) & 0x01];
        *mdst++ = color[(tmpsrc >> 1) & 0x01];
        *mdst++ = color[(tmpsrc >> 0) & 0x01];
    }
}

void CAM_UpdateProfile(camConf_t *cam)
{
    /* 写入对比度 */
    SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_CNST, cam->CNST);
    /* 写入曝光时间 */
    SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_AEC, (uint8_t) (cam->AEC));
    SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_AECH, (uint8_t) (cam->AEC >> 8));
    /* 写入自动处理函数 */
    uint8_t temp = 0xC8;
    if (cam->AutoAEC == true)
        temp |= 0x01;   //自动曝光
    if (cam->AutoAWB == true)
        temp |= 0x02;   //自动白平衡
    if (cam->AutoAGC == true)
        temp |= 0x04;   //自动增益
    SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_COM8, temp);
    /* 写入刷新率 */
    switch (cam->FPS)
    {
        case 50:
            SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_COM4, 0xC1);
            SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_CLKRC, 0x02);
            break;
        case 75:
            SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_COM4, 0x41);
            SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_CLKRC, 0x00);
            break;
        case 112:
            SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_COM4, 0x81);
            SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_CLKRC, 0x00);
            break;
        case 150:
            SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_COM4, 0xC1);
            SCCB_WriteReg(CAM_SCCB_ADDR, OV7725_CLKRC, 0x00);
            break;
        default:
            PRINTF("Camera: FPS is undefined \r\n");
            break;
    }
}

void CAM_ISR_Handle(void)
{
    if (GPIO_GetPinsInterruptFlags(CAM_VSYNC_GPIO) & (1U << CAM_VSYNC_GPIO_PIN))
    {
        /* Clear external interrupt flag. */
        GPIO_ClearPinsInterruptFlags(CAM_VSYNC_GPIO, 1U << CAM_VSYNC_GPIO_PIN);

        PORT_SetPinInterruptConfig(CAM_PCLK_PORT, CAM_PCLK_GPIO_PIN,
                kPORT_DMAFallingEdge);
        transferConfig.destAddr = (uint32_t) rb_thisBuf;
        EDMA_SubmitTransfer(&g_EDMA_Handle, &transferConfig);
        EDMA_StartTransfer(&g_EDMA_Handle);
    }
}

static void CAM_eDMA_Callback(edma_handle_t *handle, void *param,
        bool transferDone, uint32_t tcds)
{
    if (transferDone)
    {
        PORT_SetPinInterruptConfig(CAM_PCLK_PORT, CAM_PCLK_GPIO_PIN,
                kPORT_InterruptOrDMADisabled);
        CAM_Buffer_Update();
    }
}
