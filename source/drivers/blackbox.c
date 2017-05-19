/*
 * blackbox.c
 *
 *  Created on: 2017年2月9日
 *      Author: chen
 */


/* Includes ------------------------------------------------------------------*/
#include "blackbox.h"

#include "fsl_debug_console.h"
#include "fsl_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "ff.h"
#include "diskio.h"

#include <stdio.h>

/* Definitions ---------------------------------------------------------------*/
#define SDCARD_CD_GPIO         (GPIOE)        //SD卡插入检测引脚
#define SDCARD_CD_GPIO_PIN     (6U)

#define SDHC_BASEADDR          (SDHC)
#define SDHC_CLKSRC            (kCLOCK_CoreSysClk)
#define SDHC_IRQ               (SDHC_IRQn)

/* 乒乓缓冲区定义 */
#define PIN     (0U)
#define PON     (1U)

#define BUFFER_SIZE     (4096U)

/* Variables -----------------------------------------------------------------*/
uint8_t PinBuffer[BUFFER_SIZE];     //乒乓缓冲区
uint8_t PonBuffer[BUFFER_SIZE];     //乒乓缓冲区
blackbox_Info_t bufManager;         //缓冲区管理器
SemaphoreHandle_t isEvent;          //缓冲区操作事件

static FATFS g_fileSystem;             // File system object
static FIL g_fileObject;               // File object
static const TCHAR driverNumberBuffer[3U] = { SDDISK + '0', ':', '/' };


/* Codes ---------------------------------------------------------------------*/
/**
  * @brief  SD卡插入检测引脚初始化
  * @retval None
  */
static status_t SDCard_GPIO_Config(void)
{
    bool select;
    gpio_pin_config_t test_config =
    { kGPIO_DigitalInput, 0, };

    /* 设置 GPIO Input 模式 */
    GPIO_PinInit(SDCARD_CD_GPIO, SDCARD_CD_GPIO_PIN, &test_config);

    /* 读取 GPIO 状态 */
    select = GPIO_ReadPinInput(SDCARD_CD_GPIO, SDCARD_CD_GPIO_PIN);

    if (select == true)
        return kStatus_Fail;
    else
        return kStatus_Success;
}

/**
  * @brief  黑匣子初始化
  * @retval None
  */
status_t Blackbox_Config(void)
{
    status_t status;
    FRESULT s_fat;
    FATFS *pfs;
    DWORD fre_clust, fre_sect, tot_sect;

    NVIC_SetPriority(SDHC_IRQ, 6);

    status = SDCard_GPIO_Config();
    if (status != kStatus_Success)
    {
        PRINTF("BlackBox: Device invalid, please insert card.\r\n");
        return status;
    }

    /* 等待SD卡上电完成 */
    vTaskDelay(1000U);

    /* 挂载FATFS文件系统 */
    s_fat = f_mount(&g_fileSystem, driverNumberBuffer, 1U);
    if (s_fat != FR_OK)
    {
        PRINTF("FATFS: Mount volume failed, code %d\r\n", s_fat);
        return kStatus_Fail;
    }

#if (_FS_RPATH >= 2U)
    s_fat = f_chdrive((char const *) &driverNumberBuffer[0U]);
    if (s_fat != FR_OK)
    {
        PRINTF("FATFS: Change drive failed, code %d\r\n", s_fat);
        return kStatus_Fail;
    }
#endif

    /* 获取剩余空间 */
    pfs = &g_fileSystem;
    s_fat = f_getfree(driverNumberBuffer, &fre_clust, &pfs);
    if (s_fat != FR_OK)
    {
        PRINTF("FATFS: Get free size failed, code %d\r\n", s_fat);
        return kStatus_Fail;
    }
    else
    {
        tot_sect = ((pfs->n_fatent - 2) * pfs->csize) / 2048;
        fre_sect = (fre_clust * pfs->csize) / 2048;
        PRINTF("BlackBox: Mounted. Total %dMB, Free %dMB.\r\n", tot_sect, fre_sect);
    }

    /* 创建信号量 */
    isEvent = xSemaphoreCreateCounting(2, 0);    //创建计数信号量，队列深度2
    if (isEvent == NULL)
    {
        PRINTF("BlackBox: Semaphore create failed.\r\n");
        return kStatus_Fail;
    }

    /* 初始化缓冲区 */
    bufManager.Pin = PinBuffer;
    bufManager.Pon = PonBuffer;

    bufManager.PinRemain = BUFFER_SIZE;    //复位PIN剩余空间
    bufManager.PonRemain = BUFFER_SIZE;    //复位PON剩余空间

    bufManager.thisBuf = PIN;       //当期操作PIN缓冲区
    bufManager.isStart = false;     //停止记录

    return kStatus_Success;
}

/**
  * @brief  文件系统格式化
  * @retval None
  */
status_t Blackbox_Format(void)
{
    FRESULT s_fat;
    BYTE work[_MAX_SS];

    if(bufManager.isStart == true)     //检测到还在记录数据，禁止格式化
    {
        PRINTF("BlackBox: Please stop data record before format.");
        return kStatus_Fail;
    }

    PRINTF("BlackBox: Formatting...\r\n");

    s_fat = f_mkfs(driverNumberBuffer, FM_FAT32, 0U, work, sizeof(work));
    if (s_fat != FR_OK)
    {
        PRINTF("FATFS: Make file system failed, code %d\r\n", s_fat);
        return kStatus_Fail;
    }

    PRINTF("BlackBox: Formatted with success.\r\n");

    return kStatus_Success;
}

/**
  * @brief  打开文件
  * @retval None
  */
static status_t Blackbox_Open(char* file)
{
    FRESULT error = kStatus_Success;

    error = f_open(&g_fileObject, (const TCHAR*) file, (FA_WRITE | FA_CREATE_ALWAYS));
    if (error)
        PRINTF("FATFS: Open file failed, code %d \r\n", error);

    return error;
}

/**
  * @brief  关闭文件
  * @retval None
  */
static status_t Blackbox_Close(void)
{
    FRESULT error;

    error = f_close(&g_fileObject);
    if (error)
    {
        PRINTF("FATFS: Close file failed, code %d \r\n", error);
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/**
  * @brief  写入一段数据
  * @retval None
  */
static status_t Blackbox_Write(void *buf, uint32_t len)
{
    FRESULT error;
    UINT bytesWritten;

    error = f_write(&g_fileObject, buf, len, &bytesWritten);

    if ((error) || (bytesWritten != len))
    {
        PRINTF("FATFS: Write file failed, code %d \r\n", error);
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/**
  * @brief  开始数据记录
  * @retval None
  */
status_t Blackbox_Start(void)
{
    status_t status = kStatus_Success;
    FRESULT error = FR_OK;
    uint32_t fileNum = 1;
    char fileName[9];

    /* 防止重复Start */
    if (bufManager.isStart == true)
    {
        PRINTF("BlackBox: Already run.\r\n");
        return kStatus_Fail;
    }

    /* 读取上一次记录的文件号 */
    error = Blackbox_ReadConf("XD5.NUM", &fileNum, sizeof(uint32_t));
    if (error == FR_OK)
        fileNum++;

    /* 生成文件名 */
    Blackbox_WriteConf("XD5.NUM", &fileNum, sizeof(uint32_t));
    sprintf(fileName, "D%03d.XD5", fileNum);
    fileName[8] = '\0';

    /* 打开文件 */
    status = Blackbox_Open(fileName);
    if (status != kStatus_Success)
    {
        bufManager.isStart = false;
        return status;
    }

    /* 置 isStart 标志 */
    bufManager.isStart = true;

    return status;
}

/**
  * @brief  停止数据记录
  * @retval None
  */
status_t Blackbox_Stop(void)
{
    /* 防止重复Stop */
    if(bufManager.isStart == false)
    {
        PRINTF("BlackBox: Already idle...\r\n");
        return kStatus_Fail;
    }

    /* 复位 isStart 标志 */
    bufManager.isStart = false;
    /* 置信号量，写入剩余缓冲区数据 */
    xSemaphoreGive(isEvent);

    return kStatus_Success;
}

/**
  * @brief  复位文件号记录
  * @retval None
  */
status_t Blackbox_Reset(void)
{
    status_t status = kStatus_Success;
    uint32_t fileNum = 0;

    status = Blackbox_WriteConf("XD5.NUM", &fileNum, sizeof(uint32_t));
    if (status == kStatus_Success)
        PRINTF("BlackBox: Point reset to \"D001.XD5\".\r\n");

    return status;
}

/**
  * @brief  向缓冲区插入数据
  * @param  [in]  *buf： 待写入数组的指针
  * @param  [in]   len： 写入数据长度
  * @retval status
  */
static status_t Blackbox_Insert(void *buf, uint16_t len)
{
    uint16_t lenFix;

    /* 判断是否启动记录 */
    if(bufManager.isStart == false)
        return kStatus_Fail;

    /* 判断是否超出最大允许长度（单缓冲长度） */
    if(len > BUFFER_SIZE)
    {
        PRINTF("BlackBox: Insert length is out of range.\r\n");
        return kStatus_OutOfRange;
    }

    /* 判断信号量是否成功创建 */
    if (isEvent == NULL)
    {
        PRINTF("BlackBox: Semaphore create failed.\r\n");
        return kStatus_InvalidArgument;
    }

    /* 开始写入缓冲区 */
    lenFix = len;
    while(lenFix)
    {
        if(bufManager.PinRemain == 0 && bufManager.PonRemain == 0)
        {
            PRINTF("BlackBox: PIN and PON buffer are full.\r\n");
            return kStatus_Fail;
        }

        if(bufManager.thisBuf == PIN)
        {
            if(bufManager.PinRemain > lenFix)   //剩余空间大于写入长度，直接拷贝
            {
                memcpy(bufManager.Pin + BUFFER_SIZE - bufManager.PinRemain,
                        buf + len - lenFix, lenFix);

                bufManager.PinRemain -= lenFix;    //更新管理器的PIN剩余空间
                lenFix = 0;     //所有数据成功写入缓冲区
                break;
            }
            else    //剩余空间不足，先写满PIN缓冲
            {
                memcpy(bufManager.Pin + BUFFER_SIZE - bufManager.PinRemain,
                        buf + len - lenFix, bufManager.PinRemain);

                lenFix = len - bufManager.PinRemain;     //计算将写入PON缓冲的字节数
                bufManager.PinRemain = 0;   //更新管理器的PIN剩余空间
                bufManager.thisBuf = PON;   //设置可写缓冲区为PON

                xSemaphoreGive(isEvent);     //PIN缓冲区满，置信号量
            }
        }
        else if(bufManager.thisBuf == PON)
        {
            if(bufManager.PonRemain > lenFix)   //剩余空间大于写入长度，直接拷贝
            {
                memcpy(bufManager.Pon + BUFFER_SIZE - bufManager.PonRemain,
                        buf + len - lenFix, lenFix);

                bufManager.PonRemain -= lenFix;    //更新管理器的PON剩余空间
                lenFix = 0;     //所有数据成功写入缓冲区
                break;
            }
            else    //剩余空间不足，先写满PON缓冲
            {
                memcpy(bufManager.Pon + BUFFER_SIZE - bufManager.PonRemain,
                        buf + len - lenFix, bufManager.PonRemain);

                lenFix = len - bufManager.PonRemain;     //计算将写入PIN缓冲的字节数
                bufManager.PonRemain = 0;   //更新管理器的PON剩余空间
                bufManager.thisBuf = PIN;   //设置可写缓冲区为PIN

                xSemaphoreGive(isEvent);     //PON缓冲区满，置信号量
            }
        }
    }

    return kStatus_Success;
}

/**
  * @brief  发送缓冲区数据
  * @retval None
  */
status_t Blackbox_Process(void)
{
    /* 等待事件发生信号 */
    if (xSemaphoreTake(isEvent, portMAX_DELAY) != pdTRUE)
        return kStatus_NoTransferInProgress;

    /* PIN、PON都不满 */
    if (bufManager.PinRemain != 0 && bufManager.PonRemain != 0)
    {
        //TODO: 加入处理程序
    }
    /* PIN满，PON不满，写入PIN */
    if (bufManager.PinRemain == 0 && bufManager.PonRemain != 0)
    {
        Blackbox_Write(bufManager.Pin, BUFFER_SIZE);
        bufManager.PinRemain = BUFFER_SIZE;  // 复位容量
    }
    /* PON满，PIN不满，写入PON */
    if (bufManager.PinRemain != 0 && bufManager.PonRemain == 0)
    {
        Blackbox_Write(bufManager.Pon, BUFFER_SIZE);
        bufManager.PonRemain = BUFFER_SIZE;
    }
    /* PIN、PON都满 */
    if (bufManager.PinRemain == 0 && bufManager.PonRemain == 0)
    {
        //TODO: 加入处理程序
    }

    /* 检测到停止信号 */
    if (bufManager.isStart == false)
    {
        /* PIN、PON都有数据 */
        if (bufManager.PinRemain != BUFFER_SIZE && bufManager.PonRemain != BUFFER_SIZE)
        {
            //TODO: 加入处理程序
        }
        /* PIN缓冲区有数据， 写完剩余数据 */
        if (bufManager.PinRemain != BUFFER_SIZE && bufManager.PonRemain == BUFFER_SIZE)
        {
            Blackbox_Write(bufManager.Pin, BUFFER_SIZE - bufManager.PinRemain);
            /* 复位PIN容量 */
            bufManager.PinRemain = BUFFER_SIZE;
        }
        /* PON缓冲区有数据，写完剩余数据 */
        if (bufManager.PinRemain == BUFFER_SIZE && bufManager.PonRemain != BUFFER_SIZE)
        {
            Blackbox_Write(bufManager.Pon, BUFFER_SIZE - bufManager.PonRemain);
            /* 复位PON容量 */
            bufManager.PonRemain = BUFFER_SIZE;
        }
        /* PIN、PON都无数据 */
        if (bufManager.PinRemain == BUFFER_SIZE && bufManager.PonRemain == BUFFER_SIZE)
        {
            //TODO: 加入处理程序
            Blackbox_Close();
        }
    }

    return kStatus_Success;
}

/**
  * @brief  写一个配置文件
  * @retval None
  */
status_t Blackbox_WriteConf(char* file, void* conf, uint32_t len)
{
    FRESULT error = kStatus_Success;
    UINT bw;

    error = f_open(&g_fileObject, (const TCHAR*) file, (FA_CREATE_ALWAYS | FA_WRITE));
    if (error)
        PRINTF("FATFS: Open file failed, code %d \r\n", error);

    error = f_write(&g_fileObject, "CONF", 4, &bw);
    if ((error) || (bw != 4))
        PRINTF("FATFS: Write file failed, code %d \r\n", error);

    f_write(&g_fileObject, &len, sizeof(uint32_t), &bw);
    f_write(&g_fileObject, conf, len, &bw);

    f_close(&g_fileObject);

    return error;
}

/**
  * @brief  写一个配置文件
  * @retval None
  */
status_t Blackbox_ReadConf(char* file, void* conf, uint32_t len)
{
    FRESULT error;
    UINT br;
    char head[4];
    uint32_t size;

    if (conf == NULL || len == 0)
        return kStatus_InvalidArgument;

    error = f_open(&g_fileObject, (const TCHAR*) file, (FA_READ));
    if (error)
        PRINTF("FATFS: Open file failed, code %d \r\n", error);

    error = f_read(&g_fileObject, head, sizeof(head), &br);
    if ((error) || (br != 4))
        PRINTF("FATFS: Read file failed, code %d \r\n", error);

    f_read(&g_fileObject, &size, sizeof(uint32_t), &br);

    if (head[0] == 'C' && head[1] == 'O' && head[2] == 'N' && head[3] == 'F' && size == len)
        f_read(&g_fileObject, conf, size, &br);
    else
        PRINTF("FATFS: File checksum error. \r\n");

    f_close(&g_fileObject);

    return error;
}

/**
  * @brief  行驶数据记录器（Drive Data Recorder）
  * @param   ch[in]: 记录到哪个通道（0-15）
  * @param  *val[in]: 记录的数据指针
  * @param  type[in]: 数据类型
  * @retval None
  */
status_t Blackbox_DDR(uint8_t ch, void* val, DDR_Type_e type)
{
    status_t status = kStatus_Success;
    uint8_t len = 0;

    switch (type)
    {
        case INT8:
        case UINT8:
        {
            len = 1;
            break;
        }
        case INT16:
        case UINT16:
        {
            len = 2;
            break;
        }
        case INT32:
        case UINT32:
        case FLOAT:
        {
            len = 4;
            break;
        }
        case DOUBLE:
        {
            len = 8;
            break;
        }
        default:
        {
            break;
        }
    }

    status = Blackbox_Insert("_DDR", 4);
    status = Blackbox_Insert(&ch, sizeof(uint8_t));
    status = Blackbox_Insert(&type, sizeof(uint8_t));
    status = Blackbox_Insert(val, len);

    return status;
}

/**
  * @brief  第一视角图像记录器（Cockpit Image Recorder）
  * @retval None
  */
status_t Blackbox_CIR(uint8_t* bitmap, uint32_t len)
{
    FRESULT status = kStatus_Success;

    status = Blackbox_Insert("_CIR", 4);
    status = Blackbox_Insert(&len, sizeof(uint32_t));
    status = Blackbox_Insert(bitmap, len);

    return status;
}

/**
  * @brief  帧同步器
  * @retval None
  */
status_t Blackbox_SYNC(void)
{
    FRESULT status = kStatus_Success;

    status = Blackbox_Insert("_SYNC", 5);

    return status;
}

