/**
 * @file    app.c
 * @brief   Application entry point.
 */

/* Freescale includes. */
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_sysmpu.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Devices includes. */
#include "led.h"
#include "imu.h"
#include "motor.h"
#include "oled.h"
#include "camera.h"
#include "sbus.h"
#include "blackbox.h"

/* Others includes. */
#include "control.h"

/* Include internal header to get SEGGER_RTT_CB */
#include "SEGGER_RTT.h"

/* Task priorities. */
#define REALTIME_TASK     (configMAX_PRIORITIES - 1)
#define HIGH_TASK         (configMAX_PRIORITIES - 2)
#define MEDIUM_TASK       (configMAX_PRIORITIES - 3)
#define LOW_TASK          (configMAX_PRIORITIES - 4)

/* Function declaration */
static void demo_Task(void *pvParameters);
static void sensor_Task(void *pvParameters);
static void disp_Task(void *pvParameters);
static void sbus_Task(void *pvParameters);
static void sdcard_Task(void *pvParameters);

/* Public variables */
imuData_t sensor;       //姿态传感器全局变量
mSpeed_t motorInfo;     //电机信息全局变量
sbusChannel_t rcInfo;   //遥控通道全局变量

/*
 * @brief 应用程序入口
 */
int main(void) {
  	/* 初始化板载硬件 */
	BOARD_InitPins();
    BOARD_InitBootClock();
  	/* 初始化调试控制台, JLinkRTT方式 */
    DbgConsole_Init(0, 0, DEBUG_CONSOLE_DEVICE_TYPE_RTT, 0);
    /* 初始化SystemView */
    SEGGER_SYSVIEW_Conf();
    /* 打印欢迎信息 */
    PRINTF("\r\n");
#ifdef NDEBUG
    PRINTF("*************** (Release version) ***************\r\n");
#else
    PRINTF("**************** (Debug version) ****************\r\n");
#endif
    PRINTF("*   Welcome to XinDian-5 Balance Car Project!   *\r\n");
    PRINTF("*************************************************\r\n");

    /* 初始化部分外设 */
	SYSMPU_Enable(SYSMPU, false);
    LED_Config();
    Motor_Config();

    /* 创建线程 */
    xTaskCreate(demo_Task, "Demo", 256U, NULL, LOW_TASK, NULL);
    xTaskCreate(sensor_Task, "Sensors", 256U, NULL, HIGH_TASK, NULL);
    xTaskCreate(disp_Task, "Display", 256U, NULL, LOW_TASK, NULL);
    xTaskCreate(sbus_Task, "Remote", 256U, NULL, HIGH_TASK, NULL);
    xTaskCreate(sdcard_Task, "Storage", 512U, NULL, MEDIUM_TASK, NULL);

    /* 开启内核调度 */
    vTaskStartScheduler();

    while (1);
    return 0;
}

static void demo_Task(void *pvParameters)
{
    extern SEGGER_RTT_CB _SEGGER_RTT;

    PRINTF("RTT block address is: 0x%x \r\n", &_SEGGER_RTT);

    while (1)
    {
        OLED_Printf(0, 0, "Pitch: %6d", (int) (sensor.Pitch * 100.f));
        OLED_Printf(0, 1, "GyroX: %6d", (int) (sensor.GyroX * 100.f));
        OLED_Printf(0, 2, "GyroZ: %6d", (int) (sensor.GyroZ * 100.f));

        OLED_Printf(0, 4, "CH1: %4d", rcInfo.ch[0]);
        OLED_Printf(0, 5, "CH3: %4d", rcInfo.ch[2]);
        OLED_Printf(0, 6, "CH4: %4d", rcInfo.ch[4]);

        Blackbox_Insert(CAM_GetBitmap(), 600);

        char RTT;
        RTT = GETCHAR();
        switch (RTT)
        {
            case 'a':
            {
                PRINTF("\r\n--> Recording...\r\n");
                Blackbox_Start();
                break;
            }
            case 'b':
            {
                PRINTF("\r\n--> Record finished.\r\n");
                Blackbox_Stop();
                break;
            }
        }

//        CAM_ImageExtract(Pixmap);
//        OLED_DrawImage_80x60(Pixmap);

        LED_Green_Toggle();

        vTaskDelay(50);
    }
}

static void sensor_Task(void *pvParameters)
{
    status_t status;

    /* 姿态传感器初始化 */
    status = IMU_Config();
    if(status != kStatus_Success)
    {
        PRINTF("Task suspend with error code %d \r\n", status);
        vTaskSuspend(NULL);
    }
    /* 摄像头初始化 */
    status = CAM_Config();
//    if (status != kStatus_Success)
//    {
//        PRINTF("Task suspend with error code %d \r\n", status);
//        vTaskSuspend(NULL);
//    }
    /* PID控制器初始化 */
    PidControllor_Init();

    while (1)
    {
        status = IMU_Process(&sensor);
        if (status == kStatus_InvalidArgument)
        {
            PRINTF("Task suspend with error code %d \r\n", status);
            vTaskSuspend(NULL);
        }
    }
}

static void disp_Task(void *pvParameters)
{
    status_t status;
    static TickType_t xLastWakeTime;
    static const TickType_t xFrequency = 40;

    xLastWakeTime = xTaskGetTickCount();

    status = OLED_Config();
    if (status != kStatus_Success)
    {
        PRINTF("Task suspend with error code %d \r\n", status);
        vTaskSuspend(NULL);
    }

    while (1)
    {
        //TODO: 在此添加要显示的内容

        OLED_Refresh();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void sbus_Task(void *pvParameters)
{
    status_t status;

    status = Sbus_Config();
    if (status != kStatus_Success)
    {
        PRINTF("Task suspend with error code %d \r\n", status);
        vTaskSuspend(NULL);
    }

    while (1)
    {
        status = Sbus_UpdateRC(&rcInfo);
        if (status == kStatus_Success)
        {
            //TODO: 在此添加接收成功后的处理事件
        }
    }
}

static void sdcard_Task(void *pvParameters)
{
    status_t status;

    status = Blackbox_Config();
    if (status != kStatus_Success)
    {
        PRINTF("Task suspend with error code %d \r\n", status);
        vTaskSuspend(NULL);
    }

    Blackbox_Format();

    while (1)
    {
        Blackbox_Process();
    }
}

/**
  * @brief  FreeRTOS栈溢出警告程序
  * @retval none
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    PRINTF("Warning: %s stack over flow!\r\n", pcTaskName);
}
