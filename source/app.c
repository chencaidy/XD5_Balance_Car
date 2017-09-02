/**
 * @file    app.c
 * @brief   Application entry point.
 */

/* Main includes. */
#include "app.h"
#include "common.h"

/* Task Function declaration */
static void debug_Task(void *pvParameters);
static void image_Task(void *pvParameters);
static void sensor_Task(void *pvParameters);
static void disp_Task(void *pvParameters);
static void sdcard_Task(void *pvParameters);
static void hmi_Task(void *pvParameters);

/* Others Function declaration */
static void PID_Process(void);

/* 线程句柄创建 */
TaskHandle_t debugHandle;
TaskHandle_t imageHandle;
TaskHandle_t senHandle;
TaskHandle_t dispHandle;
TaskHandle_t sdHandle;
TaskHandle_t hmiHandle;

imuData_t sensor;       //姿态传感器全局变量
mSpeed_t motorInfo;     //电机信息全局变量
sbusChannel_t rcInfo;   //遥控通道全局变量

uint8_t Pixmap[60][80] = {0};   //解压后图像

/*
 * @brief 应用程序入口
 */
int main(void)
{
    /* 初始化板载硬件 */
    BOARD_InitPins();
    BOARD_InitBootClock();
    /* 初始化调试控制台, JLinkRTT方式 */
    DbgConsole_Init(0, 0, DEBUG_CONSOLE_DEVICE_TYPE_RTT, 0);
    /* 初始化SystemView */
    SEGGER_SYSVIEW_Conf();
    /* 打印欢迎信息 */
#ifdef NDEBUG
    PRINTF("\r\n*************** (Release version) ***************\r\n");
#else
    PRINTF("\r\n**************** (Debug version) ****************\r\n");
#endif
    PRINTF("*   Welcome to XinDian-5 Balance Car Project!   *\r\n");
    PRINTF("*************************************************\r\n");

    /* 初始化部分外设 */
    SYSMPU_Enable(SYSMPU, false);
    LED_Config();
    Motor_Config();

    /* 创建线程 */
    xTaskCreate(debug_Task, "Debug", 512U, NULL, LOW_TASK, &debugHandle);
    xTaskCreate(image_Task, "Image", 512U, NULL, HIGH_TASK, &imageHandle);
    xTaskCreate(sensor_Task, "Sensor", 384U, NULL, REALTIME_TASK, &senHandle);
    xTaskCreate(disp_Task, "Display", 256U, NULL, LOW_TASK, &dispHandle);
    xTaskCreate(sdcard_Task, "Storage", 512U, NULL, MEDIUM_TASK, &sdHandle);
    xTaskCreate(hmi_Task, "HMI", 512U, NULL, MEDIUM_TASK, &hmiHandle);

    /* 休眠需要传入配置信息的线程，SD卡线程初始化后恢复 */
    vTaskSuspend(imageHandle);
    vTaskSuspend(senHandle);

    /* 开启内核调度 */
    vTaskStartScheduler();

    while (1);
    return 0;
}

static void debug_Task(void *pvParameters)
{
    status_t status;
    extern SEGGER_RTT_CB _SEGGER_RTT;
    char RTT_Char;

    PRINTF("RTT block address is: 0x%x \r\n", &_SEGGER_RTT);

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

        RTT_Char = GETCHAR();
        switch (RTT_Char)
        {
            case '1':
            {
                Speed.I_Error_Start += 1;
                PRINTF("Direction.P = %d / 1000\r\n", (int) Speed.I_Error_Start);
                break;
            }
            case '2':
            {
                Speed.I_Error_Start -= 1;
                PRINTF("Direction.P = %d / 1000\r\n", (int) Speed.I_Error_Start);
                break;
            }
            case '3':
            {
                Direction.D += 0.0001;
                PRINTF("Direction.D = %d / 10000\r\n", (int) (Direction.D * 10000));
                break;
            }
            case '4':
            {
                Direction.D -= 0.0001;
                PRINTF("Direction.D = %d / 10000\r\n", (int) (Direction.D * 10000));
                break;
            }
            default:
            {
                break;
            }
        }
    }
}

static void image_Task(void *pvParameters)
{
    status_t status;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 20;

    xLastWakeTime = xTaskGetTickCount();

    /* 摄像头初始化 */
    status = CAM_Config();
    if (status != kStatus_Success)
    {
        PRINTF("Task suspend with error code %d \r\n", status);
        vTaskSuspend(NULL);
    }

    /* 还原预设配置 */
    CAM_UpdateProfile(&camera);

    while (1)
    {
        CAM_ImageExtract(Pixmap);
        img_start_timer();

        img_brake_scan();
        img_barrier_scan();

        if (Process.Flag == true)
        {
            img_circle_scan();
            img_circle_search();

            img_cross_scan();
//            img_smalls_search();
//            img_cross_search();

            img_find_middle();
            img_Slope_scan();
        }
        else
        {
            img_find_middle_start();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void sensor_Task(void *pvParameters)
{
    status_t status;

    /* 姿态传感器初始化 */
    status = IMU_Config();
    if (status != kStatus_Success)
    {
        PRINTF("Task suspend with error code %d \r\n", status);
        vTaskSuspend(NULL);
    }

    /* 写入偏移量 */
    IMU_SetBias(imuBias);

    /* PID控制器初始化 */
    PidControllor_Init();

    /* 注册PID回调函数 */
    IMU_PID_SetCallback(PID_Process);

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
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 20;

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
        OLED_DrawImage_80x60(Pixmap);

        OLED_Printf(80, 0, "EP:%6d", (int) ((sensor.Pitch - Angle.A_Bias) * 100.f));

        OLED_Refresh();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void sdcard_Task(void *pvParameters)
{
    status_t status;

    status = Blackbox_Config();
    if (status != kStatus_Success)
    {
        /* 启动需要传入配置信息的线程 */
        vTaskResume(imageHandle);
        vTaskResume(senHandle);

        /* 线程休眠 */
        PRINTF("Task suspend with error code %d \r\n", status);
        vTaskSuspend(NULL);
    }

    /* 读取配置信息 */
    Blackbox_ReadConf("/CONFIG/CAM.CF", &camera, sizeof(camera));
    Blackbox_ReadConf("/CONFIG/IMU.CF", &imuBias, sizeof(imuBias));

    /* 启动需要传入配置信息的线程 */
    vTaskResume(imageHandle);
    vTaskResume(senHandle);
    LED_Green(ON);

    while (1)
    {
        Blackbox_Process();
        LED_Green_Toggle();
    }
}

static void hmi_Task(void *pvParameters)
{
    status_t status;
    uint8_t Packge[32];
    uint16_t Length = 0;

    status = HMI_Config();
    if (status != kStatus_Success)
    {
        PRINTF("Task suspend with error code %d \r\n", status);
        vTaskSuspend(NULL);
    }

    while (1)
    {
        HMI_RxHandle();

        status = HMI_GetOnePacket(Packge, &Length);
        if (status == kStatus_Success)
        {
            HMI_RxMsgHandle(Packge);
            LED_Yellow_Toggle();
        }
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

/**
  * @brief  PID处理函数
  * @retval none
  */
static void PID_Process(void)
{
    static int i = 0;

    Motor_GetCnt(&motorInfo);

//    Speed.Goal = (rcInfo.ch[2] - 299) * 4;

    Speed_Control(motorInfo.cntL, motorInfo.cntR);
    Angle_Control(sensor.Pitch, sensor.GyroX);
    Direction_Control(Process.Turn, Barrier.Offset, sensor.GyroZ);
//    Direction_Control((rcInfo.ch[0]-1000)/20, 0, sensor.GyroZ);
    Motor_Control(&motorInfo.pwmL, &motorInfo.pwmR);

    Failsafe_Control(sensor.Pitch, 0);

    if(rcInfo.ch[4] != 0)
    {
        if (rcInfo.ch[4] < 1000)
        {
            motorInfo.pwmL = 0;
            motorInfo.pwmR = 0;
            Failsafe.ON = false;
        }
        else
        {
            Failsafe.ON = true;
        }
    }

    Motor_ChangeDuty(motorInfo);

    if (rcInfo.ch[5] > 1024)
    {
        Blackbox_SYNC();
        i++;
        if(i == 4)
        {
            Blackbox_CIR(CAM_GetBitmap(), 600);
            i = 0;
        }
 //       Blackbox_DDR(0, &Circle.Count, UINT32);
        Blackbox_DDR(0, &Angle.PWM, FLOAT);
        Blackbox_DDR(1, &Speed.PWM, FLOAT);
        Blackbox_DDR(2, &Direction.PWM, FLOAT);
        Blackbox_DDR(3, &sensor.Pitch, FLOAT);
        Blackbox_DDR(4, &sensor.GyroX, FLOAT);
        Blackbox_DDR(5, &sensor.GyroZ, FLOAT);
        Blackbox_DDR(6, &Process.Turn, FLOAT);
//        Blackbox_DDR(6, &motorInfo.pwmL, INT8);
//        Blackbox_DDR(7, &motorInfo.pwmR, INT8);
    }
}
