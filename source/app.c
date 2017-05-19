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
#include "hmi.h"

/* Others includes. */
#include "control.h"
#include "hmiHandle.h"

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
static void hmi_Task(void *pvParameters);

static void PID_Process(void);

/* Public variables */
imuData_t sensor;       //姿态传感器全局变量
mSpeed_t motorInfo;     //电机信息全局变量
sbusChannel_t rcInfo;   //遥控通道全局变量

uint8_t Pixmap[60][80] = {0};   //解压后图像

extern carAngle_t Angle;            //PID控制器 - 平衡环参数
extern carSpeed_t Speed;            //PID控制器 - 速度环参数
extern carDirection_t Direction;    //PID控制器 - 转向环参数
extern carMotor_t Motor;            //PID控制器 - 电机输出参数

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
    xTaskCreate(demo_Task, "Demo", 512U, NULL, LOW_TASK, NULL);
    xTaskCreate(sensor_Task, "Sensors", 384U, NULL, REALTIME_TASK, NULL);
    xTaskCreate(disp_Task, "Display", 256U, NULL, LOW_TASK, NULL);
    xTaskCreate(sbus_Task, "Remote", 256U, NULL, HIGH_TASK, NULL);
    xTaskCreate(sdcard_Task, "Storage", 512U, NULL, MEDIUM_TASK, NULL);
    xTaskCreate(hmi_Task, "HMI", 512U, NULL, MEDIUM_TASK, NULL);

    /* 开启内核调度 */
    vTaskStartScheduler();

    while (1);
    return 0;
}

#define CAMERA_W        (80U)
#define CAMERA_H        (60U)
#define CAMERA_ML       (CAMERA_W/2-1)
#define CAMERA_MR       (CAMERA_W/2)
#define CAMERA_MT       (CAMERA_W-1)
#define CAMERA_HT       (CAMERA_H-1)        //预编译能节省CPU时间

int8_t offset;

void Algorithm_Bak(void)
{
    uint8_t h, w;        //当前帧 高,宽 计次
    uint8_t a = 0, a_max = 0;    //当前帧视距
    int8_t b = 0, c = 0;    //左线长度，右线长度

    CAM_ImageExtract(Pixmap);

    //获取前方视距，提供分段 PD依据
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

    //获取左线长度
    for (w = 0; w < CAMERA_MR; w++)
    {
        if (Pixmap[CAMERA_HT - w][CAMERA_ML - w] == 0xFF)
        {
            b++;
        }
        else
        {
            break;
        }
    }
    //获取右线长度
    for (w = 0; w < CAMERA_MR; w++)
    {
        if (Pixmap[CAMERA_HT - w][CAMERA_MR + w] == 0xFF)
        {
            c++;
        }
        else
        {
            break;
        }
    }

    //超出视距校正
    if (b == 0 || c == 0)
    {
        for (w = 0; w < CAMERA_MR; w++)
        {
            if (Pixmap[CAMERA_HT][CAMERA_ML - w] == 0xFF)
            {
                b++;
            }
            if (Pixmap[CAMERA_HT][CAMERA_MR + w] == 0xFF)
            {
                c++;
            }
        }
    }

    //计算偏差
    offset = b - c;

    OLED_Printf(80, 4, "S1:%6d", a_max);
    OLED_Printf(80, 5, "S3:%6d", b);
    OLED_Printf(80, 6, "S5:%6d", c);

    if (rcInfo.ch[9] > 1024)
    {
        Blackbox_SYNC();
        Blackbox_CIR(CAM_GetBitmap(), 600);
    }
}

/**
  * @brief  PID处理函数
  * @retval none
  */
static void PID_Process(void)
{
    Motor_GetCnt(&motorInfo);

    Speed.Goal = (rcInfo.ch[2] - 299) * 4;

    Speed_Control(motorInfo.cntL, motorInfo.cntR);
    Angle_Control(sensor.Pitch, sensor.GyroX);
    Direction_Control(-offset, sensor.GyroZ);
//    Direction_Control((rcInfo.ch[0]-1000)/20, sensor.GyroZ);
    Motor_Control(&motorInfo.pwmL, &motorInfo.pwmR);

    if (rcInfo.ch[4] < 1000)
    {
        motorInfo.pwmL = 0;
        motorInfo.pwmR = 0;
    }

    Motor_ChangeDuty(motorInfo);

//    if (rcInfo.ch[9] > 1024)
//    {
//        Blackbox_SYNC();
//        Blackbox_DDR(0, &Angle.PWM, FLOAT);
//        Blackbox_DDR(1, &Speed.PWM, FLOAT);
//        Blackbox_DDR(2, &Direction.PWM, FLOAT);
//        Blackbox_DDR(3, &rcInfo.ch[2], UINT16);
//        Blackbox_DDR(4, &offset, INT8);
//        Blackbox_DDR(0, &sensor.GyroX, FLOAT);
//        Blackbox_DDR(1, &sensor.GyroZ, FLOAT);
//    }
}

static void demo_Task(void *pvParameters)
{
    extern SEGGER_RTT_CB _SEGGER_RTT;

    PRINTF("RTT block address is: 0x%x \r\n", &_SEGGER_RTT);

    while (1)
    {
        Algorithm_Bak();

        char RTT;
        RTT = GETCHAR();
        switch (RTT)
        {
            case 'i':
            {
                PRINTF("\r\n");
                PRINTF("Angle.P = %d.%d | Angle.D = %d \r\n", (int) Angle.P,
                        ((int) (Angle.P * 10)) % 10, (int) (Angle.D * 1000));
                PRINTF("Dead.L = %d | Dead.R = %d \r\n",
                        (int) (Motor.Dead_L * 10), (int) (Motor.Dead_R * 10));
                break;
            }
            default:
            {
//                IMU_HandleInput(RTT);
                break;
            }
        }

        vTaskDelay(20);
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
    if (status != kStatus_Success)
    {
        PRINTF("Task suspend with error code %d \r\n", status);
        vTaskSuspend(NULL);
    }

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
        OLED_DrawImage_80x60(Pixmap);

        OLED_Printf(80, 0, "EP:%6d", (int) ((sensor.Pitch - Angle.A_Bias) * 100.f));
        OLED_Printf(80, 1, "GX:%6d", (int) ((sensor.GyroX - Angle.G_Bias) * 100.f));
        OLED_Printf(80, 2, "GZ:%6d", (int) ((sensor.GyroZ - Direction.G_Bias) * 100.f));

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
    else
        LED_Green(ON);

    /* 读取设备配置信息 */
    camConf_t camera;
    Blackbox_ReadConf("cam.cf", &camera, sizeof(camera));
    CAM_UpdateProfile(&camera);

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
        if(status == kStatus_Success)
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
