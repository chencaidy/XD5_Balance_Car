/*
 * sensor.c
 *
 *  Created on: 2017年2月3日
 *      Author: chen
 */


/* Includes ------------------------------------------------------------------*/
#include "imu.h"

#include "FreeRTOS.h"
#include "task.h"

#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
#include "fsl_i2c_freertos.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "eMPL_outputs.h"

#include <math.h>

/* Definitions ---------------------------------------------------------------*/
/* GPIO configuration. */
#define IMU_LED_GPIO            (GPIOB)
#define IMU_LED_GPIO_PIN        (10U)

#define IMU_INT_GPIO            (GPIOB)
#define IMU_INT_GPIO_PIN        (9U)
#define IMU_INT_PORT            (PORTB)
#define IMU_INT_IRQN            (PORTB_IRQn)

#define IMU_SCL_PORT            (PORTB)
#define IMU_SCL_GPIO            (GPIOB)
#define IMU_SCL_GPIO_PIN        (2U)

/* I2C configuration. */
#define IMU_I2C_MASTER              (I2C0)
#define IMU_I2C_MASTER_IRQN         (I2C0_IRQn)
#define I2C_MASTER_CLK_SRC          (I2C0_CLK_SRC)

#define I2C_MASTER_SLAVE_ADDR_7BIT  (0x68U)      /* MPU9250 addr is 0x68 */
#define I2C_BAUDRATE                (400000)     /* 400Khz */

/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (200)

#define ACCEL_SENS      (16384.0f)
#define GYRO_SENS       (16.375f)
#define QUAT_SENS       (1073741824.0f)

struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
};

struct imuConfig_s
{
    /* 中断信号量 */
    SemaphoreHandle_t ready;
    /* PID回调函数 */
    void (*pidCallback)(void);
};

/* eMPL Private typedef ------------------------------------------------------*/
static struct hal_s hal = {0};
static struct imuConfig_s imuParam = {0};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};

/* Device Private typedef ----------------------------------------------------*/
static i2c_rtos_handle_t master_rtos_handle;
static i2c_master_config_t masterConfig;
static i2c_master_transfer_t masterXfer;

/* Define the init structure for the input INT1 pin */
static gpio_pin_config_t imu_int_config = {
    kGPIO_DigitalInput, 0,
};

/* Define the init structure for the output INT2 pin */
static gpio_pin_config_t imu_led_config = {
    kGPIO_DigitalOutput, 0,
};

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
    unsigned char mask = 0;
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON)
        mask |= INV_XYZ_GYRO;
    /* If you need a power transition, this function should be called with a
     * mask of the sensors still enabled. The driver turns off any sensors
     * excluded from this mask.
     */
    mpu_set_sensors(mask);
    if (!hal.dmp_on)
        mpu_configure_fifo(mask);
}

static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];
    unsigned char i = 0;

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        for(i = 0; i<3; i++) {
            gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
            accel[i] *= 2048.f; //convert to +-16G
            accel[i] = accel[i] >> 16;
            gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
    }
}

/* User Codes ----------------------------------------------------------------*/
/**
  * @brief  获取系统时间（毫秒）
  * @param [out]  *count： 计数变量指针
  * @retval none
  */
void Sensor_GetTimeStamp(long unsigned int *count)
{
    *count = (long unsigned int) xTaskGetTickCount();
}

/**
  * @brief  I2C写寄存器
  * @param  [in] slave_addr： 从机地址
  * @param  [in]   reg_addr： 寄存器地址
  * @param  [in]        len： 写入数据长度
  * @param  [in]  *data_ptr： 数据数组指针
  * @retval status
  */
status_t Sensor_I2C_WriteRegister(uint8_t slave_addr, uint8_t reg_addr, uint16_t len, const uint8_t *data_ptr)
{
    status_t status;

    masterXfer.slaveAddress = slave_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = (uint32_t)reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = (uint8_t *)data_ptr;
    masterXfer.dataSize = len;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(&master_rtos_handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("I2C master: Error during write transaction, %d\r\n", status);
        return status;
    }
    else
    {
        return kStatus_Success;
    }
}

/**
  * @brief  I2C读寄存器
  * @param  [in] slave_addr： 从机地址
  * @param  [in]   reg_addr： 寄存器地址
  * @param  [in]        len： 读出数据长度
  * @param [out]  *data_ptr： 数据数组指针
  * @retval status
  */
status_t Sensor_I2C_ReadRegister(uint8_t slave_addr, uint8_t reg_addr, uint16_t len, uint8_t *data_ptr)
{
    status_t status;

    masterXfer.slaveAddress = slave_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = (uint32_t)reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = (uint8_t *)data_ptr;
    masterXfer.dataSize = len;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(&master_rtos_handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("I2C master: Error during read transaction, %d\r\n", status);
        return status;
    }
    else
    {
        return kStatus_Success;
    }
}

/**
  * @brief  解除I2C死锁状态
  * @param  [in]  *port： PORT地址
  * @param  [in]  *gpio： GPIO地址
  * @param  [in]    pin： 引脚号
  * @retval none
  */
static void Sensor_I2C_Unlock(PORT_Type *port, GPIO_Type *gpio, uint32_t pin)
{
    /* 临时设置端口复用为GPIO */
    PORT_SetPinMux(port, pin, kPORT_MuxAsGpio);
    /* 设置 GPIO Output 模式 */
    gpio->PDDR |= (1U << pin);
    /* 连续发送 9个 SCL脉冲解除总线死锁状态 */
    for(uint8_t i=18; i>0; i--)
    {
        __NOP();
        gpio->PTOR = (1U << pin);
    }
    /* 恢复所有复用状态 */
    void BOARD_InitPins(void);
    BOARD_InitPins();
}

/**
  * @brief  惯性测量单元I2C接口初始化
  * @retval status
  */
static status_t Sensor_I2C_Config(void)
{
    uint32_t sourceClock;
    status_t status;
    uint8_t WHO_AM_I;

    /* 先进行I2C解锁，防止因MCU复位导致的死锁 */
    Sensor_I2C_Unlock(IMU_SCL_PORT, IMU_SCL_GPIO, IMU_SCL_GPIO_PIN);

    NVIC_SetPriority(IMU_I2C_MASTER_IRQN, 6);

    /*
     * masterConfig.baudRate_Bps = I2C_BAUDRATE;
     * masterConfig.enableHighDrive = false;
     * masterConfig.enableStopHold = false;
     * masterConfig.glitchFilterWidth = 0U;
     * masterConfig.enableMaster = true;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;
    sourceClock = CLOCK_GetFreq(I2C_MASTER_CLK_SRC);

    status = I2C_RTOS_Init(&master_rtos_handle, IMU_I2C_MASTER, &masterConfig, sourceClock);
    if (status != kStatus_Success)
    {
        PRINTF("I2C master: Error during init, %d \r\n", status);
        return status;
    }

    memset(&masterXfer, 0, sizeof(masterXfer));

    Sensor_I2C_ReadRegister(I2C_MASTER_SLAVE_ADDR_7BIT, 0x75, 1, &WHO_AM_I);
    if(WHO_AM_I != 0x71)
    {
        PRINTF("I2C master: Chip ID check failed, return 0x%x \r\n", WHO_AM_I);
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/**
  * @brief  惯性测量单元eMPL库初始化
  * @retval status
  */
static status_t Sensor_eMPL_Config(void)
{
    status_t result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;

    /* Set up gyro.
     * Every function preceded by mpu_ is a driver function and can be found
     * in inv_mpu.h.
     */
    result = mpu_init(NULL);
    if (result) {
        PRINTF("IMU: Could not initialize gyro.\r\n");
        return kStatus_Fail;
    }

    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.report = PRINT_ACCEL | PRINT_GYRO | PRINT_QUAT;

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    dmp_register_tap_cb(NULL);
    dmp_register_android_orient_cb(NULL);
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     */
    hal.dmp_features =
            DMP_FEATURE_6X_LP_QUAT |
            DMP_FEATURE_TAP |
//            DMP_FEATURE_ANDROID_ORIENT |
//            DMP_FEATURE_GYRO_CAL |
            DMP_FEATURE_SEND_RAW_ACCEL |
            DMP_FEATURE_SEND_CAL_GYRO;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;

    return kStatus_Success;
}

/**
  * @brief  惯性测量单元GPIO初始化（板载INT1, INT2）
  * @retval none
  */
static void Sensor_GPIO_Config(void)
{
    /* Init input INT1 GPIO. */
    NVIC_SetPriority(IMU_INT_IRQN, 6);

    PORT_SetPinInterruptConfig(IMU_INT_PORT, IMU_INT_GPIO_PIN, kPORT_InterruptFallingEdge);
    EnableIRQ(IMU_INT_IRQN);

    GPIO_PinInit(IMU_INT_GPIO, IMU_INT_GPIO_PIN, &imu_int_config);

    /* Init output INT2 GPIO. */
    GPIO_PinInit(IMU_LED_GPIO, IMU_LED_GPIO_PIN, &imu_led_config);
}

/**
  * @brief  惯性测量单元初始化
  * @param  [in]   *ptr： 待传入的专用结构体
  * @retval status
  */
status_t IMU_Config(void)
{
    status_t status;

    /* 创建二值信号量 */
    imuParam.ready = xSemaphoreCreateBinary();
    if (imuParam.ready == NULL)
    {
        PRINTF("IMU: Semaphore create failed.\r\n");
        return kStatus_Fail;
    }

    /* GPIO 初始化 */
    Sensor_GPIO_Config();

    /* 等待Sensor上电完成 */
    vTaskDelay(100);

    /* I2C 初始化 */
    status = Sensor_I2C_Config();
    if (status != kStatus_Success)
        return status;

    /* 姿态库初始化 */
    status = Sensor_eMPL_Config();
    if (status != kStatus_Success)
        return status;

    /* 运行姿态校正 */
    run_self_test();

    PRINTF("IMU: Configuration with success, task run.\r\n");
    return kStatus_Success;
}

/**
  * @brief  惯性测量单元数据处理函数
  * @retval none
  */
status_t IMU_Process(imuData_t *ptr)
{
    unsigned long sensor_timestamp;

    /* 等待数据就绪信号 */
    if (xSemaphoreTake(imuParam.ready, portMAX_DELAY) != pdTRUE)
        return kStatus_Fail;

    /* 检测形参是否传入 */
    if (ptr == NULL)
    {
       PRINTF("IMU: 'imuData_t' is NULL.\r\n");
       return kStatus_InvalidArgument;
    }

    if (!hal.sensors)
        return kStatus_Fail;

    if (hal.dmp_on)
    {
        short gyro[3], accel[3], sensors;
        unsigned char more;
        long quat[4];
        /* This function gets new data from the FIFO when the DMP is in
         * use. The FIFO can contain any combination of gyro, accel,
         * quaternion, and gesture data. The sensors parameter tells the
         * caller which data fields were actually populated with new data.
         * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
         * the FIFO isn't being filled with accel data.
         * The driver parses the gesture data to determine if a gesture
         * event has occurred; on an event, the application will be notified
         * via a callback (assuming that a callback function was properly
         * registered). The more parameter is non-zero if there are
         * leftover packets in the FIFO.
         */
        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
        if (more)
            xSemaphoreGive(imuParam.ready);     //FIFO中还有数据，继续置位

        /* Gyro and accel data are written to the FIFO by the DMP in chip
         * frame and hardware units. This behavior is convenient because it
         * keeps the gyro and accel outputs of dmp_read_fifo and
         * mpu_read_fifo consistent.
         */
        if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
        {
            ptr->GyroX = (float)gyro[0] / GYRO_SENS;
            ptr->GyroY = (float)gyro[1] / GYRO_SENS;
            ptr->GyroZ = (float)gyro[2] / GYRO_SENS;
        }
        if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
        {
            ptr->AccelX = (float)accel[0] / ACCEL_SENS;
            ptr->AccelY = (float)accel[1] / ACCEL_SENS;
            ptr->AccelZ = (float)accel[2] / ACCEL_SENS;
        }
        /* Unlike gyro and accel, quaternions are written to the FIFO in
         * the body frame, q30. The orientation is set by the scalar passed
         * to dmp_set_orientation during initialization.
         */
        if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT)
        {
            float euler[3];

            inv_get_sensor_type_euler(euler, quat);
            ptr->Pitch = euler[0];
            ptr->Roll = euler[1];
            ptr->Yaw = euler[2];
        }

        if (imuParam.pidCallback != NULL)
            imuParam.pidCallback();
    }

    return kStatus_Success;
}

/**
  * @brief  惯性测量单元指令输入
  * @param  [in]   cmd： 要执行的指令
  * @retval none
  */
void IMU_HandleInput(char cmd)
{
    switch (cmd)
    {
        /* These commands turn the hardware sensors on/off. */
        case '8':
            if (!hal.dmp_on)
            {
                /* Accel and gyro need to be on for the DMP features to work
                 * properly.
                 */
                hal.sensors ^= ACCEL_ON;
                setup_gyro();
            }
            break;
        case '9':
            if (!hal.dmp_on)
            {
                hal.sensors ^= GYRO_ON;
                setup_gyro();
            }
            break;
        /* The commands start/stop sending data to the client. */
        case 'a':
            hal.report ^= PRINT_ACCEL;
            break;
        case 'g':
            hal.report ^= PRINT_GYRO;
            break;
        case 'q':
            hal.report ^= PRINT_QUAT;
            break;
        /* The hardware self test can be run without any interaction with the
         * MPL since it's completely localized in the gyro driver. Logging is
         * assumed to be enabled; otherwise, a couple LEDs could probably be used
         * here to display the test results.
         */
        case 't':
            run_self_test();
            break;
        /* Depending on your application, sensor data may be needed at a faster or
         * slower rate. These commands can speed up or slow down the rate at which
         * the sensor data is pushed to the MPL.
         *
         * In this example, the compass rate is never changed.
         */
        case '1':
            if (hal.dmp_on)
                dmp_set_fifo_rate(10);
            else
                mpu_set_sample_rate(10);
            break;
        case '2':
            if (hal.dmp_on)
                dmp_set_fifo_rate(20);
            else
                mpu_set_sample_rate(20);
            break;
        case '3':
            if (hal.dmp_on)
                dmp_set_fifo_rate(40);
            else
                mpu_set_sample_rate(40);
            break;
        case '4':
            if (hal.dmp_on)
                dmp_set_fifo_rate(50);
            else
                mpu_set_sample_rate(50);
            break;
        case '5':
            if (hal.dmp_on)
                dmp_set_fifo_rate(100);
            else
                mpu_set_sample_rate(100);
            break;
        case '6':
            if (hal.dmp_on)
                dmp_set_fifo_rate(200);
            else
                mpu_set_sample_rate(200);
            break;
//        case ',':
//            /* Set hardware to interrupt on gesture event only. This feature is
//             * useful for keeping the MCU asleep until the DMP detects as a tap or
//             * orientation event.
//             */
//            dmp_set_interrupt_mode(DMP_INT_GESTURE);
//            break;
//        case '.':
//            /* Set hardware to interrupt periodically. */
//            dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
//            break;
//        case '7':
//            /* Reset pedometer. */
//            dmp_set_pedometer_step_count(0);
//            dmp_set_pedometer_walk_time(0);
//            break;
//        case 'f':
//            /* Toggle DMP. */
//            if (hal.dmp_on)
//            {
//                unsigned short dmp_rate;
//                hal.dmp_on = 0;
//                mpu_set_dmp_state(0);
//                /* Restore FIFO settings. */
//                mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
//                /* When the DMP is used, the hardware sampling rate is fixed at
//                 * 200Hz, and the DMP is configured to downsample the FIFO output
//                 * using the function dmp_set_fifo_rate. However, when the DMP is
//                 * turned off, the sampling rate remains at 200Hz. This could be
//                 * handled in inv_mpu.c, but it would need to know that
//                 * inv_mpu_dmp_motion_driver.c exists. To avoid this, we'll just
//                 * put the extra logic in the application layer.
//                 */
//                dmp_get_fifo_rate(&dmp_rate);
//                mpu_set_sample_rate(dmp_rate);
//            }
//            else
//            {
//                unsigned short sample_rate;
//                hal.dmp_on = 1;
//                /* Both gyro and accel must be on. */
//                hal.sensors |= ACCEL_ON | GYRO_ON;
//                mpu_set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO);
//                mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
//                /* Preserve current FIFO rate. */
//                mpu_get_sample_rate(&sample_rate);
//                dmp_set_fifo_rate(sample_rate);
//                mpu_set_dmp_state(1);
//            }
//            break;
//        case 'm':
//#ifndef MPU6050 // not enabled for 6050 product
//            /* Test the motion interrupt hardware feature. */
//            hal.motion_int_mode = 1;
//#endif
//            break;
//        case 'p':
//            /* Read current pedometer count. */
//            dmp_get_pedometer_step_count(pedo_packet);
//            dmp_get_pedometer_walk_time(pedo_packet + 1);
//            send_packet(PACKET_TYPE_PEDO, pedo_packet);
//            break;
//        case 'x':
//            kinetis_reset();
//            break;
//        case 'v':
//            /* Toggle LP quaternion.
//             * The DMP features can be enabled/disabled at runtime. Use this same
//             * approach for other features.
//             */
//            hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
//            dmp_enable_feature(hal.dmp_features);
//            break;
        default:
            break;
    }
}

/**
  * 当有新的数据产生时，
  * 这个函数应当在中断的上下文中被调用。
  */
void IMU_IRQ_Handle(void)
{
    BaseType_t reschedule;

    if (GPIO_GetPinsInterruptFlags(IMU_INT_GPIO) & (1U << IMU_INT_GPIO_PIN))
    {
        /* Clear external interrupt flag. */
        GPIO_ClearPinsInterruptFlags(IMU_INT_GPIO, 1U << IMU_INT_GPIO_PIN);

        xSemaphoreGiveFromISR(imuParam.ready, &reschedule);     //数据已经准备好，置位
        portYIELD_FROM_ISR(reschedule);
    }
}

/**
  * 如使用到PID控制器，
  * 可以通过此函数注册PID控制函数。
  * 注册成功后每次新数据到来时触发PID函数
  */
void IMU_PID_SetCallback(void *func)
{
    imuParam.pidCallback = func;
}

inline void IMU_ToggleLED(void)
{
    GPIO_TogglePinsOutput(IMU_LED_GPIO, 1U << IMU_LED_GPIO_PIN);
}
