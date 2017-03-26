/*
 * sbus.c
 *
 *  Created on: 2017年3月14日
 *      Author: chen
 */


/* Includes ------------------------------------------------------------------*/
#include "sbus.h"

#include "fsl_debug_console.h"
#include "fsl_uart.h"
#include "fsl_uart_freertos.h"
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Definitions ---------------------------------------------------------------*/
#define SBUS_UART               (UART4)
#define SBUS_UART_CLKSRC        (UART4_CLK_SRC)
#define SBUS_UART_RX_TX_IRQn    (UART4_RX_TX_IRQn)

#define UART_BUFFER_SIZE        (50U)
#define SBUS_FRAME_SIZE         (25U)

#define SBUS_FAILSAFE_INACTIVE  (0U)
#define SBUS_FAILSAFE_ACTIVE    (1U)
#define SBUS_STARTBYTE          (0x0F)
#define SBUS_ENDBYTE            (0x00)

/* Variables -----------------------------------------------------------------*/
static struct _uart_handle t_handle;
static uart_rtos_handle_t handle;
static uart_rtos_config_t uart_config;

static uint8_t g_Buffer[UART_BUFFER_SIZE];
static uint8_t frame[SBUS_FRAME_SIZE];

/* Codes ---------------------------------------------------------------------*/
status_t Sbus_Config(void)
{
    status_t status = kStatus_Success;

    NVIC_SetPriority(SBUS_UART_RX_TX_IRQn, 5);

    /*
     * config.baudRate_Bps = 100000U;
     * config.parityMode = kUART_ParityEven;
     * config.stopBitCount = kUART_TwoStopBit;
     */
    uart_config.base = SBUS_UART;
    uart_config.srcclk = CLOCK_GetFreq(SBUS_UART_CLKSRC);
    uart_config.baudrate = 100000U;
    uart_config.parity = kUART_ParityEven,
    uart_config.stopbits = kUART_TwoStopBit,
    uart_config.buffer = g_Buffer,
    uart_config.buffer_size = sizeof(g_Buffer),

    status = UART_RTOS_Init(&handle, &t_handle, &uart_config);

    return status;
}

status_t Sbus_UpdateRC(sbusChannel_t *sbus)
{
    status_t error = kStatus_Success;
    size_t cnt;

    /* 检测是否有传入参数 */
    if (sbus == NULL)
        return kStatus_InvalidArgument;

    /* 开始帧同步，检测到同步信号后开始接收 */
    UART_RTOS_Receive(&handle, frame, 1, &cnt);
    if (cnt == 1 && frame[0] == SBUS_STARTBYTE)
        error = UART_RTOS_Receive(&handle, &frame[1], SBUS_FRAME_SIZE - 1, &cnt);

    if (error == kStatus_UART_RxHardwareOverrun)
    {
        /* Notify about hardware buffer overrun */
        PRINTF("S-Bus: Hardware over run, return once. \r\n");
        return kStatus_Fail;
    }

    if (error == kStatus_UART_RxRingBufferOverrun)
    {
        /* Notify about ring buffer overrun */
        PRINTF("S-Bus: Ring buffer over run, return once. \r\n");
        return kStatus_Fail;
    }

    if (cnt == SBUS_FRAME_SIZE - 1)
    {
        if (frame[0] == SBUS_STARTBYTE && frame[SBUS_FRAME_SIZE - 1] == SBUS_ENDBYTE)
        {
            sbus->ch[0] = ((frame[1] | frame[2] << 8) & 0x07FF);
            sbus->ch[1] = ((frame[2] >> 3 | frame[3] << 5) & 0x07FF);
            sbus->ch[2] = ((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF);
            sbus->ch[3] = ((frame[5] >> 1 | frame[6] << 7) & 0x07FF);
            sbus->ch[4] = ((frame[6] >> 4 | frame[7] << 4) & 0x07FF);
            sbus->ch[5] = ((frame[7] >> 7 | frame[8] << 1 | frame[9] << 9) & 0x07FF);
            sbus->ch[6] = ((frame[9] >> 2 | frame[10] << 6) & 0x07FF);
            sbus->ch[7] = ((frame[10] >> 5 | frame[11] << 3) & 0x07FF);
            sbus->ch[8] = ((frame[12] | frame[13] << 8) & 0x07FF);
            sbus->ch[9] = ((frame[13] >> 3 | frame[14] << 5) & 0x07FF);
            sbus->ch[10] = ((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF);
            sbus->ch[11] = ((frame[16] >> 1 | frame[17] << 7) & 0x07FF);
            sbus->ch[12] = ((frame[17] >> 4 | frame[18] << 4) & 0x07FF);
            sbus->ch[13] = ((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF);
            sbus->ch[14] = ((frame[20] >> 2 | frame[21] << 6) & 0x07FF);
            sbus->ch[15] = ((frame[21] >> 5 | frame[22] << 3) & 0x07FF);

            sbus->ch[16] = ((frame[23]) & 0x0001) ? 2047 : 0;
            sbus->ch[17] = ((frame[23] >> 1) & 0x0001) ? 2047 : 0;

            if ((frame[23] >> 3) & 0x0001)
                sbus->failSafe = SBUS_FAILSAFE_ACTIVE;
            else
                sbus->failSafe = SBUS_FAILSAFE_INACTIVE;

            if ((frame[23] >> 2) & 0x0001)
                sbus->lostFrames++;
        }
    }

    return error;
}
