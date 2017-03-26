/*
 * sccb.c
 *
 *  Created on: 2017年3月4日
 *      Author: chen
 */


/* Includes ------------------------------------------------------------------*/
#include "sccb.h"

#include "fsl_debug_console.h"
#include "fsl_gpio.h"

/* Definitions ---------------------------------------------------------------*/
/* SCCB configuration. */
#define SIO_C_GPIO            (GPIOC)
#define SIO_C_GPIO_PIN        (10U)

#define SIO_D_GPIO            (GPIOC)
#define SIO_D_GPIO_PIN        (11U)

#define SDA_OUT()         (SIO_D_GPIO->PDDR |= (1U << SIO_D_GPIO_PIN))
#define SDA_IN()          (SIO_D_GPIO->PDDR &= ~(1U << SIO_D_GPIO_PIN))
#define SDA_H()           (SIO_D_GPIO->PSOR = (1U << SIO_D_GPIO_PIN))
#define SDA_L()           (SIO_D_GPIO->PCOR = (1U << SIO_D_GPIO_PIN))
#define SDA_R()           (((SIO_D_GPIO->PDIR) >> SIO_D_GPIO_PIN) & 1U)
#define SCL_H()           (SIO_C_GPIO->PSOR = (1U << SIO_C_GPIO_PIN))
#define SCL_L()           (SIO_C_GPIO->PCOR = (1U << SIO_C_GPIO_PIN))

/* User Codes ----------------------------------------------------------------*/
static void SCCB_Delay(void)
{
    volatile uint8_t i = 200;

    while (i--)
    {
    }
}

static void SCCB_Start(void)
{
    SDA_OUT();
    SDA_H();
    SCL_H();
    SCCB_Delay();
    SDA_L();
    SCCB_Delay();
    SCL_L();
}

static void SCCB_Stop(void)
{
    SDA_L();
    SCCB_Delay();
    SCL_H();
    SCCB_Delay();
    SDA_H();
}

static void SCCB_isAck(bool ack)
{
    if (ack == true)
        SDA_L();
    else
        SDA_H();

    SCCB_Delay();
    SCL_H();
    SCCB_Delay();
    SCL_L();
}

static bool SCCB_WaitAck(void)
{
    bool ack;

    SDA_IN();
    SCCB_Delay();
    SCL_H();
    SCCB_Delay();
    ack = SDA_R();
    SCL_L();
    SDA_OUT();

    return ack;
}

static void SCCB_SendByte(uint8_t data)
{
    volatile uint8_t i;

    i = 8;
    while (i--)
    {
        if (data & 0x80)
            SDA_H();
        else
            SDA_L();
        data <<= 1;
        SCCB_Delay();
        SCL_H();
        SCCB_Delay();
        SCL_L();
    }
}

static uint8_t SCCB_GetByte(void)
{
    uint8_t i, byte = 0;

    SDA_IN();
    for (i = 0; i < 8; i++)
    {
        SCCB_Delay();
        SCL_H();
        SCCB_Delay();
        byte = (byte << 1) | (SDA_R() & 1);
        SCL_L();
    }
    SDA_OUT();

    return byte;
}

int SCCB_WriteReg(uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
{
    uint8_t err, retry = 5;

    while (retry--)
    {
        err = 0;

        SCCB_Start();
        SCCB_SendByte(slave_addr);
        err += SCCB_WaitAck();
        SCCB_SendByte(reg_addr);
        err += SCCB_WaitAck();
        SCCB_SendByte(data);
        err += SCCB_WaitAck();
        SCCB_Stop();

        if (err)
            PRINTF("SCCB: Error during write transaction, retry.\r\n");
        else
            break;
    }

    return err;
}

int SCCB_ReadReg(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data_ptr)
{
    uint8_t err, retry = 5;

    while (retry--)
    {
        err = 0;

        SCCB_Start();
        SCCB_SendByte(slave_addr);
        err += SCCB_WaitAck();
        SCCB_SendByte(reg_addr);
        err += SCCB_WaitAck();
        SCCB_Stop();

        SCCB_Start();
        SCCB_SendByte(slave_addr + 1);
        err += SCCB_WaitAck();
        *data_ptr = SCCB_GetByte();
        SCCB_isAck(false);
        SCCB_Stop();

        if (err)
            PRINTF("SCCB: Error during read transaction, retry.\r\n");
        else
            break;
    }

    return err;
}

void SCCB_Config(void)
{
    gpio_pin_config_t sccb_config =
    { kGPIO_DigitalOutput, 1, };

    GPIO_PinInit(SIO_C_GPIO, SIO_C_GPIO_PIN, &sccb_config);
    GPIO_PinInit(SIO_D_GPIO, SIO_D_GPIO_PIN, &sccb_config);
}
