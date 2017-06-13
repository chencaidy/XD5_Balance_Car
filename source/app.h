/*
 * app.h
 *
 *  Created on: 2017年6月14日
 *      Author: chen
 */

#ifndef APP_H_
#define APP_H_

/* Freescale includes. */
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_sysmpu.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Include internal header to get SEGGER_RTT_CB */
#include "SEGGER_RTT.h"

/* Task priorities. */
#define REALTIME_TASK     (configMAX_PRIORITIES - 1)
#define HIGH_TASK         (configMAX_PRIORITIES - 2)
#define MEDIUM_TASK       (configMAX_PRIORITIES - 3)
#define LOW_TASK          (configMAX_PRIORITIES - 4)

#endif /* APP_H_ */
