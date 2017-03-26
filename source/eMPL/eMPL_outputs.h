/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *   @defgroup  HAL_Outputs
 *   @brief     Motion Library - HAL Outputs
 *              Sets up common outputs for HAL
 *
 *   @{
 *       @file  eMPL_outputs.h
 *       @brief Embedded MPL outputs.
 */
#ifndef _EMPL_OUTPUTS_H_
#define _EMPL_OUTPUTS_H_

#ifdef __cplusplus
extern "C" {
#endif

void inv_get_sensor_type_euler(float *euler, long *quat);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef _EMPL_OUTPUTS_H_ */

/**
 *  @}
 */
