/******************************************************************************

 @file  AccelSensorTask.h

 @brief This file contains the Simple BLE Peripheral sample application
        definitions and prototypes.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2013-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_00_31
 Release Date: 2016-06-16 18:57:29
 *****************************************************************************/

#ifndef ACCELSENSORTASK_H
#define ACCELSENSORTASK_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */
#ifndef USE_ACCEL_DEBUG_TASK
#define  Display_print0(handle, line, col, fmt)
#define  Display_print1(handle, line, col, fmt, a0)
#define  Display_print2(handle, line, col, fmt, a0, a1)
#define  Display_print3(handle, line, col, fmt, a0, a1, a2)
#define  Display_print4(handle, line, col, fmt, a0, a1, a2, a3)
#define  Display_print5(handle, line, col, fmt, a0, a1, a2, a3, a4)
#endif

#define FW_VER_MAJ				2
#define	FW_VER_MIN				90

//#define ENABLE_STANDBY
//#define BLE_ADV_EVENT_PROC

/*********************************************************************
 * MACROS
 */

#define FFT_SAMPLE_MAX		128

//#define ACCEL_SENSOR_POINT_COUNT	72
#define ACCEL_SENSOR_POINT_COUNT	36

#define ACCEL_SENSOR_MODE_WAITING	0
#define ACCEL_SENSOR_MODE_WORKING	1

// Callback events
#define ACCEL_EXT_PWR_UPDATE		0x01
#define ACCEL_SENSOR_DATA_INIT		0x02
#define ACCEL_COMPENSATE_ZERO_G		0x04

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the Simple BLE Peripheral.
 */
extern void AccelSensor_createTask(void);
#ifdef ACCELSENSOR_AUTOSCALE
extern void AccelSensor_changeScaleEvent(int scale);
extern void AccelSensor_restartCycle(void);
#endif
extern void AccelSensor_error(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ACCELSENSORTASK_H */
