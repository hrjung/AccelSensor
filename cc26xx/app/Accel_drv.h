/*******************************************************************************
  Filename:       Accel_drv.h
  Revised:        $Date: 2014-02-28 14:18:14 -0800 (Fri, 28 Feb 2014) $
  Revision:       $Revision: 37461 $

  Description:    This file contains the SRF06EB Key Service definitions
                  and prototypes.

  Copyright 2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �뛾�룄�닞 IS占쎈쐻�뜝占� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
*******************************************************************************/

#ifndef ACCEL_DRV_H
#define ACCEL_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/
extern uint8_t motor_sense_threshold;
/*********************************************************************
 * CONSTANTS
 */
#define MIN_SAMPLE_COUNT		10
#define	MAX_SAMPLE_COUNT		130

// config index
#define ACCEL_CFG_DATA_RATE				0
#define ACCEL_CFG_SCALE					1
#define ACCEL_CFG_MAX					2


#define ACCEL_INT_AXIS_HIGH_CFG		0x2A
#define ACCEL_INT_THRESHOLD			40

#define ACCEL_DETECT_MOTOR		0
#define ACCEL_NORMAL_SENSE		1

/*********************************************************************
 * TYPEDEFS
 */
/*!
 *  @brief      A function pointer to a specific implementation of
 *              identify().
 */
typedef int (*Accel_identifyFnx)(void);
/*!
 *  @brief      A function pointer to a specific implementation of
 *              open().
 */
typedef int (*Accel_openFxn)(void);
/*!
 *  @brief      A function pointer to a specific implementation of
 *              close().
 */
typedef void (*Accel_closeFxn)(void);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              read().
 */
typedef int (*Accel_readValFxn)(uint8_t *values);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              write().
 */
typedef uint8_t (*Accel_writeCfgFxn)(int index, uint8_t value);

typedef uint8_t (*Accel_readCfgFxn)(int index);

typedef uint8_t (*Accel_readRegValFxn)(int index);

typedef uint8_t (*Accel_readStatusRegFxn)(void);

typedef struct Accel_FxnTable_tag
{
	Accel_identifyFnx	identifyFnx;
	Accel_openFxn       openFxn;
	Accel_closeFxn      closeFxn;
	Accel_readValFxn    readValFxn;
	Accel_writeCfgFxn   writeCfgFxn;
	Accel_readCfgFxn	readCfgFxn;
	Accel_readRegValFxn readRegValFxn;
	Accel_readStatusRegFxn readStatusRegFxn;
} Accel_FxnTable_st;

typedef struct {
	uint8_t 	addr;
	uint8_t 	value;
} Accel_Config_st;

/*********************************************************************
 * MACROS
 */
#define Acc_isNewDataAvailable(status) (status&0x08)

/*********************************************************************
 * API FUNCTIONS
 */

extern int Acc_init(void);
extern void Acc_stop(void);
extern int Acc_identify(void);
extern int Acc_readAccVal(int8_t *pXVal, int8_t *pYVal, int8_t *pZVal);
extern uint8_t Acc_readCfg(int index);
extern uint8_t Acc_writeCfg(int index, uint8_t value);
extern uint8_t Acc_readRegVal(int index);
extern uint8_t Acc_readStatus(void);
extern uint8_t Acc_setPowerMode(int mode);
#ifdef DETECT_MOTOR_ON
extern uint8_t Acc_setSensorMode(int normal);
#endif
extern uint8_t Acc_setZeroGMode(void);
/*********************************************************************
*********************************************************************/  

#ifdef __cplusplus
}
#endif

#endif /* ACCEL_DRV_H */
