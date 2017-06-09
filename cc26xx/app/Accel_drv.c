/*******************************************************************************
  Filename:       Accel_drv.c
  Revised:        $Date: 2014-03-10 07:29:12 -0700 (Mon, 10 Mar 2014) $
  Revision:       $Revision: 37597 $

  Description:    This file contains the interface to the SRF06EB LCD Service.

  Copyright 2014 - 2015 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 占쎈쎗占쎈즲占쎈떈 IS�뜝�럥�맶占쎈쐻�뜝占� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <string.h>
#include <comdef.h>

#include "osal_snv.h"
#include "drv_bma250.h"
#include "drv_lis2de12.h"
#include "Accel_drv.h"
#include <ti/drivers/SPI.h>
#include "Board.h"

#include "board_gpio.h"


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

#if defined(ACCEL_LIS2DE12_USED)
const Accel_FxnTable_st Accel_fxnTable = {
	Lis2de12_identify,
	Lis2de12_open,
	Lis2de12_close,
	Lis2de12_readVal, // read sensored X, Y, Z values
	Lis2de12_writeConfig,
	Lis2de12_readConfig,
	Lis2de12_readRegValue,
	Lis2de12_readStatusReg,
};
#elif defined(ACCEL_BMA250_USED)
const Accel_FxnTable_st Accel_fxnTable = {
	NULL,
	Bma250_open,
	Bma250_close,
	Bma250_readVal,
	NULL,
	Bma250_readCfg,
	NULL,
	NULL,
};
#else
error "ACCELEROMETER is not defined"
#endif

/*******************************************************************************
 * EXTERNAL VARIABLES
 */
uint8_t motor_sense_threshold = ACCEL_INT_THRESHOLD;

// SPI parameter
SPI_Handle AccelSpiHandle = NULL;

// LCD pin table
PIN_Config AccelSpiPinTable[] = {
	Board_ACC_CSN     | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH  | PIN_PUSHPULL,  // Enable Acc CSB. Need to be low.
	Board_ACC_PWR     | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH  | PIN_PUSHPULL,  // Enable Acc PWR. Need to be high to work.
    PIN_TERMINATE                                                           // Terminate list
};

// LCD pin state
PIN_State AccelSpiPinState;

// LCD pin handle
PIN_Handle hAccelSpiPins;

/*********************************************************************
 * LOCAL VARIABLES
 */
static int acc_initialized = FALSE;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

int Acc_isAccInitialized(void)
{
	return acc_initialized;
}

/****************************************************************************
* @fn       Acc_init(void)
*
* @brief    Initialize SPI interface and accelerometer.
*
* @param    None.
*
* @return   void
****************************************************************************/
int Acc_init(void)
{
	int result=0;

	if(Accel_fxnTable.openFxn())
	{
		acc_initialized = TRUE;
		result = 1;
	}

	return result;
}

/****************************************************************************
* @fn       Acc_stop(void)
*
* @brief    Sets the accelerometer in low-power mode.
*
* @param    None.
*
* @return   void
******************************************************************************/
void Acc_stop(void)
{
  if (Acc_isAccInitialized())
  {
    // We cheat and simply turn off power to the accelerometer
    acc_initialized = FALSE;
  }
}

/****************************************************************************
* @fn       Acc_identify(void)
*
* @brief    Sets the BMA250 accelerometer in low-power mode.
*
* @param    None.
*
* @return   void
******************************************************************************/
int Acc_identify(void)
{
	int result=0;
	if(Acc_isAccInitialized())
	{
		if(Accel_fxnTable.identifyFnx)
			result = Accel_fxnTable.identifyFnx();
	}

	return result;
}

/****************************************************************************
* @fn       Acc_readAcc
*
* @brief    Read x, y and z acceleration data in one operation.
*
* @param    pXVal   Pointer to destination of read out X acceleration
* @param    pYVal   Pointer to destination of read out Y acceleration
* @param    pZVal   Pointer to destination of read out Z acceleration
*
* @return   void
****************************************************************************/
int Acc_readAccVal(int8_t *pXVal, int8_t *pYVal, int8_t *pZVal)
{
  int8_t readout[3] = {0,0,0};
  int result=0;

  // Read all data from accelerometer
  result = Accel_fxnTable.readValFxn((uint8_t *)readout);
  //result = Lis2de12_readVal((uint8_t *)readout);
  if(result)
  {
	  // Use only most significant byte of each channel.
	  *pXVal = readout[0];
	  *pYVal = readout[1];
	  *pZVal = readout[2];
  }

  return result;
}

uint8_t Acc_readCfg(int index)
{
	uint8_t result=0;

	if(Accel_fxnTable.readCfgFxn)
		result = Accel_fxnTable.readCfgFxn(index);

	return result;
}

uint8_t Acc_writeCfg(int index, uint8_t value)
{
	uint8_t result=0;

	if(Accel_fxnTable.writeCfgFxn)
		result = Accel_fxnTable.writeCfgFxn(index, value);

	return result;
}

uint8_t Acc_readRegVal(int index)
{
	uint8_t result=0;

	if(Accel_fxnTable.readRegValFxn)
		result = Accel_fxnTable.readRegValFxn(index);

	return result;
}

uint8_t Acc_readStatus(void)
{
	uint8_t result=0;

	if(Accel_fxnTable.readStatusRegFxn)
		result = Accel_fxnTable.readStatusRegFxn();

	return result;
}

uint8_t Acc_setPowerMode(int mode)
{
	uint8_t result;

	if(mode)
		result = Lis2de12_restorePowerMode();
	else
		result = Lis2de12_setPowerDown();

	return result;
}

#ifdef DETECT_MOTOR_ON
uint8_t Acc_setSensorMode(int normal)
{
	uint8_t status;
	if(normal == ACCEL_NORMAL_SENSE)
		status = Lis2de12_setNormalSensor();
	else
		status = Lis2de12_setIntSensor();

	return status;
}
#endif


uint8_t Acc_setZeroGMode(void)
{
	uint8_t status;

	status = Lis2de12_compConfig();

	return status;
}

/*********************************************************************
*********************************************************************/
