/*******************************************************************************
  Filename:       snv_drv.c
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
  PROVIDED �뜝�럥�럸�뜝�럥利꿨뜝�럥�뻽 IS占쎈쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "Accel_drv.h"
#include <ti/drivers/SPI.h>
//#include "Board.h"

//#include "board_gpio.h"
#include "snv_drv.h"


/*********************************************************************
 * TYPEDEFS
 */

static SNV_data_st snv;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static uint8_t SNV_write(void);
static uint8_t SNV_read(void);

/*******************************************************************************
 * EXTERNAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */

static int SNV_initialized=0;


/*********************************************************************
 * PUBLIC FUNCTIONS
 */
extern void AccelSensor_initZeroG(int *zero_g);
/****************************************************************************
* @fn       SNV_init
*
* @brief    return initialized flag in SNV
*
* @return   void
****************************************************************************/
uint8_t SNV_init(void)
{
	uint8_t result=SNV_SUCCESS;

	//read SNV
	result = SNV_read();
	if(result != SNV_SUCCESS || snv.flag != SNV_INITIALIZED)
	{
		//default setting and write back to SNV
		snv.flag = SNV_INITIALIZED;

		snv.odr = Acc_readCfg(ACCEL_CFG_DATA_RATE);
		snv.scale = Acc_readCfg(ACCEL_CFG_SCALE);
		snv.zero_g[0] = 0;
		snv.zero_g[1] = 0;
		snv.zero_g[2] = 0;

		//write back to SNV
		result = SNV_write();
	}

	SNV_initialized = 1;
	return result;
}

void SNV_applyInitalValue(void)
{
	Acc_writeCfg(ACCEL_CFG_DATA_RATE, (uint8_t)snv.odr);
	Acc_writeCfg(ACCEL_CFG_SCALE, (uint8_t)snv.scale);

	AccelSensor_initZeroG(snv.zero_g);
}

int SNV_isInitialized(void)
{
	return SNV_initialized;
}

int SNV_getItemValue(int snv_id)
{
	int value;

	switch(snv_id)
	{
	case SNV_ACCEL_DATARATE_ITEM :
		value = snv.odr;
		break;

	case SNV_ACCEL_SCALE_ITEM :
		value = snv.scale;
		break;
	}

	return value;
}

uint8_t SNV_setItemValue(int snv_id, uint8_t value)
{
	uint8_t status=1;

	switch(snv_id)
	{
	case SNV_ACCEL_DATARATE_ITEM :
		snv.odr = (int)value;
		status = Acc_writeCfg(ACCEL_CFG_DATA_RATE, (uint8_t)snv.odr);
		break;

	case SNV_ACCEL_SCALE_ITEM :
		snv.scale = (int)value;
		status = Acc_writeCfg(ACCEL_CFG_SCALE, (uint8_t)snv.scale);
		break;
	}

	if(status == 0) return 0;

	return SNV_write();
}

uint8_t SNV_getMultiItem(int snv_id, int *value)
{
	SNV_read();
	switch(snv_id)
	{
	case SNV_ACCEL_ZERO_G_ITEM :
		value[0] = snv.zero_g[0];
		value[1] = snv.zero_g[1];
		value[2] = snv.zero_g[2];
		break;
	}

	return 1;
}

uint8_t SNV_setMultiItem(int snv_id, int *value)
{
	uint8_t status=1;

	switch(snv_id)
	{
	case SNV_ACCEL_ZERO_G_ITEM :
		snv.zero_g[0] = (int)value[0];
		snv.zero_g[1] = (int)value[1];
		snv.zero_g[2] = (int)value[2];
		AccelSensor_initZeroG(snv.zero_g);
		break;

	default :
		status = 0;
		break;
	}

	if(status == 0) return 0;

	return SNV_write();
}

//**********************************************
//	only for Debug command
int SNV_forceInit(void)
{
	snv.flag = 0;
	return SNV_write();
}

uint8 SNV_readValue(SNV_data_st *val)
{
	uint8 status, len = sizeof(SNV_data_st);
	SNV_data_st snv_val;

	status = osal_snv_read((uint8)ACCEL_SNV_START, len, (void *)&snv_val);

	if(status == SUCCESS) memcpy(val, &snv_val, len);

	return status;
}

uint8 SNV_writeValue(SNV_data_st *val)
{
	uint8 len = sizeof(SNV_data_st);
	SNV_data_st snv_val;

	memcpy(&snv_val, val, len);
	return osal_snv_write((uint8)ACCEL_SNV_START, len, (void *)&snv_val);
}
//***********************************************


static uint8_t SNV_write(void)
{
	uint8_t status, result=SNV_SUCCESS;
	uint8 len = sizeof(SNV_data_st);
	SNV_data_st snv_val;

	memcpy(&snv_val, &snv, len);
	status = osal_snv_write(ACCEL_SNV_START, len, &snv_val);
	if(status != SUCCESS) result = SNV_WRITE_ERROR;

	return result;
}

static uint8_t SNV_read(void)
{
	uint8_t status, result=SNV_SUCCESS;
	uint8 len = sizeof(SNV_data_st);
	SNV_data_st snv_val;

	status = osal_snv_read(ACCEL_SNV_START, len, &snv_val);
	if(status != SUCCESS) result = SNV_READ_ERROR;
	else	memcpy(&snv, &snv_val, len);

	return result;
}

/*********************************************************************
*********************************************************************/
