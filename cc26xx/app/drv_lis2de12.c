/*******************************************************************************
  Filename:       drv_lis2de12.c
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

#ifdef ACCEL_LIS2DE12_USED

#include "osal_snv.h"
#include "Accel_drv.h"
#include "drv_lis2de12.h"
#include <ti/drivers/SPI.h>
#include "Board.h"
//#include "board_led.h" //hrjung add
#include "board_gpio.h"
#include "snv_drv.h"

#include <xdc/runtime/System.h> //for serial log


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static int Lis2de12_initConfigReg(void);
static uint8_t Lis2de12_setConfigData(int cfg_index);
static uint8_t Lis2de12_setConfigReg(uint8_t addr, uint8_t value);
#ifdef DETECT_MOTOR_ON
static uint8_t Lis2de12_setIntConfig(int enable);
#endif
/*******************************************************************************
 * EXTERNAL VARIABLES
 */
extern SPI_Handle AccelSpiHandle;

extern PIN_Config AccelSpiPinTable[];

// LCD pin state
extern PIN_State AccelSpiPinState;

// LCD pin handle
extern PIN_Handle hAccelSpiPins;

#define bspSpiSelect()	PIN_setOutputValue(hAccelSpiPins, Board_ACC_CSN, 0)
#define bspSpiDeselect() PIN_setOutputValue(hAccelSpiPins, Board_ACC_CSN, 1)
/*********************************************************************
 * LOCAL VARIABLES
 */

static Accel_Config_st cfg[ACCEL_CFG_MAX] = {
		LIS2DE12_CFG1_ADDR, 0x9F, //ORD = 5.3kHz, enable all axis, enable low-power mode(not allowed by data sheet)
		LIS2DE12_CFG4_ADDR, 0x10, //full scale selection ACCEL_SCALE_DEFAULT: 4g
		//LIS2DE12_CFG4_ADDR, 0x90, //full scale selection ACCEL_SCALE_DEFAULT: 4g, set BDU=1 for temperature
};

//static uint8_t fifo_cfg=0x5F; // FM=1(FIFO mode), TR=0, FTH=31

extern void test_ACC_output(uint8_t value);
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

//
//  Write to an SPI device
//
static int bspSpiWrite(const uint8_t *buf, size_t length)
{
  SPI_Transaction masterTransaction;
  bool success;

  masterTransaction.count  = length;
  masterTransaction.txBuf  = (void*)buf;
  masterTransaction.arg    = NULL;
  masterTransaction.rxBuf  = NULL;

  success = SPI_transfer(AccelSpiHandle, &masterTransaction);

  return success ? 0 : -1;
}

//
//  Read from an SPI device
//
static int bspSpiRead(uint8_t *buf, size_t length)
{
  SPI_Transaction masterTransaction;
  bool success;

  masterTransaction.count  = length;
  masterTransaction.txBuf  = NULL;
  masterTransaction.arg    = NULL;
  masterTransaction.rxBuf  = buf;

  success = SPI_transfer(AccelSpiHandle, &masterTransaction);

  return success ? 0 : -1;
}

static int bspSpiWriteRead(uint8_t *wbuf, uint8_t wlen, uint8_t *rbuf, uint8_t rlen)
{
  SPI_Transaction masterTransaction;
  bool success;

  masterTransaction.count  = wlen + rlen;
  masterTransaction.txBuf  = wbuf;
  masterTransaction.arg    = NULL;
  masterTransaction.rxBuf  = wbuf;

  success = SPI_transfer(AccelSpiHandle, &masterTransaction);
  if (success)
  {
    memcpy(rbuf,wbuf+wlen,rlen);
  }

  return success ? 0 : -1;
}

/*********************************************************************
 * @fn      Lis2de12_open
 *
 * @brief   Open LIS2DE12 peripheral on SRF06EB.
 *
 * @param   none
 *
 * @return  void
 */
int Lis2de12_open(void)
{
	int result=1;

	//Enable the PWR, CSN for LIS2DE12
	hAccelSpiPins = PIN_open(&AccelSpiPinState, AccelSpiPinTable);

	SPI_Params spiParams;

	SPI_init();
	SPI_Params_init(&spiParams);
	spiParams.bitRate = 8000000;

	// Open SPI peripheral
	AccelSpiHandle = SPI_open(Board_SPI0, &spiParams);
	if(AccelSpiHandle == NULL)
		result=0;

	//for(i=0; i<ACCEL_CFG_MAX; i++) mapTable[i] = Lis2de12_ConfigTable[i];

	return result;
}

/* See bsp_spi.h file for description */
void Lis2de12_close(void)
{
  if (AccelSpiHandle != NULL)
  {
    SPI_close(AccelSpiHandle);
    AccelSpiHandle = NULL;
  }
}

int Lis2de12_readVal(uint8_t *vals)
{
  uint8_t wbuf[10], length=5, rbuf[6];

  wbuf[0] = LIS2DE12_OUTX_ADDR | 0xC0; // multiple read command

  bspSpiSelect();
#if 1
  if(bspSpiWriteRead(wbuf, 1, rbuf, length))
  {
    /* failure */
	bspSpiDeselect();
    return 0;
  }
#else
  if(bspSpiWrite(wbuf, 1))
  {
    /* failure */
	bspSpiDeselect();
    return 0;
  }

  //test_ACC_output(1);
  bspSpiRead(rbuf, length);
  //test_ACC_output(0);
#endif
  bspSpiDeselect();

  vals[0] = rbuf[0]; // X
  vals[1] = rbuf[2]; // Y
  vals[2] = rbuf[4]; // Z

  return 1;
}

int Lis2de12_readRegister(uint8_t addr, uint8_t  length, uint8_t *rbuf)
{
  uint8_t wbuf[10];

  wbuf[0] = addr | 0x80; // read command

  bspSpiSelect();
#if 1
  if(bspSpiWriteRead(wbuf, 1, rbuf, length))
  {
    /* failure */
	bspSpiDeselect();
    return 0;
  }
#else
  if(bspSpiWrite(wbuf, 1))
  {
    /* failure */
	bspSpiDeselect();
    return 0;
  }

  bspSpiRead(rbuf, length);
#endif

  bspSpiDeselect();

  return 1;

}

static int Lis2de12_writeRegister(uint8_t addr, uint8_t  length, uint8_t *wbuf)
{
  wbuf[0] = addr & 0x7F; // write command

  bspSpiSelect();
  if(bspSpiWrite(wbuf, length))
  {
    /* failure */
	bspSpiDeselect();
    return 0;
  }

  bspSpiDeselect();

  return 1;
}

int Lis2de12_identify(void)
{
	uint8_t rbuf[2] = {0,0};

	if(Lis2de12_readRegister(LIS2DE12_ID_ADDR, 1, rbuf))
	{
		if(rbuf[0] == LIS2DE12_ID)
		{
			if(Lis2de12_initConfigReg())
			{
				return 1;
			}
		}
	}

	return 0;
}

//read config register value
static int Lis2de12_initConfigReg(void)
{
	int result=1;
	uint8_t status1, status2, status3;

	status1 = Lis2de12_setConfigData(ACCEL_CFG_SCALE);
	status2 = Lis2de12_setConfigData(ACCEL_CFG_DATA_RATE);
#ifdef DETECT_MOTOR_ON
	status3 = Lis2de12_setIntSensor();
#else
	status3 = Lis2de12_setConfigReg(LIS2DE12_CFG3_ADDR, 0x10); // enable data ready interrupt
#endif
	if(status1 == 0 || status2 == 0 || status3 == 0) result = 0;

	return result;
}

static uint8_t Lis2de12_getDataRate(int index)
{
	return (cfg[index].value&0xF0)>>4;
}

static uint8_t Lis2de12_getFullScale(int index)
{
	return (cfg[index].value&0x30)>>4;
}

//read config register
uint8_t Lis2de12_readConfig(int cfg_index)
{
	uint8_t result=0;
	switch(cfg_index)
	{
	case ACCEL_CFG_DATA_RATE :
		result = Lis2de12_getDataRate(cfg_index);
		break;
	case ACCEL_CFG_SCALE :
		result = Lis2de12_getFullScale(cfg_index);
		break;
	}

	return result;
}

static uint8_t Lis2de12_setConfigData(int cfg_index)
{
	uint8_t wbuf[2] = {0,0}, result=0;
	int status;

	wbuf[0] = cfg[cfg_index].addr;
	wbuf[1] = cfg[cfg_index].value;
	status = Lis2de12_writeRegister(wbuf[0], 2, wbuf);
	if(status) result = 1;

	return result;
}

static uint8_t Lis2de12_setConfigReg(uint8_t addr, uint8_t value)
{
	uint8_t wbuf[2] = {0,0}, result=0;
	int status;

	wbuf[0] = addr;
	wbuf[1] = value;
	status = Lis2de12_writeRegister(wbuf[0], 2, wbuf);
	if(status) result = 1;

	return result;
}

static uint8_t Lis2de12_setDataRate(int cfg_index, uint8_t value)
{
	cfg[cfg_index].value &= 0x0F;
	cfg[cfg_index].value |= ((value&0x0F)<<4);
	if(value < 9)
		cfg[cfg_index].value &= 0xF7; //disable low power mode
	else	//ODR=5.3kHz
		cfg[cfg_index].value |= 0x08; //enable low power mode

	return 1;
}

static uint8_t Lis2de12_setFullScale(int cfg_index, uint8_t value)
{
	cfg[cfg_index].value &= 0xCF;
	cfg[cfg_index].value |= ((value&0x03)<<4);

	return 1;
}

uint8_t Lis2de12_writeConfig(int cfg_index, uint8_t value)
{
	uint8_t result=0;
	switch(cfg_index)
	{
	case ACCEL_CFG_DATA_RATE :
		result = Lis2de12_setDataRate(cfg_index, value);
		break;
	case ACCEL_CFG_SCALE :
		result = Lis2de12_setFullScale(cfg_index, value);
		break;
	default:
		return result;
	}
	result = Lis2de12_setConfigData(cfg_index);

	return result;
}

uint8_t Lis2de12_setPowerDown(void)
{
	uint8_t value = cfg[ACCEL_CFG_DATA_RATE].value;

	value &= 0x07; // set ODR=0
	return Lis2de12_setConfigReg(cfg[ACCEL_CFG_DATA_RATE].addr, value);
}

uint8_t Lis2de12_restorePowerMode(void)
{
	return Lis2de12_setConfigReg(cfg[ACCEL_CFG_DATA_RATE].addr, cfg[ACCEL_CFG_DATA_RATE].value);
}

uint8_t Lis2de12_readRegValue(int cfg_index)
{
	uint8_t value=0;
	int result;

	result = Lis2de12_readRegister(cfg[cfg_index].addr, 1, &value);
	if(result == 0) value=0;

	return value;
}

uint8_t Lis2de12_readStatusReg(void)
{
	uint8_t value=0;
	int result;

	result = Lis2de12_readRegister(LIS2DE12_STATUS_ADDR, 1, &value);
	if(result == 0) value=0;

	return value;
}

#ifdef DETECT_MOTOR_ON
static uint8_t Lis2de12_setIntConfig(int enable)
{
	uint8_t status=1, status1, status2, status3;

	if(enable)
	{
		//any X, Y, Z high event interrupt
		status1 = Lis2de12_setConfigReg(LIS2DE12_INT1_CFG_ADDR, ACCEL_INT_AXIS_HIGH_CFG);
		status2 = Lis2de12_setConfigReg(LIS2DE12_INT1_THS_ADDR, motor_sense_threshold); //high event threshold value, over 1g for 4g

		status3 = Lis2de12_setConfigReg(LIS2DE12_CFG3_ADDR, 0x40); //AOI1 interrupt on INT1
		status3 = Lis2de12_setConfigReg(LIS2DE12_INT1_DUR_ADDR, 30);
	}
	else
	{
		status1 = Lis2de12_setConfigReg(LIS2DE12_INT1_CFG_ADDR, 0);
		status2 = Lis2de12_setConfigReg(LIS2DE12_INT1_THS_ADDR, 0);

		status3 = Lis2de12_setConfigReg(LIS2DE12_CFG3_ADDR, 0x10); //DRDY1 on INT1
		status3 = Lis2de12_setConfigReg(LIS2DE12_INT1_DUR_ADDR, 0);
	}

	if(status1 == 0 || status2 == 0 || status3 == 0) status = 0;

	return status;
}

uint8_t Lis2de12_setNormalSensor(void)
{
	uint8_t status=1;

	//status1 = Lis2de12_setDataReadyInt(1); // enable data ready interrupt
	status = Lis2de12_setIntConfig(0); // disable INT1

	//if(status1 == 0 || status2 == 0) status = 0;

	return status;
}

uint8_t Lis2de12_setIntSensor(void)
{
	uint8_t status=1; //, status1, status2;

	//status1 = Lis2de12_setDataReadyInt(0); // disable data ready interrupt
	status = Lis2de12_setIntConfig(1); // enable INT1

	//if(status1 == 0 || status2 == 0) status = 0;

	return status;
}
#endif

uint8_t Lis2de12_compConfig(void)
{
	uint8_t status;

	status = Lis2de12_setConfigData(ACCEL_CFG_SCALE);
	status = Lis2de12_setConfigData(ACCEL_CFG_DATA_RATE);
	status = Lis2de12_setConfigReg(LIS2DE12_INT1_CFG_ADDR, 0);
	status = Lis2de12_setConfigReg(LIS2DE12_INT1_THS_ADDR, 0);
	status = Lis2de12_setConfigReg(LIS2DE12_INT1_DUR_ADDR, 0);
	status = Lis2de12_setConfigReg(LIS2DE12_CFG3_ADDR, 0);

	return 1;
}

#endif

/*********************************************************************
*********************************************************************/
