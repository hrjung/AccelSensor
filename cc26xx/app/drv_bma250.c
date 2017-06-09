/*******************************************************************************
  Filename:       drv_bma250.c
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
  PROVIDED 獄쏄툞 IS�뜝占� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#if 0 //#ifdef ACCEL_BMA250_USED

#include "bma250.h"
#include "drv_bma250.h"
#include <ti/drivers/SPI.h>
#include "Board.h"
#include "Accel_drv.h"

#include <xdc/runtime/System.h> //for serial log



/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*******************************************************************************
 * EXTERNAL VARIABLES
 */
extern SPI_Handle AccelSpiHandle;

extern PIN_Config AccelSpiPinTable[];

// LCD pin state
extern PIN_State AccelSpiPinState;

// LCD pin handle
extern PIN_Handle hAccelSpiPins;
/*********************************************************************
 * LOCAL VARIABLES
 */
static Accel_Config_st cfg[ACCEL_CFG_MAX] = {
		ACC_BW, 	0x87, //ORD = 5.376kHz, enable all axis
		ACC_RANGE, 	0x10, //full scale selection 4g
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Bma250_open
 *
 * @brief   Open BMA250 peripheral on SRF06EB.
 *
 * @param   none
 *
 * @return  void
 */
void Bma250_open(int *mapTable)
{
  int i;

  //Enable the PWR, CSN for BMA250
  hAccelSpiPins = PIN_open(&AccelSpiPinState, AccelSpiPinTable);
    
  SPI_Params spiParams;
  spiParams.bitRate = 4000000;
  SPI_init();

  SPI_Params_init(&spiParams);
  
  // Open SPI peripheral
  AccelSpiHandle = SPI_open(Board_SPI0, &spiParams);
  if(AccelSpiHandle == NULL)
	  System_printf("SPI open error!\r\n");
  
  for(i=0; i<ACCEL_CFG_MAX; i++) mapTable[i] = Bma250_ConfigTable[i];
}


void bspSpiSelect(void)
{
    PIN_setOutputValue(hAccelSpiPins, Board_ACC_CSN, 0);
}

void bspSpiDeselect(void)
{
    PIN_setOutputValue(hAccelSpiPins, Board_ACC_CSN, 1);
}


/* See bsp_spi.h file for description */
void Bma250_close(void)
{
  if (AccelSpiHandle != NULL)
  {
    SPI_close(AccelSpiHandle);
    AccelSpiHandle = NULL;
  }
}

//
//  Write to an SPI device
//
int bspSpiWrite(const uint8_t *buf, size_t length)
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
int bspSpiRead(uint8_t *buf, size_t length)
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

int bspSpiWriteRead(uint8_t *wbuf, uint8_t wlen, uint8_t *rbuf, uint8_t rlen)
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

void Bma250_readVal(uint8_t *vals)
{
  uint8_t wbuf[1], length=6, rbuf[6];

  wbuf[0] = ACC_X_LSB | 0x80; // read command

  bspSpiSelect();
  if(bspSpiWrite(wbuf, sizeof(wbuf)))
  {
    /* failure */
	bspSpiDeselect();
    return;
  }

  bspSpiRead(rbuf, length);

  bspSpiDeselect();

  vals[0] = rbuf[1]; // X
  vals[1] = rbuf[3]; // Y
  vals[2] = rbuf[5]; // Z

}

void Bma250_readRegister(uint8_t addr, uint8_t  length, uint8_t *rbuf)
{
  uint8_t wbuf[1];

  wbuf[0] = addr | 0x80; // read command

  bspSpiSelect();
  if(bspSpiWrite(wbuf, sizeof(wbuf)))
  {
    /* failure */
	bspSpiDeselect();
    return;
  }

  bspSpiRead(rbuf, length);

  bspSpiDeselect();

}

uint8_t Bma250_readCfg(int index)
{


	//Bma250_readRegister()
	return 0;
}

#endif

/*********************************************************************
*********************************************************************/
