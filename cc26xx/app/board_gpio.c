/******************************************************************************

 @file  board_gpio.c

 @brief This file contains the interface to the SRF06EB generic GPIO.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2014-2016, Texas Instruments Incorporated
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

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/pin/PINCC26XX.h>

#ifdef USE_ICALL
#include <icall.h>
#endif
   
#include <inc/hw_ints.h>

#include "util.h"
#include "board.h"

#if 1 //#if defined(CC2650CUST_5XD)


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
//static uint8_t errDetect=0;

/*******************************************************************************
 * EXTERNAL VARIABLES
 */
extern int AccelSensor_isSensorInterruptEnabled(void);
extern void AccelSensor_sendSensorIntEvent(void);
/*********************************************************************
 * LOCAL VARIABLES
 */

// PIN configuration structure to set all KEY pins as inputs with pullups enabled
PIN_Config gpioPinsCfg[] =
{
	Board_ACC_INT1 	| PIN_INPUT_EN  | PIN_NOPULL | PIN_HYSTERESIS,
	Board_ACC_TEST  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW    | PIN_PUSHPULL,

    PIN_TERMINATE
};

PIN_State  gpioPinsState;
PIN_Handle hGpioPins;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void test_ACC_output(uint8_t value)
{
	PIN_setOutputValue(hGpioPins, Board_ACC_TEST, value);
}

void GpioIntCallbackFunction(PIN_Handle handle, PIN_Id pinId)
{
	//set event in SBP task to process outside of hwi context
	//test_ACC_output(1);
	if(AccelSensor_isSensorInterruptEnabled())
		AccelSensor_sendSensorIntEvent();
	//test_ACC_output(0);
}

/*********************************************************************
 * @fn      Board_initKeys
 *
 * @brief   Enable interrupts for keys on GPIOs.
 *
 * @param   appKeyCB - application key pressed callback
 *
 * @return  none
 */
void Board_initGenericGpio(void)
{
//	PIN_Status status;
  // Initialize KEY pins. Enable int after callback registered
	hGpioPins = PIN_open(&gpioPinsState, gpioPinsCfg);
//  PIN_registerIntCb(hGpioPins, Board_keyCallback);

	//PIN_setConfig(hGpioPins, PIN_BM_INPUT_MODE, Board_KEY_DOWN | PIN_INPUT_EN | PIN_HYSTERESIS);
	//PIN_setConfig(hGpioPins, PIN_BM_IRQ, Board_MOTOR_PWR | PIN_IRQ_NEGEDGE);
	//  if(status != PIN_SUCCESS)
	//	  while(1);

	PIN_setInterrupt(hGpioPins, Board_ACC_INT1|PIN_IRQ_POSEDGE); //PIN_IRQ_NEGEDGE
	//PIN_setConfig(hGpioPins, PIN_BM_IRQ, Board_ACC_INT1 | PIN_IRQ_POSEDGE);
	PIN_registerIntCb(hGpioPins, &GpioIntCallbackFunction);

#ifdef POWER_SAVING
	//Enable wakeup
	PIN_setConfig(hGpioPins, PINCC26XX_BM_WAKEUP, Board_ACC_INT1 | PINCC26XX_BM_WAKEUP);
#endif //POWER_SAVING
  
}

#endif

/*********************************************************************
*********************************************************************/
