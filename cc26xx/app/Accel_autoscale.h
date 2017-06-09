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
  PROVIDED 占쎈쐻占쎈윥占쎈읁占쎈쐻占쎈윥筌앷엥�쐻占쎈윥占쎈뻥 IS�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪占쎈쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef ACCEL_AUTOSCALE_H
#define ACCEL_AUTOSCALE_H

#ifdef __cplusplus
extern "C" {
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
#define 	ACCEL_SCALE_2G		0
#define 	ACCEL_SCALE_4G		1
#define 	ACCEL_SCALE_8G		2
#define 	ACCEL_SCALE_16G		3

#define 	ACCEL_SCALE_DEFAULT	1

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * API FUNCTIONS
 */
extern int AccelScale_isOn(void);
#ifdef ACCELSENSOR_AUTOSCALE
extern void AccelScale_initOneSample(void);
extern void AccelScale_storeOneSample(int idx, int8_t x, int8_t y, int8_t z);
extern void AccelScale_initMinMax(void);
extern void AccelScale_storeMinMax(int16 index, int count);
#endif

extern double AccelScale_getScale(void);

extern void AccelScale_changeScale(uint8_t scale);
#ifdef ACCELSENSOR_AUTOSCALE
extern void AccelScale_needScaleUp(void);
extern void AccelScale_needScaleDown(void);
#endif

/*********************************************************************
*********************************************************************/  

#ifdef __cplusplus
}
#endif

#endif /* ACCEL_AUTOSCALE_H */
