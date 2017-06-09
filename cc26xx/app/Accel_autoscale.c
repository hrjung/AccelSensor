/*******************************************************************************
  Filename:       Accel_autoscale.c
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
  PROVIDED �뜝�럥�맶�뜝�럥�쑅�뜝�럥�쓠�뜝�럥�맶�뜝�럥�쑅嶺뚯빓�뿥占쎌맶�뜝�럥�쑅�뜝�럥六� IS占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈윫�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪占쎈쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include <math.h>

#include <ti/mw/display/Display.h>

#include "AccelSensorTask.h"
#include "Accel_drv.h"

#include "snv_drv.h"
#include "Accel_autoscale.h"

extern int AccelDbg_isDispLogOn(void);

extern void AccelSensor_changeScaleEvent(int scale);

extern int AccelSensor_getCycle(void);
extern int AccelSensor_isCycleDone(void);
/*********************************************************************
 * TYPEDEFS
 */
#define SCALE_UPPER_LIMIT			0x70
#define SCALE_DOWN_SCALE_LIMIT		0x30

#define SCALE_THRESHOLD_COUNT		10

/*********************************************************************
 * LOCAL FUNCTIONS
 */


/*******************************************************************************
 * EXTERNAL VARIABLES
 */

extern Display_Handle dispHandle;

/*********************************************************************
 * LOCAL VARIABLES
 */

static int autoscale_on=0; // default off
static int scale_idx=ACCEL_SCALE_DEFAULT;

#ifdef ACCELSENSOR_AUTOSCALE
static int8_t sensor_batch[3][MAX_SAMPLE_COUNT];
static int8_t sens_max[3][ACCEL_SENSOR_POINT_COUNT];
static int8_t sens_min[3][ACCEL_SENSOR_POINT_COUNT];
#endif
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

int AccelScale_isOn(void)
{
	return autoscale_on;
}

void AccelScale_setAutoScale(int on_off)
{
	autoscale_on = on_off;
}

#ifdef ACCELSENSOR_AUTOSCALE
void AccelScale_initOneSample(void)
{
	int i;

	for(i=0; i<MAX_SAMPLE_COUNT; i++)
	{
		sensor_batch[0][i] = 0;
		sensor_batch[1][i] = 0;
		sensor_batch[2][i] = 0;
	}
}

void AccelScale_initMinMax(void)
{
	int i;

	for(i=0; i<ACCEL_SENSOR_POINT_COUNT; i++)
	{
		sens_max[0][i] = 0x80;
		sens_max[1][i] = 0x80;
		sens_max[2][i] = 0x80;

		sens_min[0][i] = 0x7F;
		sens_min[1][i] = 0x7F;
		sens_min[2][i] = 0x7F;
	}
}

void AccelScale_storeOneSample(int idx, int8_t x, int8_t y, int8_t z)
{
	sensor_batch[0][idx] = x;
	sensor_batch[1][idx] = y;
	sensor_batch[2][idx] = z;

//	if(AccelDbg_isDispLogOn()==1)
//		Display_print4(dispHandle, 2, 0, "data=%d, 0x%x, 0x%x, 0x%x", idx, (int8)sensor_batch[0][idx], (int8)sensor_batch[1][idx], (int8)sensor_batch[2][idx]);
}

void AccelScale_storeMinMax(int16 index, int count)
{
	int i;

	for(i=0; i<count; i++)
	{
		if(sens_max[0][index] < sensor_batch[0][i] ) sens_max[0][index] = sensor_batch[0][i];
		if(sens_max[1][index] < sensor_batch[1][i] ) sens_max[1][index] = sensor_batch[1][i];
		if(sens_max[2][index] < sensor_batch[2][i] ) sens_max[2][index] = sensor_batch[2][i];

		if(sens_min[0][index] > sensor_batch[0][i] ) sens_min[0][index] = sensor_batch[0][i];
		if(sens_min[1][index] > sensor_batch[1][i] ) sens_min[1][index] = sensor_batch[1][i];
		if(sens_min[2][index] > sensor_batch[2][i] ) sens_min[2][index] = sensor_batch[2][i];
	}
}
#endif

void AccelScale_changeScale(uint8_t scale)
{
	uint8_t result=0;
	int pre_scale;

	pre_scale = SNV_getItemValue(SNV_ACCEL_SCALE_ITEM);
	if(scale != pre_scale)
	{
		result = SNV_setItemValue(SNV_ACCEL_SCALE_ITEM, scale);
		if(result != SNV_SUCCESS)
		{
			Display_print0(dispHandle, 2, 0, "SNV scale update error");
			AccelSensor_error();
		}
		Display_print2(dispHandle, 3, 0, "scale changed %d -> %d", pre_scale, scale);
	}
}

#ifdef ACCELSENSOR_AUTOSCALE
void AccelScale_needScaleUp(void)
{
	int i, up_count=0;
	//int threshold = 10; //(sample_count*ACCEL_SENSOR_POINT_COUNT/10);

	if(AccelSensor_isCycleDone() == 0) return; //need to complete at least one cycle
//
//	for(i=0; i<sample_count; i++)
//	{
//		if(abs(sensor_batch[0][i]) > SCALE_UPPER_LIMIT
//			|| abs(sensor_batch[1][i]) > SCALE_UPPER_LIMIT
//			|| abs(sensor_batch[2][i]) > SCALE_UPPER_LIMIT)
//			up_count++;
//	}

	for(i=0; i<ACCEL_SENSOR_POINT_COUNT; i++)
	{
		if(abs(sens_max[0][i]) > SCALE_UPPER_LIMIT) up_count++;
		if(abs(sens_max[1][i]) > SCALE_UPPER_LIMIT) up_count++;
		if(abs(sens_max[2][i]) > SCALE_UPPER_LIMIT) up_count++;

		if(abs(sens_min[0][i]) > SCALE_UPPER_LIMIT) up_count++;
		if(abs(sens_min[1][i]) > SCALE_UPPER_LIMIT) up_count++;
		if(abs(sens_min[2][i]) > SCALE_UPPER_LIMIT) up_count++;

		if(up_count > SCALE_THRESHOLD_COUNT) break;
	}

	//Display_print2(dispHandle, 3, 0, "up_count %d thresh=%d", up_count, threshold);
	if(i < ACCEL_SENSOR_POINT_COUNT)
	{
		if(scale_idx < ACCEL_SCALE_16G)
		{
			scale_idx++;

			AccelSensor_changeScaleEvent(scale_idx);
			AccelSensor_restartCycle();
		}
//		else
//			Display_print1(dispHandle, 3, 0, "already have max scale %d", scale_idx);
	}
}

void AccelScale_needScaleDown(void)
{
	int i;

	if(AccelSensor_isCycleDone() == 0) return; //need to complete at least one cycle

	for(i=0; i<ACCEL_SENSOR_POINT_COUNT; i++)
	{
		if(abs(sens_max[0][i]) > SCALE_DOWN_SCALE_LIMIT) break;
		if(abs(sens_max[1][i]) > SCALE_DOWN_SCALE_LIMIT) break;
		if(abs(sens_max[2][i]) > SCALE_DOWN_SCALE_LIMIT) break;

		if(abs(sens_min[0][i]) > SCALE_DOWN_SCALE_LIMIT) break;
		if(abs(sens_min[1][i]) > SCALE_DOWN_SCALE_LIMIT) break;
		if(abs(sens_min[2][i]) > SCALE_DOWN_SCALE_LIMIT) break;
	}

	if(i < ACCEL_SENSOR_POINT_COUNT)
	{
		if(scale_idx > ACCEL_SCALE_2G)
		{
			scale_idx--;

			AccelSensor_changeScaleEvent(scale_idx);
			AccelSensor_restartCycle();
		}
//		else
//			Display_print1(dispHandle, 3, 0, "already have min scale %d", scale_idx);
	}
}
#endif
/*********************************************************************
*********************************************************************/
