/******************************************************************************

 @file  accelerometer.c

 @brief Accelerometer Profile for CC26xx.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2009-2016, Texas Instruments Incorporated
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
#include "bcomdef.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "accelerometer.h"
#include "AccelSensorTask.h"

#include <ti/mw/display/Display.h>

extern Display_Handle dispHandle;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#if ACCELSENSOR_MERGE_CALC_RESULT
#define SERVAPP_NUM_ATTR_SUPPORTED        12
#else
#define SERVAPP_NUM_ATTR_SUPPORTED        23
#endif

#define ACCEL_DATA_AVERAGE_POS			5
#define ACCEL_DATA_DEVIATION_POS		9
#define ACCEL_DATA_MAXIMUM_POS			13
#define ACCEL_DATA_MINIMUM_POS			17

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Accelerometer Service UUID
CONST uint8 accServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ACCEL_SERVICE_UUID), HI_UINT16(ACCEL_SERVICE_UUID)
};

// Accelerometer init UUID : 0xFFA5
CONST uint8 sensInitUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ACCEL_DATA_INIT_UUID), HI_UINT16(ACCEL_DATA_INIT_UUID)
};
#if ACCELSENSOR_MERGE_CALC_RESULT
// Accelerometer Sensor Data UUID : 0xFFA5
CONST uint8 sensorAveDivUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ACCEL_SENSOR_AVE_DIV_UUID), HI_UINT16(ACCEL_SENSOR_AVE_DIV_UUID)
};

CONST uint8 sensorMaxMinUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ACCEL_SENSOR_MAX_MIN_UUID), HI_UINT16(ACCEL_SENSOR_MAX_MIN_UUID)
};
#else
// Accelerometer Average UUID : 0xFFA2
CONST uint8 averageUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ACCEL_AVERAGE_UUID), HI_UINT16(ACCEL_AVERAGE_UUID)
};

// Accelerometer Deviation UUID : 0xFFA3
CONST uint8 deviationUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ACCEL_DEVIATION_UUID), HI_UINT16(ACCEL_DEVIATION_UUID)
};

// Accelerometer max UUID : 0xFFA4
CONST uint8 maxUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ACCEL_MAX_UUID), HI_UINT16(ACCEL_MAX_UUID)
};

// Accelerometer min UUID : 0xFFA5
CONST uint8 minUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ACCEL_MIN_UUID), HI_UINT16(ACCEL_MIN_UUID)
};
#endif

// Accelerometer noti UUID : 0xFFA5
//CONST uint8 nofiCfgUUID[ATT_BT_UUID_SIZE] =
//{
//  LO_UINT16(ACCEL_NOTI_CFG_UUID), HI_UINT16(ACCEL_NOTI_CFG_UUID)
//};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static accelCBs_t *accel_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Accelerometer Service attribute
static CONST gattAttrType_t accelService = { ATT_BT_UUID_SIZE, accServUUID };

// Characteristic Properties
static uint8 accelSensInitCharProps = GATT_PROP_READ | GATT_PROP_WRITE;
#if ACCELSENSOR_MERGE_CALC_RESULT
static uint8 accelSensorAveDivCharProps = GATT_PROP_NOTIFY;
static uint8 accelSensorMaxMinCharProps = GATT_PROP_NOTIFY;
#else
static uint8 accelAverageCharProps = GATT_PROP_NOTIFY;
static uint8 accelDeviationCharProps = GATT_PROP_NOTIFY;
static uint8 accelSensorMaxCharProps = GATT_PROP_NOTIFY;
static uint8 accelSensorMinCharProps = GATT_PROP_NOTIFY;
#endif

static uint8 accelSensInit = FALSE;
//static uint8 accelNofiCfg = 0; //disable
// Characteristics value, 3 valuse for x, y, z
#if ACCELSENSOR_MERGE_CALC_RESULT
static int16 accelSensorAveDiv[ACCEL_SENSOR_CALC_LEN] = {0,0,0, 0,0,0};
static int16 accelSensorMaxMin[ACCEL_SENSOR_CALC_LEN] = {0,0,0, 0,0,0};
#else
static int16 accelAverage[ACCEL_SENSOR_DATA_LEN] = {0,0,0};
static int16 accelDeviation[ACCEL_SENSOR_DATA_LEN] = {0,0,0};
static int16 accelSensorMax[ACCEL_SENSOR_DATA_LEN] = {0,0,0};
static int16 accelSensorMin[ACCEL_SENSOR_DATA_LEN] = {0,0,0};
#endif


// Characteristic Configs for notify
#if ACCELSENSOR_MERGE_CALC_RESULT
static gattCharCfg_t *accelAveDivConfig;
static gattCharCfg_t *accelMaxMinConfig;
#else
static gattCharCfg_t *accelAverageConfig;
static gattCharCfg_t *accelDeviationConfig;
static gattCharCfg_t *accelSensorMaxConfig;
static gattCharCfg_t *accelSensorMinConfig;
#endif
//static gattCharCfg_t *accelSensorNofiConfig;

// Characteristic user descriptions
static uint8 accelSensInitUserDesc[20] 			= "Accel Data Init   ";
#if ACCELSENSOR_MERGE_CALC_RESULT
static uint8 accelSensorAveDivCharUserDesc[20] 	= "Accel Sensor AveDiv";
static uint8 accelSensorMaxMinCharUserDesc[20] 	= "Accel Sensor MaxMin";
#else
static uint8 accelAverageCharUserDesc[20] 		= "Accel Average Data ";
static uint8 accelDeviationCharUserDesc[22] 	= "Accel Deviation Data  ";
static uint8 accelSensorDataMaxUserDesc[20] 	= "Accel Sensor MAX  ";
static uint8 accelSensorDataMinUserDesc[20] 	= "Accel Sensor MIN  ";
#endif

/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t accelAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Accelerometer Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&accelService                  /* pValue */
  },

  	 //===============================================
	// Sensor data init Characteristic Declaration
	{
		  { ATT_BT_UUID_SIZE, characterUUID }, //type 0x2803
		  GATT_PERMIT_READ,						// permissions
		  0,									// handle
		  &accelSensInitCharProps				// pValue
	},

	  // Sensor Min Characteristic Value
	  {
		{ ATT_BT_UUID_SIZE, sensInitUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		&accelSensInit
	  },

	  // Sensor Min Characteristic User Description
	  {
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		accelSensInitUserDesc
	  },

#if ACCELSENSOR_MERGE_CALC_RESULT //hrjung merge notify data to one parameter
	  //===============================================
		// Accelero sensor calc Characteristic Declaration
		{
		  { ATT_BT_UUID_SIZE, characterUUID }, //type 0x2803
		  GATT_PERMIT_READ,						// permissions
		  0,									// handle
		  &accelSensorAveDivCharProps				// pValue
		},

		  // sensor calc Characteristic Value
		  {
			{ ATT_BT_UUID_SIZE, sensorAveDivUUID },
			GATT_PERMIT_READ,
			0,
			(uint8 *)accelSensorAveDiv
		  },

	      // sensor calc configuration
	      {
	        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
	        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	        0,
	        (uint8 *)&accelAveDivConfig
	      },

		  // sensor calc Characteristic User Description
		  {
			{ ATT_BT_UUID_SIZE, charUserDescUUID },
			GATT_PERMIT_READ,
			0,
			accelSensorAveDivCharUserDesc
		  },
		  //===============================================
			// Accelero sensor calc Characteristic Declaration
			{
			  { ATT_BT_UUID_SIZE, characterUUID }, //type 0x2803
			  GATT_PERMIT_READ,						// permissions
			  0,									// handle
			  &accelSensorMaxMinCharProps			// pValue
			},

			  // sensor calc Characteristic Value
			  {
				{ ATT_BT_UUID_SIZE, sensorMaxMinUUID },
				GATT_PERMIT_READ,
				0,
				(uint8 *)accelSensorMaxMin
			  },

		      // sensor calc configuration
		      {
		        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
		        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		        0,
		        (uint8 *)&accelMaxMinConfig
		      },

			  // sensor calc Characteristic User Description
			  {
				{ ATT_BT_UUID_SIZE, charUserDescUUID },
				GATT_PERMIT_READ,
				0,
				accelSensorMaxMinCharUserDesc
			  },
#else
	//===============================================
	// Accelero average data Characteristic Declaration
	{
	  { ATT_BT_UUID_SIZE, characterUUID }, //type 0x2803
	  GATT_PERMIT_READ,						// permissions
	  0,									// handle
	  &accelAverageCharProps				// pValue
	},

	  // average data Characteristic Value
	  {
		{ ATT_BT_UUID_SIZE, averageUUID },
		GATT_PERMIT_READ,
		0,
		(uint8 *)accelAverage
	  },

      // average data configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&accelAverageConfig
      },

	  // average data Characteristic User Description
	  {
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		accelAverageCharUserDesc
	  },

	//===============================================
	// Accelero deviation data Characteristic Declaration
	{
	  { ATT_BT_UUID_SIZE, characterUUID }, //type 0x2803
	  GATT_PERMIT_READ,						// permissions
	  0,									// handle
	  &accelDeviationCharProps				// pValue
	},
	  // deviation data Characteristic Value
	  {
		{ ATT_BT_UUID_SIZE, deviationUUID },
		GATT_PERMIT_READ,
		0,
		(uint8 *)accelDeviation
	  },

      // deviation data configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&accelDeviationConfig
      },

	  // deviation data Characteristic User Description
	  {
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		accelDeviationCharUserDesc
	  },

	  //===============================================
	  // Sensor Max Characteristic Value
		// Sensor Max data Characteristic Declaration
	  {
		  { ATT_BT_UUID_SIZE, characterUUID }, //type 0x2803
		  GATT_PERMIT_READ,						// permissions
		  0,									// handle
		  &accelSensorMaxCharProps				// pValue
	  },

	  {
		{ ATT_BT_UUID_SIZE, maxUUID },
		GATT_PERMIT_READ,
		0,
		(uint8 *)accelSensorMax
	  },

      // Sensor Max data configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&accelSensorMaxConfig
      },

	  // Sensor max Characteristic User Description
	  {
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		accelSensorDataMaxUserDesc
	  },

	  //===============================================
	// Sensor Min data Characteristic Declaration
	{
		  { ATT_BT_UUID_SIZE, characterUUID }, //type 0x2803
		  GATT_PERMIT_READ,						// permissions
		  0,									// handle
		  &accelSensorMinCharProps				// pValue
	},

	  // Sensor Min Characteristic Value
	  {
		{ ATT_BT_UUID_SIZE, minUUID },
		GATT_PERMIT_READ,
		0,
		(uint8 *)accelSensorMin
	  },

      // Sensor Min data configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&accelSensorMinConfig
      },

	  // Sensor Min Characteristic User Description
	  {
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ,
		0,
		accelSensorDataMinUserDesc
	  },
#endif

//	    // Accel Notify config Characteristic Declaration
//	    {
//	       { ATT_BT_UUID_SIZE, characterUUID },
//	       GATT_PERMIT_READ,
//	       0,
//	       &accelSensorNotiCfgCharProps
//	    },
//
//	      // Accelerometer Enable Characteristic Value
//	      {
//	        { ATT_BT_UUID_SIZE, nofiCfgUUID },
//	        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
//	        0,
//	        &accelNofiCfg
//	      },
//
//	      // Accelerometer Enable User Description
//	      {
//	        { ATT_BT_UUID_SIZE, charUserDescUUID },
//	        GATT_PERMIT_READ,
//	        0,
//	        (uint8*)&accelSensorNotiCfgUserDesc
//	      },

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t accel_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr, 
                                  uint8_t *pValue, uint16_t *pLen,
                                  uint16_t offset, uint16_t maxLen,
                                  uint8_t method);

static bStatus_t accel_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t len,
                                   uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Accelerometer Service Callbacks
// Note: When an operation on a characteristic requires authorization and 
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the 
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an 
// operation on a characteristic requires authorization the Stack will call 
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be 
// made within these functions.
CONST gattServiceCBs_t  accelCBs =
{
  accel_ReadAttrCB,  // Read callback function pointer
  accel_WriteAttrCB, // Write callback function pointer
  NULL               // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Accel_AddService
 *
 * @brief   Initializes the Accelerometer service by
 *          registering GATT attributes with the GATT server. Only
 *          call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Accel_AddService(uint32 services)
{
  uint8 status = SUCCESS;
  size_t allocSize = sizeof(gattCharCfg_t) * linkDBNumConns;

  // Allocate Client Characteristic Configuration tables
#if ACCELSENSOR_MERGE_CALC_RESULT
  accelAveDivConfig = (gattCharCfg_t *)ICall_malloc(allocSize);
  if (accelAveDivConfig == NULL)
  {
    return (bleMemAllocError);
  }

  accelMaxMinConfig = (gattCharCfg_t *)ICall_malloc(allocSize);
  if (accelMaxMinConfig == NULL)
  {
	ICall_free(accelAveDivConfig);
    return (bleMemAllocError);
  }
#else
  accelAverageConfig = (gattCharCfg_t *)ICall_malloc(allocSize);
  if (accelAverageConfig == NULL)
  {
    return (bleMemAllocError);
  }
    
  accelDeviationConfig = (gattCharCfg_t *)ICall_malloc(allocSize);
  if (accelDeviationConfig == NULL)
  {
    // Free already allocated data
    ICall_free(accelAverageConfig);
      
    return (bleMemAllocError);
  }
  
  accelSensorMaxConfig = (gattCharCfg_t *)ICall_malloc(allocSize);
  if (accelSensorMaxConfig == NULL)
  {
    // Free already allocated data
    ICall_free(accelAverageConfig);
    ICall_free(accelDeviationConfig);

    return (bleMemAllocError);
  }

  accelSensorMinConfig = (gattCharCfg_t *)ICall_malloc(allocSize);
  if (accelSensorMinConfig == NULL)
  {
    // Free already allocated data
    ICall_free(accelAverageConfig);
    ICall_free(accelDeviationConfig);
    ICall_free(accelSensorMaxConfig);

    return (bleMemAllocError);
  }
#endif

  // Initialize Client Characteristic Configuration attributes
#if ACCELSENSOR_MERGE_CALC_RESULT
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, accelAveDivConfig);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, accelMaxMinConfig);
#else
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, accelAverageConfig);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, accelDeviationConfig);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, accelSensorMaxConfig);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, accelSensorMinConfig);
#endif

  if (services & ACCEL_SERVICE)
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(accelAttrTbl, 
                                         GATT_NUM_ATTRS(accelAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &accelCBs);
  }

  return (status);
}

/*********************************************************************
 * @fn      Accel_RegisterAppCBs
 *
 * @brief   Does the profile initialization.  Only call this function
 *          once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t Accel_RegisterAppCBs(accelCBs_t *appCallbacks)
{
  if (appCallbacks)
  {
    accel_AppCBs = appCallbacks;
    
    return (SUCCESS);
  }
  else
  {
    return (bleAlreadyInRequestedMode);
  }
}

/*********************************************************************
 * @fn      Accel_SetParameter
 *
 * @brief   Set an Accelerometer Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len   - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *                  the parameter ID and WILL be cast to the appropriate 
 *                  data type (example: data type of uint16 will be cast to 
 *                  uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Accel_SetParameter(uint16 param, uint8 len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case ACCEL_DATA_INIT:
      if (len == sizeof(uint8))
    	  accelSensInit = *((uint8*)value);
      else
    	  ret = bleInvalidRange;

      break;
#if ACCELSENSOR_MERGE_CALC_RESULT
    case ACCEL_AVE_DIV:
      if (len == ACCEL_SENSOR_CALC_LEN*sizeof(int16))
      {
    	  memcpy(accelSensorAveDiv, value, len);
          // See if Notification has been enabled
          GATTServApp_ProcessCharCfg( accelAveDivConfig, (uint8 *)accelSensorAveDiv, FALSE,
        		  	  	  	  	  	  accelAttrTbl, GATT_NUM_ATTRS( accelAttrTbl ),
                                      INVALID_TASK_ID, accel_ReadAttrCB );
      }
      else
        ret = bleInvalidRange;

      break;

    case ACCEL_MAX_MIN:
      if (len == ACCEL_SENSOR_CALC_LEN*sizeof(int16))
      {
    	  memcpy(accelSensorMaxMin, value, len);
          // See if Notification has been enabled
          GATTServApp_ProcessCharCfg( accelMaxMinConfig, (uint8 *)accelSensorMaxMin, FALSE,
        		  	  	  	  	  	  accelAttrTbl, GATT_NUM_ATTRS( accelAttrTbl ),
                                      INVALID_TASK_ID, accel_ReadAttrCB );
      }
      else
        ret = bleInvalidRange;

      break;
#else
    case ACCEL_AVERAGE:
      if (len == ACCEL_SENSOR_DATA_LEN*sizeof(int16))
      {
    	  memcpy(accelAverage, value, sizeof(int16)*ACCEL_SENSOR_DATA_LEN);
          // See if Notification has been enabled
          GATTServApp_ProcessCharCfg( accelAverageConfig, (uint8 *)accelAverage, FALSE,
        		  	  	  	  	  	  accelAttrTbl, GATT_NUM_ATTRS( accelAttrTbl ),
                                      INVALID_TASK_ID, accel_ReadAttrCB );
      }
      else
        ret = bleInvalidRange;

      break;

    case ACCEL_DEVIATION:
      if (len == ACCEL_SENSOR_DATA_LEN*sizeof(int16))
      {
    	  memcpy(accelDeviation, value, sizeof(int16)*ACCEL_SENSOR_DATA_LEN);
          GATTServApp_ProcessCharCfg( accelDeviationConfig, (uint8 *)accelDeviation, FALSE,
        		  	  	  	  	  	  accelAttrTbl, GATT_NUM_ATTRS( accelAttrTbl ),
                                      INVALID_TASK_ID, accel_ReadAttrCB );
      }
      else
        ret = bleInvalidRange;

      break;

    case ACCEL_MAX:
      if (len == ACCEL_SENSOR_DATA_LEN*sizeof(int16))
      {
    	  memcpy(accelSensorMax, value, sizeof(int16)*ACCEL_SENSOR_DATA_LEN);
          GATTServApp_ProcessCharCfg( accelSensorMaxConfig, (uint8 *)accelSensorMax, FALSE,
        		  	  	  	  	  	  accelAttrTbl, GATT_NUM_ATTRS( accelAttrTbl ),
                                      INVALID_TASK_ID, accel_ReadAttrCB );
      }
      else
        ret = bleInvalidRange;

      break;

    case ACCEL_MIN:
      if (len == ACCEL_SENSOR_DATA_LEN*sizeof(int16))
      {
    	  memcpy(accelSensorMin, value, sizeof(int16)*ACCEL_SENSOR_DATA_LEN);
          GATTServApp_ProcessCharCfg( accelSensorMinConfig, (uint8 *)accelSensorMin, FALSE,
        		  	  	  	  	  	  accelAttrTbl, GATT_NUM_ATTRS( accelAttrTbl ),
                                      INVALID_TASK_ID, accel_ReadAttrCB );
      }
      else
        ret = bleInvalidRange;

      break;
#endif
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return (ret);
}

/*********************************************************************
 * @fn      Accel_GetParameter
 *
 * @brief   Get an Accelerometer Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *                  the parameter ID and WILL be cast to the appropriate 
 *                  data type (example: data type of uint16 will be cast to 
 *                  uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Accel_GetParameter(uint16 param, void *value)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
	case ACCEL_DATA_INIT:
		*((uint8*)value) = accelSensInit;
		break;

//	case ACCEL_NOTIFY_CFG:
//		*((uint8*)value) = accelNofiCfg;
//		break;
#if ACCELSENSOR_MERGE_CALC_RESULT
    case ACCEL_AVE_DIV:
      memcpy(value, accelSensorAveDiv, sizeof(int16)*ACCEL_SENSOR_CALC_LEN);
      break;

    case ACCEL_MAX_MIN:
      memcpy(value, accelSensorMaxMin, sizeof(int16)*ACCEL_SENSOR_CALC_LEN);
      break;
#else
    case ACCEL_AVERAGE:
      memcpy(value, accelAverage, sizeof(int16)*ACCEL_SENSOR_DATA_LEN);
      break;

    case ACCEL_DEVIATION:
      memcpy(value, accelDeviation, sizeof(int16)*ACCEL_SENSOR_DATA_LEN);
      break;

    case ACCEL_MAX:
      memcpy(value, accelSensorMax, sizeof(int16)*ACCEL_SENSOR_DATA_LEN);
      break;

    case ACCEL_MIN:
      memcpy(value, accelSensorMin, sizeof(int16)*ACCEL_SENSOR_DATA_LEN);
      break;
#endif
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return (ret);
}

/*********************************************************************
 * @fn          accel_ReadAttr
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr      - pointer to attribute
 * @param       pValue     - pointer to data to be read
 * @param       pLen       - length of data to be read
 * @param       offset     - offset of the first octet to be read
 * @param       maxLen     - maximum length of data to be read
 * @param       method     - type of read message 
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t accel_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr, 
                                  uint8_t *pValue, uint16_t *pLen,
                                  uint16_t offset, uint16_t maxLen,
                                  uint8_t method)
{
  uint16 uuid;
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  Display_print0(dispHandle, 2, 0, "accel_ReadAttrCB");
  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {    
    // 16-bit UUID
    uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch (uuid)
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those types for reads
      case ACCEL_DATA_INIT_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
#if ACCELSENSOR_MERGE_CALC_RESULT
      case ACCEL_SENSOR_AVE_DIV_UUID:
        *pLen = sizeof(int16)*ACCEL_SENSOR_CALC_LEN;
        memcpy(pValue, pAttr->pValue, *pLen);
        //Display_print4(dispHandle, 2, 0, "accel_ReadAttrCB len=%d val=%d %d %d", *pLen, pValue[0],pValue[1],pValue[2]);
        break;

      case ACCEL_SENSOR_MAX_MIN_UUID:
        *pLen = sizeof(int16)*ACCEL_SENSOR_CALC_LEN;
        memcpy(pValue, pAttr->pValue, *pLen);
        //Display_print4(dispHandle, 2, 0, "accel_ReadAttrCB len=%d val=%d %d %d", *pLen, pValue[0],pValue[1],pValue[2]);
        break;
#else
      case ACCEL_AVERAGE_UUID:
      case ACCEL_DEVIATION_UUID:
      case ACCEL_MAX_UUID:
      case ACCEL_MIN_UUID:
        *pLen = sizeof(int16)*ACCEL_SENSOR_DATA_LEN;
        memcpy(pValue, pAttr->pValue, *pLen);
        break;
#endif
      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }
  
  return (status);
}

/*********************************************************************
 * @fn      accel_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr      - pointer to attribute
 * @param   pValue     - pointer to data to be written
 * @param   len        - length of data
 * @param   offset     - offset of the first octet to be written
 * @param   method     - type of write message 
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t accel_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t len,
                                   uint16_t offset, uint8_t method)
{
	bStatus_t status = SUCCESS;
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

    Display_print3(dispHandle, 2, 0, "accel_WriteAttrCB len=%d %d uuid=0x%x", pAttr->type.len, len, uuid);
    if (offset > 0)
    {
      return (ATT_ERR_ATTR_NOT_LONG);
    }

    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {

		switch (uuid)
		{
	      case ACCEL_DATA_INIT_UUID:
			// Validate the value.
			// Make sure it's not a blob operation.
			if (len > 1)
			{
				status = ATT_ERR_INVALID_VALUE_SIZE;
			}

			// Write the value.
			if (status == SUCCESS)
			{
			      uint8 *pCurValue = (uint8 *)pAttr->pValue;

			      *pCurValue = pValue[0];

				  if (pValue[0] != FALSE && pValue[0] != TRUE)
					  status = ATT_ERR_INVALID_VALUE;
				  else
				  {
					  if(uuid == ACCEL_DATA_INIT_UUID)
						  accel_AppCBs->pfnAccelEnabler(ACCEL_SENSOR_DATA_INIT, *pCurValue);
					  else
						  status = ATT_ERR_ATTR_NOT_FOUND;
				  }
			}

			break;

		  case GATT_CLIENT_CHAR_CFG_UUID:
		  {
			  //set 2 byte pValue forcely, if not return length error status
			  uint16_t val=(uint16_t)pValue[0];
			  uint16_t length = sizeof(uint16_t);
			//status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
			  status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, (uint8_t *)&val, length,
													 offset, GATT_CLIENT_CFG_NOTIFY);
//			if (status == SUCCESS)
//			{
//			  uint8 charCfg = pValue[0];
//
//			  accel_AppCBs->pfnAccelEnabler(ACCEL_MEAS_ENABLE_NOTIFY, charCfg);
//			}
			  Display_print3(dispHandle, 2, 0, "accel_WriteAttrCB len=%d status=0x%x pVal=%d", length, status, val);
		  }
			break;

		  default:
			  // Should never get here!
			  status = ATT_ERR_ATTR_NOT_FOUND;
		}
    }
    else
    {
		// 128-bit UUID
		status = ATT_ERR_INVALID_HANDLE;
    }

  // If an attribute changed then callback function to notify application of 
  // change.
//  if ((notify != 0xFF) && accel_AppCBs && accel_AppCBs->pfnAccelEnabler)
//  {
//    accel_AppCBs->pfnAccelEnabler();
//  }
  
  return (status);
}
/*********************************************************************
*********************************************************************/
