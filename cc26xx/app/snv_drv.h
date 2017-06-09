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

#ifndef SNV_DRV_H
#define SNV_DRV_H

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

/*********************************************************************
 * TYPEDEFS
 */
enum {
	SNV_INIT_FLAG_ITEM		= 0,
	SNV_ACCEL_DATARATE_ITEM,
	SNV_ACCEL_SCALE_ITEM,
	SNV_ACCEL_ZERO_G_ITEM,

	SNV_ITEM_MAX,
};

typedef struct {
	int		flag;		// 0x55555555: initialized, else: not initialized
	int 	odr;		// 0 ~ 9 : 0 ~ 5.3KHz
	int		scale;		// 0 ~ 3 : 2g ~ 16g
	int		zero_g[3];  // gravity compensation value for each axis

} SNV_data_st;

/*********************************************************************
 * MACROS
 */
//================== SNV ID definition ============================
#define BLE_NVID_CUST_START             0x80  //!< Start of the Customer's NV IDs
#define BLE_NVID_CUST_END               0x80  // 0x8F  //!< End of the Customer's NV IDs


#define BLE_NVID_USER_01				(BLE_NVID_CUST_START)
#define BLE_NVID_USER_02				(BLE_NVID_CUST_START+1)
#define BLE_NVID_USER_03				(BLE_NVID_CUST_START+2)
#define BLE_NVID_USER_04				(BLE_NVID_CUST_START+3)
#define BLE_NVID_USER_05				(BLE_NVID_CUST_START+4)
#define BLE_NVID_USER_06				(BLE_NVID_CUST_START+5)
#define BLE_NVID_USER_07				(BLE_NVID_CUST_START+6)
#define BLE_NVID_USER_08				(BLE_NVID_CUST_START+7)
#define BLE_NVID_USER_09				(BLE_NVID_CUST_START+8)
#define BLE_NVID_USER_10				(BLE_NVID_CUST_START+9)
#define BLE_NVID_USER_11				(BLE_NVID_CUST_START+10)
#define BLE_NVID_USER_12				(BLE_NVID_CUST_START+11)
#define BLE_NVID_USER_13				(BLE_NVID_CUST_START+12)
#define BLE_NVID_USER_14				(BLE_NVID_CUST_START+13)
#define BLE_NVID_USER_15				(BLE_NVID_CUST_START+14)
#define BLE_NVID_USER_16				(BLE_NVID_CUST_START+15)


#define ACCEL_SNV_START					BLE_NVID_USER_01


#define SNV_INITIALIZED			0x55555555

#define SNV_SUCCESS						0
#define SNV_READ_ERROR					1
#define SNV_WRITE_ERROR					2
#define SNV_NORMAL_ERROR				3

/*********************************************************************
 * API FUNCTIONS
 */

extern uint8_t SNV_init(void);
extern void SNV_applyInitalValue(void);
extern int SNV_isInitialized(void);
extern int SNV_getItemValue(int snv_id);
extern uint8_t SNV_setItemValue(int snv_id, uint8_t value);
extern uint8_t SNV_getMultiItem(int snv_id, int *value);
extern uint8_t SNV_setMultiItem(int snv_id, int *value);
/*********************************************************************
*********************************************************************/  

#ifdef __cplusplus
}
#endif

#endif /* SNV_DRV_H */
