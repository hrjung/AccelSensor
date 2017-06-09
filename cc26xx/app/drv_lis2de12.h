/*******************************************************************************
  Filename:       drv_lis2de12.h
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

#ifndef BOARD_LIS2DE12_H
#define BOARD_LIS2DE12_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

// register address definition
#define LIS2DE12_ID_ADDR		0x0F

#define LIS2DE12_TEMP_STAT_ADDR	0x07
#define LIS2DE12_TEMP_OUTL_ADDR	0x0C
#define LIS2DE12_TEMP_OUTH_ADDR	0x0D
#define LIS2DE12_TEMP_CFG_ADDR	0x1F

#define LIS2DE12_CFG1_ADDR		0x20
#define LIS2DE12_CFG3_ADDR		0x22
#define LIS2DE12_CFG4_ADDR		0x23
#define LIS2DE12_CFG5_ADDR		0x24
#define LIS2DE12_CFG6_ADDR		0x25
#define LIS2DE12_STATUS_ADDR	0x27
#define LIS2DE12_OUT_ADDR		0x28
#define LIS2DE12_OUTX_ADDR		0x29
#define LIS2DE12_OUTY_ADDR		0x2B
#define LIS2DE12_OUTZ_ADDR		0x2D

#define LIS2DE12_FIFO_CTR_ADDR	0x2E
#define LIS2DE12_FIFO_SRC_ADDR	0x2F

#define LIS2DE12_INT1_CFG_ADDR	0x30
#define LIS2DE12_INT1_SRC_ADDR	0x31
#define LIS2DE12_INT1_THS_ADDR	0x32
#define LIS2DE12_INT1_DUR_ADDR	0x33

// register 0x0f device id
#define LIS2DE12_ID			0x33

// register 0x20 Data Rate mode(ODR) : default 0000
#define LIS2DE12_ODR_0		0x00
#define LIS2DE12_ODR_1		0x10
#define LIS2DE12_ODR_10		0x20
#define LIS2DE12_ODR_25		0x30
#define LIS2DE12_ODR_50		0x40
#define LIS2DE12_ODR_100	0x50
#define LIS2DE12_ODR_200	0x60
#define LIS2DE12_ODR_400	0x70
#define LIS2DE12_ODR_1_6K	0x80
#define LIS2DE12_ODR_5_4K	0x90

// register 0x22 interrupt source : data ready, watermark, overrun
#define LIS2DE12_INT_DRDY	0x10
#define LIS2DE12_INT_WTM	0x04
#define LIS2DE12_INT_OVRN	0x02

// register 0x23 Full scale selection : default 00
#define LIS2DE12_FS_2G		0x00
#define LIS2DE12_FS_4G		0x10
#define LIS2DE12_FS_8G		0x20
#define LIS2DE12_FS_16G		0x30

// register 0x2F FIFO interrupt source check
#define LIS2DE12_SRC_EMPTY	0x20
#define LIS2DE12_SRC_OVRN	0x40
#define LIS2DE12_SRC_WTM	0x80
#define LIS2DE12_WTM_CNT_MASK	0x1F

// register 0x25 memory clear
#define LIS2DE12_BOOT		0x80


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */
// LCD macros  diable
#define LCD_WRITE_STRING(str, line)
#define LCD_WRITE_STRING_VALUE(str, value, format, line)


/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      Board_openLis2de12
 *
 * @brief   Open LIS2DE12 on custom board.
 *
 * @param   none
 *
 * @return  none
 */
extern int Lis2de12_open(void);
extern void Lis2de12_close(void);
extern int Lis2de12_identify(void);
extern int Lis2de12_readVal(uint8_t *vals);
extern uint8_t Lis2de12_readConfig(int index);
extern uint8_t Lis2de12_writeConfig(int index, uint8_t value);
extern uint8_t Lis2de12_readRegValue(int cfg_index);
extern uint8_t Lis2de12_readStatusReg(void);
extern uint8_t Lis2de12_readFifoStatus(uint8_t *status);
extern uint8_t Lis2de12_resetSensor(void);
extern uint8_t Lis2de12_setPowerDown(void);
extern uint8_t Lis2de12_restorePowerMode(void);

extern uint8_t Lis2de12_setNormalSensor(void);
extern uint8_t Lis2de12_setIntSensor(void);

extern uint8_t Lis2de12_compConfig(void);
/*********************************************************************
*********************************************************************/  

#ifdef __cplusplus
}
#endif

#endif /* BOARD_BMA250_H */
