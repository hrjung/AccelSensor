/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== uartecho.c ========
 */

#ifdef USE_ACCEL_DEBUG_TASK

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
//#include <ti/drivers/UART.h>
#include <ti/mw/display/DisplayUART.h> //hrjung add for DisplayUart
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/* Example/Board Header files */
#include "Board.h"
#include "AccelSensorTask.h"

//#include "hci_tl.h"
//#include "gatt.h"
//#include "gapgattserver.h"
//#include "gattservapp.h"

//#include "gapbondmgr.h"

#include "osal_snv.h"
//#include "icall_apimsg.h"

#include "util.h"
#include "Accel_drv.h"
#include "snv_drv.h"
#include "AccelUartTask.h"

#include "board_led.h" //hrjung add

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Task configuration
#define DBG_TASK_STACK_SIZE     1024
#define DBG_TASK_PRIORITY       1

#define DBG_COMMAND_LENGTH		80

Task_Struct Dbg_taskStruct;
Char Dbg_taskStack[DBG_TASK_STACK_SIZE];

static UART_Handle DbgUartHandle;
const char echoPrompt[] = "\r\nNara>";

enum {
	NONE_TYPE = 0,
	BOOL_TYPE,
	INT_TYPE,

};

typedef enum {
	DBG_CMD_SHOW_HELP	= 0,
	DBG_CMD_DATA_RATE,
	DBG_CMD_SET_SCALE,
//	DBG_CMD_SET_DISP_ON,
	DBG_CMD_SET_PERIOD,
//	DBG_CMD_SMAPLE_CNT,
	DBG_CMD_READ_REG,
	DBG_CMD_READ_VAL,
	DBG_CMD_INIT_SNV,
	DBG_CMD_READ_SNV,
//	DBG_CMD_AUTOSCALE	= 11,
	DBG_CMD_VERSION,
	DBG_CMD_INIT_DATA,
	DBG_CMD_DISP_CNT,
	DBG_CMD_DISP_TIME,
	DBG_CMD_GET_ZERO_G,

	DBG_CMD_SET_INT,
	DBG_CMD_MOTOR_DETECT,

	DBG_CMD_MAX,
	DBG_CMD_EMPTY,

} dbg_cmd_code_e;

typedef enum {
	DBG_ERROR_SUCCESS		= 0,
	DBG_ERROR_INVALID_CMD	= 1,
	DBG_ERROR_WRONG_TYPE	= 2,
} dbg_cmd_err_e;


typedef struct {
	uint8_t	 code;  	// command code
	uint8_t	 cnt;	// paramter count
	uint8_t	 type;	// parameter type : number
	uint8_t	 reserv;
	uint32_t arg_val; // parameter value
	char 	*name;
	char 	*usage;
} dbg_cmd_tble_st;

dbg_cmd_tble_st dbgCmdTable[] = {
		{ DBG_CMD_SHOW_HELP, 0, NONE_TYPE, 0, 0, "HELP", " HELP : show this message" },
		{ DBG_CMD_DATA_RATE, 1, INT_TYPE, 0, 0, "DRATE", " DRATE [0 ~ 9] : set sensor data rate to be 0 ~ 5.3kbps" },
		{ DBG_CMD_SET_SCALE, 1, INT_TYPE, 0, 0, "SCALE", " SCALE [0 ~ 3] : set sensor scale option to be 2g, 4g, 8g, 16g scale" },
//		{ DBG_CMD_SET_DISP_ON, 1, INT_TYPE, 0, 0, "DISP", " DISP [0 ~ 2] display sensor value 0:off, 1:sensor log, 2:average, deviation " },
		{ DBG_CMD_SET_PERIOD, 1, INT_TYPE, 0, 0, "PERIOD", " PERIOD [100 ~ 5000] set sensor period 100ms ~ 5000ms " },
//		{ DBG_CMD_SMAPLE_CNT, 1, INT_TYPE, 0, 0, "SMPL", " SMPL [10 ~ 150] : set sensong data at a time 10 ~ 150 samples " },
		{ DBG_CMD_READ_REG, 1, INT_TYPE, 0, 0, "RREG", " RREG [0 ~ 1] : read sensor register value 0: ctrl1, 1:ctrl4 " },
		{ DBG_CMD_READ_VAL, 0, NONE_TYPE, 0, 0, "RVAL", " RVAL : read sensor data X, Y, Z " },
		{ DBG_CMD_INIT_SNV, 0, NONE_TYPE, 0, 0, "INITNV", " INITNV : initialize SNV data " },
		{ DBG_CMD_READ_SNV, 0, NONE_TYPE, 0, 0, "RDSNV", " RDSNV : read SNV data " },
//		{ DBG_CMD_AUTOSCALE, 0, NONE_TYPE, 0, 0, "ASCALE", " ASCALE [0 ~ 1] : autoscale on/off 0: off, 1: on " },
		{ DBG_CMD_VERSION, 0, NONE_TYPE, 0, 0, "VER", " VER : display FW version " },
		{ DBG_CMD_INIT_DATA, 0, NONE_TYPE, 0, 0, "INIT", " INIT : clear all data " },
		{ DBG_CMD_DISP_CNT, 1, INT_TYPE, 0, 0, "DISPN", " DISPN [1 ~ 50] display sensor value during n period " },
		{ DBG_CMD_DISP_TIME, 1, INT_TYPE, 0, 0, "DISPT", " DISPT [100 ~ 10000] display sensor value during t msec " },
		{ DBG_CMD_GET_ZERO_G, 0, NONE_TYPE, 0, 0, "ZEROG", " ZEROG : calculate gravity for each axis at steady state " },
		{ DBG_CMD_SET_INT, 1, INT_TYPE, 0, 0, "INTE", " INTE [0 ~ 1] 0: disable, 1: enable interrupt" },
		{ DBG_CMD_MOTOR_DETECT, 1, INT_TYPE, 0, 0, "DETM", " DETM [10 ~ 200] threshold value for detecting motor operation" },

		{ DBG_CMD_MAX, 0, NONE_TYPE, 0, 0, "", "" }
};

extern Display_Handle dispHandle;

extern void AccelSensor_setDebugEvent(uint8_t event);

extern int AccelSensor_getSensingPeriod(void);
extern int AccelSensor_setSensingPeriod(int period);
//extern int AccelSensor_getSensingCount(void);
//extern int AccelSensor_setSensingCount(int period);
extern int AccelSensor_isSensorInterruptEnabled(void);
extern void AccelSensor_setSensorInterrupt(int enabled);
extern void AccelSensor_getZeroG(int *zero_g);
extern void AccelSensor_getZeroG_SNV(int *zero_g);

extern void AccelScale_setAutoScale(int on_off);
extern int AccelScale_isOn(void);

extern int SNV_forceInit(void);
extern uint8 SNV_readValue(SNV_data_st *val);

//extern void test_ACC_output(void);

#define	CHAR_BACK_SPACE		0x08
#define CHAR_DELETE			0x7F

#define isLowerCapital(input)    	(input >= 'a' && input <= 'z')
#define isValidInput(input, pos) 	(input >= ' ' && input <= 'z' && pos < DBG_COMMAND_LENGTH-1)
#define isEndOfInput(input, pos) 	(input == '\r' || input == '\n' || pos >= DBG_COMMAND_LENGTH)
#define isBackSpace(input, pos) 	(input == CHAR_BACK_SPACE || input == CHAR_DELETE)

int AccelDbg_isDispLogOn(void);

static void AccelDbg_Task(UArg arg0, UArg arg1);
static UART_Handle DisplayUart_getUartHandle(Display_Handle hDisplay);

static int AccelDbg_readCommand(dbg_cmd_tble_st *retCmd);
static int AccelDbg_parseCommand(char *cmdString, int len, dbg_cmd_tble_st *retCmd);
static void AccelDbg_processCommand(dbg_cmd_tble_st cmd);
//static void AccelDbg_setShowSensorLog(dbg_cmd_tble_st Cmd);
static void AccelDbg_setSensingPeriod(dbg_cmd_tble_st Cmd);

#define DBG_CMD_VALID	(-1)
int updated_value = DBG_CMD_VALID;
void AccelDbg_setValueDone(void)
{
	updated_value = DBG_CMD_VALID;
}

int AccelDbg_getValue(void)
{
	return updated_value;
}

static int AccelDbg_setValue(uint8_t value)
{
	int result=0;

	if(updated_value == DBG_CMD_VALID)
	{
		updated_value = (int)value;
		result = 1;
	}

	return result;
}

void AccelDbg_showPrompt(void)
{
	UART_write(DbgUartHandle, echoPrompt, sizeof(echoPrompt));
}

static void AccelDbg_sendAppMsg(uint8_t message, uint8_t value)
{
	uint8_t sensor_msg=0;
	if(AccelDbg_setValue(value) == 0)
	{
		Display_print0(dispHandle, 3, 0, "send value for update not ready ");
		return;
	}

	sensor_msg |= message;
	AccelSensor_setDebugEvent(sensor_msg);
}

static void AccelDbg_setDataRate(dbg_cmd_tble_st Cmd)
{
	uint8_t value;
	//uint8_t status=0;

	if(Cmd.cnt == 0)
	{
		value = (uint8_t)SNV_getItemValue(SNV_ACCEL_DATARATE_ITEM);
		Display_print1(dispHandle, 3, 0, " Data Rate: %d", (int)value);
	}
	else
	{
		value = Cmd.arg_val;
		if(value > 9) //type error
		{
			Display_print1(dispHandle, 3, 0, " set Data Rate invalid parameter: %d", (int)value);
		}
		else
		{
			AccelDbg_sendAppMsg(UPDATE_ODR_SNV, value);
			//status = SNV_setItemValue(SNV_ACCEL_DATARATE_ITEM, (uint8_t)value);
			//Display_print1(dispHandle, 3, 0, " set Data Rate: %d", value);
		}
	}
}

static void AccelDbg_setFullScale(dbg_cmd_tble_st Cmd)
{
	uint8_t value;
	//uint8_t status=0;

	if(Cmd.cnt == 0)
	{
		value = (uint8_t)SNV_getItemValue(SNV_ACCEL_SCALE_ITEM);
		Display_print1(dispHandle, 3, 0, " Scale: %d", (int)value);
	}
	else
	{
		value = Cmd.arg_val;
		if(value > 3) //type error
		{
			Display_print1(dispHandle, 3, 0, " set Scale invalid parameter: %d", (int)value);
		}
		else
		{
			AccelDbg_sendAppMsg(UPDATE_SCALE_SNV, value);
			//status = SNV_setItemValue(SNV_ACCEL_SCALE_ITEM, (uint8_t)value);
			//Display_print1(dispHandle, 3, 0, " set Data Rate: %d", value);
		}
	}
}

int AccelDbg_DispLogOn=0;
int AccelDbg_DispLogCount=0;
int AccelDbg_isDispLogOn(void)
{
	return AccelDbg_DispLogOn;
}

int AccelDbg_LastLogCount(void)
{
	return (AccelDbg_DispLogCount == 1);
}

int AccelDbg_isSensorLogOn(void)
{
	return (AccelDbg_DispLogOn || AccelDbg_DispLogCount);
}

void AccelDbg_updateLogCount(void)
{
	if(AccelDbg_DispLogCount > 0) AccelDbg_DispLogCount--;
}

int AccelDbg_showDebugCmd(void)
{
	//return (AccelDbg_isDispLogOn() == 0);
	return !AccelDbg_isSensorLogOn();
}

//static void AccelDbg_setShowSensorLog(dbg_cmd_tble_st Cmd)
//{
//	uint8_t value;
//
//	if(Cmd.cnt == 0)
//	{
//		Display_print1(dispHandle, 3, 0, " Show Sensor data: %d", (int)AccelDbg_DispLogOn);
//	}
//	else
//	{
//		value = Cmd.arg_val;
//		if(value > 3) //type error
//		{
//			Display_print1(dispHandle, 3, 0, " Show Sensor log invalid parameter: %d", (int)value);
//		}
//		else
//		{
//			AccelDbg_DispLogOn = value;
//			Display_print1(dispHandle, 3, 0, " Show Sensor log: %d", (int)AccelDbg_DispLogOn);
//		}
//	}
//}

static void AccelDbg_setSensingPeriod(dbg_cmd_tble_st Cmd)
{
	int period, value;

	if(Cmd.cnt == 0)
	{
		period = AccelSensor_getSensingPeriod();
		Display_print1(dispHandle, 3, 0, " Show Sensing Period: %d", (int)period);
	}
	else
	{
		value = Cmd.arg_val;
		if(value >= 100 && value <= 5000) // range of period
		{
			period = AccelSensor_setSensingPeriod(value);
			Display_print1(dispHandle, 3, 0, " set Sensing Period: %d", period);
		}
		else
			Display_print1(dispHandle, 3, 0, " Period should be 100 <= period <= 5000: but %d", value);
	}
}

#if 0
static void AccelDbg_setSensingCount(dbg_cmd_tble_st Cmd)
{
	int count, value;

	if(Cmd.cnt == 0)
	{
		count = AccelSensor_getSensingCount();
		Display_print1(dispHandle, 3, 0, " Show Sample count: %d", (int)count);
	}
	else
	{
		value = Cmd.arg_val;
		if(value >= MIN_SAMPLE_COUNT && value <= MAX_SAMPLE_COUNT) // range of count
		{
			count = AccelSensor_setSensingCount(value);
			Display_print1(dispHandle, 3, 0, " set Sample count: %d", count);
		}
		else
			Display_print1(dispHandle, 3, 0, " Sample count should be 10 <= count <= 150: but %d", value);
	}
}
#endif

static void AccelDbg_readSensorRegister(dbg_cmd_tble_st Cmd)
{
	int reg;
	uint8_t value;

	if(Cmd.cnt == 0)
	{
		Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
	else
	{
		reg = Cmd.arg_val;
		if(reg == 0 || reg == 1) // 0: ctrl1, 1:ctrl4
		{
			value = Acc_readRegVal(reg);
			Display_print2(dispHandle, 3, 0, " register %d value: 0x%x ", reg, value);
		}
		else
			Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}

static void AccelDbg_readSensorValue(dbg_cmd_tble_st Cmd)
{
	int result;
	int8_t x_val, y_val, z_val;

	if(Cmd.cnt == 0)
	{
		result = Acc_readAccVal(&x_val, &y_val, &z_val);
		if(result)
			Display_print3(dispHandle, 3, 0, " Read Sensor value: x=%d y=%d z=%d", x_val, y_val, z_val);
		else
			Display_print1(dispHandle, 3, 0, "Read error=%d", result);
	}
	else
	{
		Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}


static void AccelDbg_initSNVValue(dbg_cmd_tble_st Cmd)
{
	uint8 sensor_msg = 0;

	if(Cmd.cnt == 0)
	{
		sensor_msg |= INIT_ALL_SNV;
		AccelSensor_setDebugEvent(sensor_msg);
	}
	else
	{
		Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}

static void AccelDbg_readNVValue(dbg_cmd_tble_st Cmd)
{
	uint8 sensor_msg = 0;

	if(Cmd.cnt == 0)
	{
		sensor_msg |= READ_ALL_SNV;
		AccelSensor_setDebugEvent(sensor_msg);
	}
	else
	{
		Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}
/*
static void AccelDbg_autoscale(dbg_cmd_tble_st Cmd)
{
	int on_off;

	if(Cmd.cnt == 0)
	{
		on_off = AccelScale_isOn();
		Display_print1(dispHandle, 3, 0, " Autoscale setting is %d", (int)on_off);
	}
	else
	{
		on_off = Cmd.arg_val;
		if(on_off == 0 || on_off == 1) // 0: ctrl1, 1:ctrl4
		{
			AccelScale_setAutoScale(on_off);
			Display_print1(dispHandle, 3, 0, " autoscale set %d ", on_off);
		}
		else
			Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}
*/
static void AccelDbg_showVersion(dbg_cmd_tble_st Cmd)
{
	if(Cmd.cnt == 0)
	{
		Display_print2(dispHandle, 3, 0, " FW version: %d.%d", FW_VER_MAJ, FW_VER_MIN);
	}
	else
	{
		Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}


static void AccelDbg_initSensorData(dbg_cmd_tble_st Cmd)
{
	uint8 sensor_msg = 0;

	if(Cmd.cnt == 0)
	{
		//Display_print0(dispHandle, 3, 0, " clear sensor data");
		sensor_msg |= INIT_SENS_DATA;
		AccelSensor_setDebugEvent(sensor_msg);
	}
	else
	{
		Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}

static void AccelDbg_showSensorLogCount(dbg_cmd_tble_st Cmd)
{
	int count;
	//const char *scale[4] = {"2G", "4G", "8G", "16G"};

	if(Cmd.cnt == 0)
	{
		Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
	else
	{
		count = Cmd.arg_val;
		if(count > 0 && count <= 50)
		{
			AccelDbg_DispLogCount = count;
			Display_print1(dispHandle, 3, 0, " SCALE:\t %d", (int)SNV_getItemValue(SNV_ACCEL_SCALE_ITEM));
			Display_print1(dispHandle, 3, 0, " PERIOD: %d", (int)AccelSensor_getSensingPeriod());
			Display_print1(dispHandle, 3, 0, " N:\t %d", (int)count);
			Display_print1(dispHandle, 3, 0, " TEMPERATURE:\t %d", (int)batt_readInternalTemperature());
		}
		else
			Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}

static void AccelDbg_showSensorLogTime(dbg_cmd_tble_st Cmd)
{
	int t_period, period=(int)AccelSensor_getSensingPeriod();

	if(Cmd.cnt == 0)
	{
		Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
	else
	{
		t_period = Cmd.arg_val;
		if(t_period >= 100 && t_period <= 10000)
		{
			AccelDbg_DispLogCount = (int)(t_period/period);
			Display_print1(dispHandle, 3, 0, " SCALE:\t %d", (int)SNV_getItemValue(SNV_ACCEL_SCALE_ITEM));
			Display_print1(dispHandle, 3, 0, " PERIOD: %d", (int)AccelSensor_getSensingPeriod());
			Display_print2(dispHandle, 3, 0, " T:\t %d\tN: %d", (int)t_period, AccelDbg_DispLogCount);
		}
		else
			Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}

static void AccelDbg_getZeroG(dbg_cmd_tble_st Cmd)
{
	uint8 sensor_msg = 0;
	int zero_g[3], value;

	if(Cmd.cnt == 0)
	{
		sensor_msg |= COMPENSATE_ZERO_G;
		AccelSensor_setDebugEvent(sensor_msg);
	}
	else
	{
		value = Cmd.arg_val;
		if(value == 0)
		{
			AccelSensor_getZeroG(zero_g);
			Display_print3(dispHandle, 2, 0, "cur value x=%d y=%d z=%d", zero_g[0], zero_g[1], zero_g[2]);
		}
		else if(value == 1)
		{
			AccelSensor_getZeroG_SNV(zero_g);
			Display_print3(dispHandle, 2, 0, "SNV value x=%d y=%d z=%d", zero_g[0], zero_g[1], zero_g[2]);
		}
		else
			Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}

static void AccelDbg_setInterrupt(dbg_cmd_tble_st Cmd)
{
	int value;

	if(Cmd.cnt == 0)
	{
		value = AccelSensor_isSensorInterruptEnabled();
		Display_print1(dispHandle, 3, 0, "sensor interrupt value = %d", value);
	}
	else
	{
		value = Cmd.arg_val;
		if(value == 0 || value == 1) // 0: disable, 1:enable
		{
			AccelSensor_setSensorInterrupt(value);
			Display_print1(dispHandle, 3, 0, " sensor interrupt enable=%d", value);
		}
		else
			Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}

static void AccelDbg_updateThreshold(uint8_t value)
{
	motor_sense_threshold = value;
}

static void AccelDbg_setThreshold(dbg_cmd_tble_st Cmd)
{
	int value;

	if(Cmd.cnt == 0)
	{
		value = motor_sense_threshold;
		Display_print1(dispHandle, 3, 0, "motor sensing threshold value = %d", value);
	}
	else
	{
		value = Cmd.arg_val;
		if(value >= 10 && value <= 200)
		{
			AccelDbg_updateThreshold(value);
			Display_print1(dispHandle, 3, 0, " motor sensing threshold changed %d", value);
		}
		else
			Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}
#ifdef DETECT_MOTOR_ON
extern int AccelSensor_isMotorWorking(void);
extern void AccelSensor_setSensorMode(int mode);
static void AccelDbg_setSensorMode(dbg_cmd_tble_st Cmd)
{
	int value;

	if(Cmd.cnt == 0)
	{
		if(AccelSensor_isMotorWorking())
			value = 1;
		else
			value = 0;
		Display_print1(dispHandle, 3, 0, "sensor mode = %d", value);
	}
	else
	{
		value = Cmd.arg_val;
		if(value == 0 || value == 1) // 0: INT mode, 1: normal mode
		{
			AccelSensor_setSensorMode(value);
			Display_print1(dispHandle, 3, 0, " set sensor mode=%d", value);
		}
		else
			Display_print0(dispHandle, 3, 0, Cmd.usage);
	}
}
#endif

static int AccelDbg_readCommand(dbg_cmd_tble_st *retCmd)
{
	int pos, state;
    char input;
    char cmdString[DBG_COMMAND_LENGTH];

    memset(cmdString, 0, DBG_COMMAND_LENGTH);
    pos=0;
	while(1) {
		state = UART_read(DbgUartHandle, &input, 1);
		if(state < 0) Display_print1(dispHandle, 3, 0, "UART_read error %d", state);

		if( isValidInput(input, pos) ) {
			if( isLowerCapital(input) ) input -= 0x20; // convert upper capital

			cmdString[pos] = input;
			pos++;

			if(AccelDbg_showDebugCmd())
				UART_write(DbgUartHandle, &input, 1);
		}
		else if( isEndOfInput(input, pos) )	{
			if(AccelDbg_showDebugCmd())
				UART_write(DbgUartHandle, "\r\n", 2);
			return AccelDbg_parseCommand(cmdString, pos, retCmd);
		}
		else if( isBackSpace(input, pos) ) {
			pos--;
			input = CHAR_BACK_SPACE;
			if(AccelDbg_showDebugCmd())
				UART_write(DbgUartHandle, &input, 1);
		}
		else {
			break;
		}
	}

	return DBG_ERROR_INVALID_CMD;
}

static int AccelDbg_parseCommand(char *cmdString, int len, dbg_cmd_tble_st *retCmd)
{
	int i, err_status=DBG_ERROR_SUCCESS;
	char parameters[3][20]; //3x10 array, command + param1 + param2
	int param_cnt=0, st_pos=0;

	memset(parameters[0], 0, 20);
	memset(parameters[1], 0, 20);
	memset(parameters[2], 0, 20);

	for(i=0; i<len+1; i++) {
		if(cmdString[i] == ' ' || i == len || param_cnt == 2) {
			memcpy(&parameters[param_cnt][0], &cmdString[st_pos], i-st_pos);
			param_cnt++;
			st_pos = i+1;
		}
	}

	for(i=0; i<DBG_CMD_MAX; i++)
	{
		if( strcmp(&parameters[0][0], dbgCmdTable[i].name) == 0) break;
	}

	if(i>=DBG_CMD_MAX) {
		err_status = DBG_ERROR_INVALID_CMD;
		if(len <= 1)
			retCmd->code = DBG_CMD_EMPTY;
		else
			retCmd->code = DBG_CMD_MAX;
	}
	else {
		retCmd->name = dbgCmdTable[i].name;
		retCmd->cnt = param_cnt-1;
		retCmd->code = dbgCmdTable[i].code;
		retCmd->usage = dbgCmdTable[i].usage;
		retCmd->reserv = 0;
		if(retCmd->cnt > 0)
			retCmd->arg_val = (int)atoi(&parameters[1][0]);
		else
			retCmd->arg_val = 0;

		err_status = DBG_ERROR_SUCCESS;
	}

#if 0 // for debug only
	Display_print0(dispHandle, 3, 0, parameters[0]);
	if(parameters[1][0] != 0) Display_print0(dispHandle, 3, 0, parameters[1]);
#endif

	return err_status;
}

static void AccelDbg_processCommand(dbg_cmd_tble_st Cmd)
{
	//static int first=0;
	uint8_t code;

	switch(Cmd.code)
	{
	case DBG_CMD_SHOW_HELP :
		code = DBG_CMD_SHOW_HELP;
		while(code < DBG_CMD_MAX)
		{
			Display_print0(dispHandle, 3, 0, dbgCmdTable[code].usage);
			code++;
		}
		break;

	case DBG_CMD_DATA_RATE :
		AccelDbg_setDataRate(Cmd);
		break;

	case DBG_CMD_SET_SCALE :
		AccelDbg_setFullScale(Cmd);
		break;

//	case DBG_CMD_SET_EXT_PWR :
//		AccelDbg_setExternalPower(Cmd);
//		break;

//	case DBG_CMD_SET_DISP_ON :
//		AccelDbg_setShowSensorLog(Cmd);
//		break;

	case DBG_CMD_SET_PERIOD :
		AccelDbg_setSensingPeriod(Cmd);
		break;

//	case DBG_CMD_SMAPLE_CNT :
//		AccelDbg_setSensingCount(Cmd);
//		break;

	case DBG_CMD_READ_REG :
		AccelDbg_readSensorRegister(Cmd);
		break;

	case DBG_CMD_READ_VAL :
		AccelDbg_readSensorValue(Cmd);
		break;

	case DBG_CMD_INIT_SNV :
		AccelDbg_initSNVValue(Cmd);
		break;

	case DBG_CMD_READ_SNV :
		AccelDbg_readNVValue(Cmd);
		break;

//	case DBG_CMD_AUTOSCALE :
//		AccelDbg_autoscale(Cmd);
//		break;

	case DBG_CMD_VERSION :
		AccelDbg_showVersion(Cmd);
		break;

	case DBG_CMD_INIT_DATA :
		AccelDbg_initSensorData(Cmd);
		break;

	case DBG_CMD_DISP_CNT :
		AccelDbg_showSensorLogCount(Cmd);
		break;

	case DBG_CMD_DISP_TIME :
		AccelDbg_showSensorLogTime(Cmd);
		break;

	case DBG_CMD_GET_ZERO_G :
		AccelDbg_getZeroG(Cmd);
		break;

	case DBG_CMD_SET_INT :
		AccelDbg_setInterrupt(Cmd);
		break;
	case DBG_CMD_MOTOR_DETECT :
		AccelDbg_setThreshold(Cmd);
		break;

	case DBG_CMD_EMPTY : // in case of press return key -> just show
		UART_write(DbgUartHandle, "\r\n", sizeof("\r\n"));
		break;


	case DBG_CMD_MAX:
	default :
		if(AccelDbg_showDebugCmd())
			Display_print0(dispHandle, 3, 0, "invalid command !");
		break;

	}
	if(AccelDbg_showDebugCmd())
		AccelDbg_showPrompt();
}

void AccelDbg_createTask(void)
{
    Task_Params taskParams;

	/* Construct BIOS objects */
	Task_Params_init(&taskParams);
	taskParams.stackSize = DBG_TASK_STACK_SIZE;
	taskParams.stack = &Dbg_taskStack;
	taskParams.priority = DBG_TASK_PRIORITY;

	Task_construct(&Dbg_taskStruct, (Task_FuncPtr)AccelDbg_Task, &taskParams, NULL);
  //Task_construct(&sbpTask, echoFxn, &taskParams, NULL);
}

/*
 *  ======== echoFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
static void AccelDbg_Task(UArg arg0, UArg arg1)
{
	dbg_cmd_tble_st retCmd;
	//int err_state;
#if 0
    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    //uartParams.readMode = UART_MODE_CALLBACK;
    uart = UART_open(Board_UART0, &uartParams);
#else

    dispHandle = Display_open(Display_Type_UART, NULL);
    if (dispHandle != NULL) {
    	DbgUartHandle = DisplayUart_getUartHandle(dispHandle);
    }
#endif

    Display_print0(dispHandle, 0, 0, "AccelUart Task");
    AccelDbg_showPrompt();

    //Power_sleep(PowerCC26XX_STANDBY);

    /* Loop forever echoing */
    while (1) {

    	memset(&retCmd, 0, sizeof(dbg_cmd_tble_st));
    	AccelDbg_readCommand(&retCmd);
    	AccelDbg_processCommand(retCmd);

    	//Display_print1(dispHandle, 3, 0, "input error %d", err_state);
        //Board_Led_control(board_led_type_LED1, board_led_state_ON);
    }
}

static UART_Handle DisplayUart_getUartHandle(Display_Handle hDisplay)
{
    DisplayUart_Object  *object  = (DisplayUart_Object  *)hDisplay->object;

    return object->hUart;
}

#endif
