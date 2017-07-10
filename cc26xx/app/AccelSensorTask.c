/******************************************************************************

 @file  AccelSensorTask.c

 @brief This file contains the Simple BLE Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2013-2016, Texas Instruments Incorporated
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
#include <string.h>
#include <math.h>
#include <time.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Seconds.h>  // to use Second_get()
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

#include "hci_tl.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
// #include "simple_gatt_profile.h"  //hrjung remove profile
#include "accelerometer.h" // accelerometer profile

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC
   
#include <ti/mw/display/Display.h>
#include "AccelSensorTask.h"
#include "AccelUartTask.h"
#include "board_key.h"
#include "board_led.h" //hrjung add
#include "board_gpio.h"
#include "board.h"

#include "Accel_drv.h"
#if defined(ACCEL_BMA250_USED)
#include "bma250.h"
#include "drv_bma250.h"
#elif defined(ACCEL_LIS2DE12_USED)
#include "drv_lis2de12.h"
#else
#error "accelerometer not defined"
#endif

//#include "rom_crypto.h" //for AES encryption
#include "snv_drv.h"

//#include "Accel_autoscale.h" // for autoscale

#ifdef USE_ACCEL_DEBUG_TASK
extern int AccelDbg_isDispLogOn(void); //AccelUartTask.c
extern int AccelDbg_isSensorLogOn(void);
extern void AccelDbg_updateLogCount(void);
extern int AccelDbg_LastLogCount(void);
extern int AccelDbg_getValue(void);
extern void AccelDbg_setValueDone(void);
extern void AccelDbg_showPrompt(void);
#endif
//extern uint8_t Lis2de12_readId(void);
//extern int Lis2de12_readVal(uint8_t *vals);
//extern int Acc_isAccInitialized(void);

extern void test_ACC_output(uint8_t value);

//for SNV processing
extern int SNV_forceInit(void);
extern uint8 SNV_readValue(SNV_data_st *val);

extern int fft_transform(float *real, float *imag);
extern void fft_getMagnitude(uint16_t *mag, float *real, float *img);
/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          800 //(160*5)

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800
#else //!FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often (in ms) to read the accelerometer
#define ACCEL_READ_PERIOD             		2000

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// For simulated measurements.
//#define FLAGS_IDX_MAX                            5

//=================================================================

// Task configuration
#define SBP_TASK_PRIORITY                     2

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   1024
#endif


// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT             	0x0001
#define SBP_ACCEL_CHANGE_SCALE_EVT      	0x0002
#define SBP_CONN_EVT_END_EVT             	0x0004
#define SBP_SNV_UPDATE_EVT               	0x0008
#define SBP_ACCEL_READ_EVT               	0x0010
#define SBP_ACCEL_CHANGE_EVT              	0x0020   // configure device param via BLE
#define SBP_ADV_CB_EVT						0x0040   // adv event from stack
#define SBP_SENSOR_INT_EVT					0x0080   // sensor data ready event

//#define SBP_POWER_MONITOR_EVT				0x0040
//#define SBP_SENS_DATA_INIT_EVT				0x0080

// sub event for SBP_SENSOR_INT_EVT
#define SBP_SENSOR_INT_MSG					0x01
#define SBP_SENSOR_READ_END_MSG				0x10

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;


typedef struct
{
	int16 	x[ACCEL_SENSOR_POINT_COUNT];
	int16 	y[ACCEL_SENSOR_POINT_COUNT];
	int16 	z[ACCEL_SENSOR_POINT_COUNT];
} sensor_calc_st;

static int	sens_index=0;

#ifdef ACCELSENSOR_AUTOSCALE
static int	sens_cycle_cnt=0;
static int	sens_cycle_done=0;
#endif

static sensor_calc_st sens_ave;
//static sensor_calc_st sens_dev;
//static sensor_calc_st sens_max;
//static sensor_calc_st sens_min;
static int16_t grav_x=0, grav_y=0, grav_z=0;
/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct accelReadClock; //hrjung add

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Profile state and parameters
static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x41,   // 'A'
  0x63,   // 'c'
  0x63,   // 'c'
  0x65,   // 'e'
  0x6c,   // 'l'
  0x65,   // 'e'
  0x72,   // 'r'
  0x6f,   // 'o'
  0x6d,   // 'm'
  0x65,   // 'e'
  0x74,   // 't'
  0x65,   // 'e'
  0x72,   // 'r'
  0x53,   // 'S'
  0x65,   // 'e'
  0x6e,   // 'n'
  0x73,   // 's'
  0x6f,   // 'o'
  0x72,   // 'r'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
#if !defined(FEATURE_OAD) || defined(FEATURE_OAD_ONCHIP)
  0x03,   // length of this data
#else //OAD for external flash
  0x05,  // lenght of this data
#endif //FEATURE_OAD
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
#endif //FEATURE_OAD
#ifndef FEATURE_OAD_ONCHIP
  LO_UINT16(ACCEL_SERVICE_UUID),
  HI_UINT16(ACCEL_SERVICE_UUID)
#endif //FEATURE_OAD_ONCHIP
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Accelerometer Sensor";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Accelerometer Profile Parameters
//hrjung remove static uint8_t accelEnabler = FALSE;

#define SENSOR_BUF_LENGTH	MAX_SAMPLE_COUNT
typedef struct
{
	int16		x[SENSOR_BUF_LENGTH];
	int16		y[SENSOR_BUF_LENGTH];
	int16		z[SENSOR_BUF_LENGTH];
} sensor_data_st;

static sensor_data_st	sensor;
//static int ptimer_interval = SBP_CALC_EVT_PERIOD;
static int sensing_period = ACCEL_READ_PERIOD;
const static int sensing_count = FFT_SAMPLE_MAX;
static int sensing_index=0;

//debug event
#ifdef USE_ACCEL_DEBUG_TASK
static int event_from_debug=0;
static uint8_t eventCmd=0;
#endif

Power_NotifyObj rFSwitchPowerNotifyObj;

static uint16_t mag_x[FFT_SAMPLE_MAX/2], mag_y[FFT_SAMPLE_MAX/2], mag_z[FFT_SAMPLE_MAX/2];
static float real_in[FFT_SAMPLE_MAX], img_in[FFT_SAMPLE_MAX];


static uint32_t base_time = 1483972450; // 2017-01-09 14:34:10
extern uint_t PIN_NumPins;//hrjung test
/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void AccelSensor_init( void );
static void AccelSensor_taskFxn(UArg a0, UArg a1);

static uint8_t AccelSensor_processStackMsg(ICall_Hdr *pMsg);
static uint8_t AccelSensor_processGATTMsg(gattMsgEvent_t *pMsg);
static void AccelSensor_processAppMsg(sbpEvt_t *pMsg);
static void AccelSensor_processStateChangeEvt(gaprole_States_t newState);
static void AccelSensor_clockHandler(UArg arg);

static void AccelSensor_sendAttRsp(void);
static void AccelSensor_freeAttRsp(uint8_t status);

static void AccelSensor_stateChangeCB(gaprole_States_t newState);
static void AccelSensor_enqueueMsg(uint8_t event, uint8_t state);

#ifdef FEATURE_OAD
void AccelSensor_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

#ifdef USE_ACCEL_DEBUG_TASK
static int AccelSensor_isDebugEventOccurred(void);
static void AccelSensor_processDebugEvent(uint8_t shift, uint8_t cmd);
#endif

//hrjung add for accelerometer
static void AccelSensor_start(void);
static void AccelSensor_stop(void);
//static int AccelSensor_process(int condition);

static void AccelSensor_accelBleUpdateCB(uint8_t cmd, uint8_t event);
static void AccelSensor_processBLEValueChangeEvt(uint8 event);

static void AccelSensor_initSensorData(void);
static void AccelSensor_initCalcResult(void);

//static void AccelSensor_processAccelReadEvt(void);
static void AccelSensor_accelRead(int index);
static void AccelSensor_putSensorData(int idx, int8_t x, int8_t y, int8_t z);
static void AccelSensor_putCalcResult(int *ave);
//static void AccelSensor_putSensorCalc(int16 *ave, int16 *dev, int16 *max, int16 *min);
//static void AccelSensor_calcResult(void);
//static void AccelSensor_printResultTask(void);

static void AccelSensor_initSensorInt(void);
static void AccelSensor_processSensorInterruptMsg(uint8_t event);
static void AccelSensor_processSensorInterrupt(void);
static void AccelSensor_sendSensorReadEndEvent(void);
static void AccelSensor_postProcessSensorData(void);
void AccelSensor_setSensorInterrupt(int enabled);

static uint8_t pwrSwitchNotifyCb(uint8_t eventType, uint32_t *eventArg, uint32_t *clientArg);
static void AccelSensor_readBatteryLevel(uint32_t count);
static void AccelSensor_readTemperature(uint32_t count);
static void AccelSensor_RunLED(uint32_t count, board_led_state *status);



static void AccelSensor_compensateZeroG(void);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t AccelSensor_gapRoleCBs =
{
  AccelSensor_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t AccelSensor_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};


#ifdef FEATURE_OAD
static oadTargetCBs_t AccelSensor_oadCBs =
{
  AccelSensor_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

// Accelerometer Profile Callbacks
static accelCBs_t AccelSensor_accelCBs =
{
	AccelSensor_accelBleUpdateCB  // Called when Enabler attribute changes
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
#ifdef USE_ACCEL_DEBUG_TASK
int AccelSensor_getSensingPeriod(void)
{
	return sensing_period/1000;
}

int AccelSensor_setSensingPeriod(int period)
{
	sensing_period = period*1000;
	Util_rescheduleClock(&accelReadClock, sensing_period);

	return sensing_period;
}
#endif

#if 0
int AccelSensor_getSensingCount(void)
{
	return sensing_count;
}

int AccelSensor_setSensingCount(int count)
{
	sensing_count = count;

	return sensing_count;
}
#endif
/*********************************************************************
 * @fn      AccelSensor_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void AccelSensor_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, AccelSensor_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      AccelSensor_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void AccelSensor_init(void)
{
  uint8_t status=0;

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);
    
#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
//hrjung, not use collect data via timer interrupt
  Util_constructClock(&accelReadClock, AccelSensor_clockHandler,
		  	  	  	  sensing_period, sensing_period, false, SBP_ACCEL_READ_EVT);


#if !defined(CC2650CUST_5XD)
  Board_initKeys(AccelSensor_keyPressHandler);
#endif
  Board_Led_initialize();
  Board_initGenericGpio();

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);

    //TODO : test only : use advertisement channel 37 only
    uint16_t desiredAdvChannel = GAP_ADVCHAN_37;
    GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint16_t),
                         &desiredAdvChannel);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

  Accel_AddService(GATT_ALL_SERVICES);        // Accelerometer Profile

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&AccelSensor_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE

  // Start the Accelerometer Profile
  VOID Accel_RegisterAppCBs(&AccelSensor_accelCBs);

  // Start the Device
  VOID GAPRole_StartDevice(&AccelSensor_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&AccelSensor_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  HCI_LE_ReadMaxDataLenCmd();

  //hrjung add
#ifdef BLE_ADV_EVENT_PROC_
  HCI_EXT_AdvEventNoticeCmd(selfEntity, SBP_ADV_CB_EVT);
#endif

#if defined FEATURE_OAD
#if defined (HAL_IMAGE_A)
  Display_print0(dispHandle, 0, 0, "BLE Peripheral A");
#else
  Display_print0(dispHandle, 0, 0, "BLE Peripheral B");
#endif // HAL_IMAGE_A
#else
  Display_print0(dispHandle, 0, 0, "AccelSensor Task");
#endif // FEATURE_OAD

  //hrjung for power test
  Board_Led_control(board_led_type_LED1, board_led_state_ON);

  Seconds_set(base_time);

  // initialize SNV
  status = SNV_init();
  if(status != SNV_SUCCESS)
  {
	  Display_print1(dispHandle, 3, 0, "SNV_init error %d", status);
	  AccelSensor_error();
  }

  //initialize Sensor
  AccelSensor_initSensorData();
  AccelSensor_initCalcResult();

  if(Acc_init() == 0) //open sensor
  {
	  Display_print1(dispHandle, 3, 0, "Acc open error, NUimPin=%d ", PIN_NumPins);
	  AccelSensor_error();
  }

  SNV_applyInitalValue(); // SNV -> config data

  if(Acc_identify() == 0)
  {
	  Display_print0(dispHandle, 3, 0, "Acc identify error ");
	  AccelSensor_error();
  }

  //Acc_setPowerMode(0); // goes to power down

  Display_print1(dispHandle, 3, 0, "Initialize OK tick=%d", Clock_tickPeriod);

  AccelSensor_start(); //test

#ifdef ENABLE_STANDBY
  /* Power Notify */
  Power_registerNotify(&rFSwitchPowerNotifyObj,
                   PowerCC26XX_ENTERING_STANDBY | PowerCC26XX_AWAKE_STANDBY,
                   (Power_NotifyFxn) pwrSwitchNotifyCb, NULL);
#endif

}
#ifdef ENABLE_STANDBY
static uint8_t pwrSwitchNotifyCb(uint8_t eventType, uint32_t *eventArg, uint32_t *clientArg)
{
	if(eventType == PowerCC26XX_ENTERING_STANDBY)
	{
		Board_Led_control(board_led_type_LED1, board_led_state_OFF);
	}
	else if(eventType == PowerCC26XX_AWAKE_STANDBY)
	{
		Board_Led_control(board_led_type_LED1, board_led_state_ON);
	}

	// Notification handled successfully
	return Power_NOTIFYDONE;
}
#endif
/*********************************************************************
 * @fn      AccelSensor_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void AccelSensor_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  AccelSensor_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              AccelSensor_sendAttRsp();
            }

#ifdef BLE_ADV_EVENT_PROC_
            // advertisement event recieved
            if (pEvt->event_flag & SBP_ADV_CB_EVT)
            {
            	static uint32_t adv_count=1;
#ifdef DETECT_MOTOR_ON
            	if(AccelSensor_isMotorWorking())
#endif
				{
					if(AccelSensor_isReadPeriod(adv_count))
					{
						AccelSensor_sendSensorReadStartEvent();
						//Display_print1(dispHandle, 3, 0, "read sensor adv=%d", adv_count);
					}
					//AccelSensor_RunLED(adv_count, &LED_status);
				}

				AccelSensor_readBatteryLevel(adv_count);
				AccelSensor_readTemperature(adv_count);

				adv_count++;
				if(adv_count > (10*24*3600*2)) adv_count=0; // 10 day for 0.5 sec ADV
            }
#endif
          }
          else
          {
            // Process inter-task message
            safeToDealloc = AccelSensor_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          AccelSensor_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }



//    if (events & SBP_ACCEL_CHANGE_EVT)
//    {
//      events &= ~SBP_ACCEL_CHANGE_EVT;
//
//      AccelSensor_processBLEValueChangeEvt();
//    }

    if( events & SBP_SENSOR_INT_EVT)
    {
    	events &= ~SBP_SENSOR_INT_EVT;
    	//test_ACC_output(0);
    	AccelSensor_processSensorInterrupt();
    }

    if (events & SBP_ACCEL_READ_EVT)
    {
      events &= ~SBP_ACCEL_READ_EVT;

      ////Display_print0(dispHandle, 3, 0, "READ_EVT");
      //AccelSensor_enqueueMsg(SBP_ACCEL_READ_EVT, 0);
      //AccelSensor_processAccelReadEvt();
      //AccelSensor_printResultTask();

      AccelSensor_initSensorInt();
    }

#ifdef USE_ACCEL_DEBUG_TASK
    if(AccelSensor_isDebugEventOccurred())
    {
    	AccelSensor_processDebugEvent(0, eventCmd);
    	event_from_debug=0; //clear flag
    	eventCmd = 0;
    	Display_print0(dispHandle, 3, 0, "Debug event processed");
    }
#endif

#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
      oadTargetWrite_t *oadWriteEvt = Queue_dequeue(hOadQ);

      // Identify new image.
      if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }
      // Write a next block request.
      else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }

      // Free buffer.
      ICall_free(oadWriteEvt);
    }
#endif //FEATURE_OAD
  }
}

/*********************************************************************
 * @fn      AccelSensor_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t AccelSensor_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = AccelSensor_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;

          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      AccelSensor_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t AccelSensor_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      AccelSensor_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_print1(dispHandle, 5, 0, "MTU Size: $d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      AccelSensor_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void AccelSensor_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      AccelSensor_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, 5, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}

/*********************************************************************
 * @fn      AccelSensor_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void AccelSensor_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, 5, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, 5, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      AccelSensor_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void AccelSensor_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
    	AccelSensor_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
    	break;
#ifdef ACCELSENSOR_AUTOSCALE
    case SBP_ACCEL_CHANGE_SCALE_EVT:
    	AccelScale_changeScale(pMsg->hdr.state);
    	break;
#endif

    case SBP_ACCEL_CHANGE_EVT:
    	AccelSensor_processBLEValueChangeEvt(pMsg->hdr.state);
    	break;

    case SBP_SENSOR_INT_EVT :
    	AccelSensor_processSensorInterruptMsg(pMsg->hdr.state);
    	break;

//    case SBP_SENS_DATA_INIT_EVT :
//    	AccelSensor_initSensorData();
//    	AccelSensor_initCalcResult();
//    	Display_print0(dispHandle, 2, 0, "Whole sensor data initialized");
//    	break;

//    case SBP_SET_SEND_NOTI_EVT:
//    	Accel_enableNotifyCfg();
//    	break;

#ifdef USE_ACCEL_DEBUG_TASK
    case SBP_SNV_UPDATE_EVT:
    	AccelSensor_processDebugEvent(0, pMsg->hdr.state);
    	break;
#endif

#if 0 //test only
    case SBP_ACCEL_READ_EVT:
    {
    	uint8_t resp[10];

    	memset(resp, 0, 10);
    	Board_readBma250(2, 6, resp);
    	Display_print3(dispHandle, 2, 0, "%d, %d, %d", (uint16_t)resp[1], (uint16_t)resp[3], (uint16_t)resp[5]);
    	//Cutil_printAccelValue(resp[1], resp[3], resp[5]);
    }
    break;
#endif

    default:
      // Do nothing.
      break;
  }
}
#ifdef USE_ACCEL_DEBUG_TASK
int AccelSensor_isDebugEventOccurred(void)
{
	return event_from_debug;
}

void AccelSensor_setDebugEvent(uint8_t event)
{
	event_from_debug=1;
	eventCmd |= event;
}

static void AccelSensor_processDebugEvent(uint8_t shift, uint8_t cmd)
{
	int status=0, value;

	AccelSensor_stop();
	if(cmd & READ_ALL_SNV)
	{
		SNV_data_st snv_val;

		status = SNV_readValue(&snv_val);
		if(status == SNV_SUCCESS)
		{
			Display_print5(dispHandle, 3, 0, " SNV = 0x%x %d %d %d %d", snv_val.flag, snv_val.odr, snv_val.scale, snv_val.zero_g[0], snv_val.zero_g[1]);
		}
		else
			Display_print1(dispHandle, 3, 0, " SNV read error status=%d", status);
	}
	else if(cmd & INIT_ALL_SNV)
	{
		status = SNV_forceInit();
		if(status == SNV_SUCCESS)
			Display_print0(dispHandle, 3, 0, " SNV is initialized" );
		else
			Display_print1(dispHandle, 3, 0, " SNV initialize error status=%d", status);
	}
	else if(cmd & UPDATE_ODR_SNV)
	{
		value = AccelDbg_getValue();
		//Display_print1(dispHandle, 3, 0, "SNV value for update %d ", value);
		if(value != -1)
		{
			status = SNV_setItemValue(SNV_ACCEL_DATARATE_ITEM, (uint8_t)value);
			AccelDbg_setValueDone();
			Display_print2(dispHandle, 3, 0, " set Data Rate: %d status %d", value, status);
		}
		else
			Display_print0(dispHandle, 3, 0, "SNV value for update not ready ");

	}
	else if(cmd & UPDATE_SCALE_SNV)
	{
		value = AccelDbg_getValue();
		//Display_print1(dispHandle, 3, 0, "SNV value for update %d ", value);
		if(value != -1)
		{
			status = SNV_setItemValue(SNV_ACCEL_SCALE_ITEM, (uint8_t)value);
			AccelDbg_setValueDone();
#ifdef ACCELSENSOR_AUTOSCALE
			AccelSensor_restartCycle();
#endif
			Display_print2(dispHandle, 3, 0, " set Scale: %d status %d", value, status);
		}
		else
			Display_print0(dispHandle, 3, 0, "SNV value for update not ready ");

	}
	else if(cmd & INIT_SENS_DATA)
	{
		AccelSensor_enqueueMsg(SBP_ACCEL_CHANGE_EVT, ACCEL_SENSOR_DATA_INIT);
		AccelDbg_setValueDone();

		Display_print0(dispHandle, 3, 0, " SBP_SENS_DATA_INIT_EVT received");
	}
	else if(cmd & COMPENSATE_ZERO_G)
	{
		AccelSensor_enqueueMsg(SBP_ACCEL_CHANGE_EVT, ACCEL_COMPENSATE_ZERO_G);
		AccelDbg_setValueDone();

		Display_print0(dispHandle, 3, 0, " ACCEL_COMPENSATE_ZERO_G received");
	}
	else
		Display_print1(dispHandle, 3, 0, "unrecognized cmd=%d ", cmd);

	AccelSensor_start();
}
#endif

/*********************************************************************
 * @fn      AccelSensor_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void AccelSensor_stateChangeCB(gaprole_States_t newState)
{
  AccelSensor_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      AccelSensor_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void AccelSensor_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, 2, 0, "Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
      Display_print0(dispHandle, 2, 0, "Advertising");
      break;

#ifdef PLUS_BROADCASTER
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of
     * state to the application.  These are then disabled here so that sending
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        AccelSensor_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;

        numActive = linkDB_NumActive();

        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
          Display_print1(dispHandle, 2, 0, "Num Conns: %d", (uint16_t)numActive);
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(linkInfo.addr));
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

          Display_print0(dispHandle, 2, 0, "Connected");
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));
        }

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Display_print0(dispHandle, 2, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:

      AccelSensor_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, 2, 0, "Disconnected");

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      AccelSensor_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, 2, 0, "Timed Out");

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
      Display_print0(dispHandle, 2, 0, "Error");
      break;

    default:
      Display_clearLine(dispHandle, 2);
      break;
  }

  // Update the state
  gapProfileState = newState;
}



#ifdef ACCELSENSOR_CALCULATION
static void AccelSensor_printResultTask(void)
{
	int16 cur_ave[3], cur_div[3], cur_max[3], cur_min[3];
	int16 cur_ave_div[ACCEL_SENSOR_CALC_LEN], cur_max_min[ACCEL_SENSOR_CALC_LEN];
	int i;
	char axis[3]={'X', 'Y', 'Z'};

#if ACCELSENSOR_MERGE_CALC_RESULT
	Accel_GetParameter(ACCEL_AVE_DIV, cur_ave_div);
	for(i=0; i<3; i++) cur_ave[i] = cur_ave_div[i];
	for(i=0; i<3; i++) cur_div[i] = cur_ave_div[i+3];
	Accel_GetParameter(ACCEL_MAX_MIN, cur_max_min);
	for(i=0; i<3; i++) cur_max[i] = cur_max_min[i];
	for(i=0; i<3; i++) cur_min[i] = cur_max_min[i+3];
#else
	Accel_GetParameter(ACCEL_AVERAGE, cur_ave);
	Accel_GetParameter(ACCEL_DEVIATION, cur_div);
	Accel_GetParameter(ACCEL_MAX, cur_max);
	Accel_GetParameter(ACCEL_MIN, cur_min);
#endif

	if(AccelDbg_isDispLogOn()==2)
	{
		for(i=0; i<3; i++)
			Display_print5(dispHandle, 2, 0, "%c, %d, %d, %d, %d", axis[i], (int)cur_ave[i], (int)cur_div[i], (int)cur_max[i], (int)cur_min[i]);
	}
}
#endif


#ifdef FEATURE_OAD
/*********************************************************************
 * @fn      AccelSensor_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void AccelSensor_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_enqueue(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      AccelSensor_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void AccelSensor_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      AccelSensor_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void AccelSensor_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}

/*********************************************************************
 * @fn       AccelSensor_accelBleUpdateCB
 *
 * @brief   Called by the Accelerometer Profile when the Enabler Attribute
 *          is changed.
 *
 * @param   none
 *
 * @return  none
 */
static void AccelSensor_accelBleUpdateCB(uint8_t cmd, uint8_t event)
{
#if 1
	Display_print2(dispHandle, 2, 0, "SBP_ACCEL_CHANGE_EVT cmd=%d evt=%d", cmd, event);
	AccelSensor_enqueueMsg(SBP_ACCEL_CHANGE_EVT, cmd);

#else
  // Set Event.
  events |= cmd;

  // Post application's Semaphore.
  Semaphore_post(sem);
#endif
}


/***************************************************************
 * 	Auto sc
 */
static void AccelSensor_start(void)
{
	Util_startClock(&accelReadClock);
#ifndef ENABLE_STANDBY
	Board_Led_control(board_led_type_LED1, board_led_state_ON);
#endif
	AccelSensor_setSensorInterrupt(1);
}

static void AccelSensor_stop(void)
{
	Util_stopClock(&accelReadClock);
#ifndef ENABLE_STANDBY
	Board_Led_control(board_led_type_LED1, board_led_state_OFF);
#endif
	AccelSensor_setSensorInterrupt(0);
}

/*********************************************************************
 * @fn       AccelSensor_processBLEValueChangeEvt
 *
 * @brief   Process a callback from the Accelerometer Profile when the
 *          Enabler Attribute is changed.
 *
 * @param   none
 *
 * @return  none
 */
static void AccelSensor_processBLEValueChangeEvt(uint8_t event)
{
	uint8_t status=SUCCESS, ble_val;

	switch(event)
	{
	case ACCEL_SENSOR_DATA_INIT :
		//Display_print0(dispHandle, 2, 0, "AccelSensor_processBLEValueChangeEvt");
		status = Accel_GetParameter(ACCEL_DATA_INIT, &ble_val);
		if(status != SUCCESS)
		{
			Display_print0(dispHandle, 2, 0, "Get Parameter error Sensor data init");
			return ;
		}
		if(ble_val)
		{
			AccelSensor_initSensorData();
			AccelSensor_initCalcResult();

			Display_print0(dispHandle, 2, 0, "Whole sensor data initialized");
			ble_val = 0;
			status = Accel_SetParameter(ACCEL_DATA_INIT, sizeof(uint8), &ble_val); // reset to 0
		}
		break;

	case ACCEL_COMPENSATE_ZERO_G :
		AccelSensor_compensateZeroG();
		Display_print3(dispHandle, 2, 0, "compensate x=%d y=%d z=%d", grav_x, grav_y, grav_z);
		break;

	default:
		Display_print1(dispHandle, 2, 0, "Wrong BLE change event %d", event);
		break;

	}
}

/*********************************************************************
 * @fn      AccelSensor_processAccelReadEvt
 *
 * @brief   Process accelerometer task.
 *
 * @param   none
 *
 * @return  none
 */

#ifdef ACCELSENSOR_AUTOSCALE
void AccelSensor_restartCycle(void)
{
	sens_cycle_cnt = 0;
	sens_cycle_done = 0;
	AccelScale_initMinMax();

	Display_print0(dispHandle, 2, 0, "Autoscale restarted!");
}

int AccelSensor_getCycle(void)
{
	return sens_cycle_cnt;
}

int AccelSensor_isCycleDone(void)
{
	return sens_cycle_done;
}

void AccelSensor_changeScaleEvent(int scale)
{
	AccelSensor_enqueueMsg(SBP_ACCEL_CHANGE_SCALE_EVT, (uint8_t)scale);
}
#endif

#ifdef ACCELSENSOR_CALCULATION
static void AccelSensor_processAccelReadEvt(void)
{
	int i, smpl_count=0;
	uint8_t status;

#ifdef ACCELSENSOR_AUTOSCALE
	AccelScale_initOneSample();
#endif
	for(i=0; i<sensing_count; i++)
	{
		status = Acc_readStatus();
		if(Acc_isNewDataAvailable(status))
		{
			//test_ACC_output(1);
			AccelSensor_accelRead(i);
			smpl_count++;
			//test_ACC_output(0);
		}
	}

	Display_print3(dispHandle, 2, 0, "status=0x%x actual sample=%d / %d ", status, smpl_count, sensing_count);

	//AccelSensor_calcResult();

#ifdef ACCELSENSOR_AUTOSCALE
	if(AccelScale_isOn())
	{
		AccelScale_storeMinMax(sens_cycle_cnt, sensing_count);
		AccelScale_needScaleUp();
		AccelScale_needScaleDown();
	}
#endif
}
#endif

/*********************************************************************
 * @fn      AccelSensor_accelRead
 *
 * @brief   Called by the application to read accelerometer data
 *          and put data in accelerometer profile
 *
 * @param   none
 *
 * @return  none
 */

static void AccelSensor_accelRead(int index)
{
	int8_t new_x = 0, new_y = 0, new_z = 0;
	int result=0;

	//test_ACC_output(1);
	// Read data for each axis of the accelerometer
	result = Acc_readAccVal(&new_x, &new_y, &new_z);
	if(result == 0) Display_print1(dispHandle, 2, 0, "sensor read error idx=%d", index);

#ifdef ACCELSENSOR_AUTOSCALE
	AccelScale_storeOneSample(index, new_x, new_y, new_z); //sample for check auto scale
#endif

	AccelSensor_putSensorData(index, new_x, new_y, new_z);

	//test_ACC_output(0);
}

static void AccelSensor_initSensorData(void)
{
	int i;

	sensing_index = 0;
	for(i=0; i<SENSOR_BUF_LENGTH; i++)
	{
		sensor.x[i] = 0;
		sensor.y[i] = 0;
		sensor.z[i] = 0;
	}

	for(i=0; i<FFT_SAMPLE_MAX/2; i++)
	{
		mag_x[i]=0;
		mag_y[i]=0;
		mag_z[i]=0;
	}
}
#ifdef ACCELSENSOR_CALCULATION

static void AccelSensor_putSensorCalc(int16 *ave, int16 *dev, int16 *max, int16 *min)
{
	int idx = sens_index;

	sens_ave.x[idx] = (int16)ave[0];
	sens_ave.y[idx] = (int16)ave[1];
	sens_ave.z[idx] = (int16)ave[2];

	sens_dev.x[idx] = (int16)dev[0];
	sens_dev.y[idx] = (int16)dev[1];
	sens_dev.z[idx] = (int16)dev[2];

	sens_max.x[idx] = (int16)max[0];
	sens_max.y[idx] = (int16)max[1];
	sens_max.z[idx] = (int16)max[2];

	sens_min.x[idx] = (int16)min[0];
	sens_min.y[idx] = (int16)min[1];
	sens_min.z[idx] = (int16)min[2];

	sens_index++;
	sens_index %= ACCEL_SENSOR_POINT_COUNT;

#ifdef ACCELSENSOR_AUTOSCALE
	sens_cycle_cnt++;
	if(sens_cycle_cnt >= ACCEL_SENSOR_POINT_COUNT)
	{
		sens_cycle_cnt=0;
		sens_cycle_done=1;
		//Display_print1(dispHandle, 2, 0, "sensor cycle(%d) done ", ACCEL_SENSOR_POINT_COUNT);
	}
#endif
}
#endif

static double AccelSensor_getScale(void)
{
	double scale_factor[4] = {15.6, 31.2, 62.5, 187.5}; // sensitivity in table 3 of LIS2DE12 datasheet
	uint8_t scale_idx;

	scale_idx = SNV_getItemValue(SNV_ACCEL_SCALE_ITEM);
	return scale_factor[scale_idx];
}

static void AccelSensor_putSensorData(int idx, int8_t x, int8_t y, int8_t z)
{
	double scale = AccelSensor_getScale();

	sensor.x[idx] = (int16)round((double)(x*scale)) - grav_x;
	sensor.y[idx] = (int16)round((double)(y*scale)) - grav_y;
	sensor.z[idx] = (int16)round((double)(z*scale)) - grav_z;

//	if(AccelDbg_isSensorLogOn())
//	{
//		Display_print4(dispHandle, 2, 0, "D=%d\t%d\t%d\t%d", idx, (int)sensor.x[idx], (int)sensor.y[idx], (int)sensor.z[idx]);
//	}
}

#ifdef ACCELSENSOR_CALCULATION
static double AccelSensor_calcDeviation(int16 *sensor_data, int ave)
{
	int i;
	double diff, deviation=0.0;

	for(i=0; i<sensing_count; i++)
	{
		diff = (double)((int)sensor_data[i] - ave);
		diff = diff*diff;

		deviation += diff;
//		if(AccelDbg_isDispLogOn()==3)
//			Display_print4(dispHandle, 2, 0, "diff %d, %d, %d, %d", i, (int)sensor_data[i], (int)ave, (int)diff);
	}
//	if(AccelDbg_isDispLogOn()==3)
//		Display_print3(dispHandle, 2, 0, "dev %d, %d, %d", (int)ave, (int)deviation, (int)sqrt(deviation));

	return sqrt(deviation);
}

static void AccelSensor_calcResult(void)
{
	int i;
	int ave[ACCEL_SENSOR_DATA_LEN], total[ACCEL_SENSOR_DATA_LEN], deviation[ACCEL_SENSOR_DATA_LEN];
	int max[ACCEL_SENSOR_DATA_LEN]={-24000,-24000,-24000}, min[ACCEL_SENSOR_DATA_LEN]={24000,24000,24000};
#if ACCELSENSOR_MERGE_CALC_RESULT
	int16 sensor_ave_div[ACCEL_SENSOR_CALC_LEN], sensor_max_min[ACCEL_SENSOR_CALC_LEN];
	uint8 len = ACCEL_SENSOR_CALC_LEN*sizeof(int16);
#else
	uint8 len = ACCEL_SENSOR_DATA_LEN*sizeof(int16);
#endif
	int16 ave16[ACCEL_SENSOR_DATA_LEN], dev16[ACCEL_SENSOR_DATA_LEN];
	int16 max16[ACCEL_SENSOR_DATA_LEN], min16[ACCEL_SENSOR_DATA_LEN];

	//total
	total[0]=0; total[1]=0; total[2]=0;
	for(i=0; i<sensing_count; i++)
	{
		total[0] += (int)sensor.x[i];
		total[1] += (int)sensor.y[i];
		total[2] += (int)sensor.z[i];

		if(max[0] < (int)sensor.x[i]) max[0] = (int)sensor.x[i];
		if(min[0] > (int)sensor.x[i]) min[0] = (int)sensor.x[i];

		if(max[1] < (int)sensor.y[i]) max[1] = (int)sensor.y[i];
		if(min[1] > (int)sensor.y[i]) min[1] = (int)sensor.y[i];

		if(max[2] < (int)sensor.z[i]) max[2] = (int)sensor.z[i];
		if(min[2] > (int)sensor.z[i]) min[2] = (int)sensor.z[i];

		//time_cnt++;
		if(AccelDbg_isDispLogOn()==3)
			Display_print4(dispHandle, 2, 0, "D=%d, %d, %d, %d", i, (int)sensor.x[i], (int)sensor.y[i], (int)sensor.z[i]);
		  	//Display_print4(dispHandle, 2, 0, "%u, %d, %d, %d", time_cnt, (int8_t)new_x, (int8_t)new_y, (int8_t)new_z);
	}

	//average
	for(i=0; i<3; i++)
		ave[i] = (int)round((double)total[i]/(double)sensing_count);

	//standard deviation
	deviation[0] = (int)AccelSensor_calcDeviation(&sensor.x[0], ave[0]);
	deviation[1] = (int)AccelSensor_calcDeviation(&sensor.y[0], ave[1]);
	deviation[2] = (int)AccelSensor_calcDeviation(&sensor.z[0], ave[2]);

#if ACCELSENSOR_MERGE_CALC_RESULT
	int j=0;
	for(i=0; i<ACCEL_SENSOR_DATA_LEN; i++) sensor_ave_div[j++] = (int16)ave[i];
	for(i=0; i<ACCEL_SENSOR_DATA_LEN; i++) sensor_ave_div[j++] = (int16)deviation[i];
	j=0;
	for(i=0; i<ACCEL_SENSOR_DATA_LEN; i++) sensor_max_min[j++] = (int16)max[i];
	for(i=0; i<ACCEL_SENSOR_DATA_LEN; i++) sensor_max_min[j++] = (int16)min[i];
#endif
	for(i=0; i<ACCEL_SENSOR_DATA_LEN; i++)
	{
		ave16[i] = (int16)ave[i];
		dev16[i] = (int16)deviation[i];
		max16[i] = (int16)max[i];
		min16[i] = (int16)min[i];
	}


	AccelSensor_putSensorCalc(ave16, dev16, max16, min16);

	if(AccelDbg_isDispLogOn()==3) // TODO : 3 -> 2
	//if(AccelDbg_isSensorLogOn())
	{
		for(i=0; i<3; i++)
			Display_print5(dispHandle, 2, 0, "total=%d, ave=%d, dev=%d, max=%d, min=%d", total[i], (int)ave16[i], (int)dev16[i], max16[i], min16[i]);
	}
	AccelDbg_updateLogCount();

#if ACCELSENSOR_MERGE_CALC_RESULT
	Accel_SetParameter(ACCEL_AVE_DIV, len, (int16 *)sensor_ave_div);
	Accel_SetParameter(ACCEL_MAX_MIN, len, (int16 *)sensor_max_min);
#else
	Accel_SetParameter(ACCEL_AVERAGE, len, (int16 *)ave16);
	Accel_SetParameter(ACCEL_DEVIATION, len, (int16 *)dev16);
	Accel_SetParameter(ACCEL_MAX, len, (int16 *)max16);
	Accel_SetParameter(ACCEL_MIN, len, (int16 *)min16);
#endif

}
#endif

static int sens_int_enabled=1;
int AccelSensor_isSensorInterruptEnabled(void)
{
	return sens_int_enabled;
}

void AccelSensor_setSensorInterrupt(int enabled)
{
	sens_int_enabled = enabled;
}

static void AccelSensor_postTaskEvent(uint16_t evt)
{
  // Store the event.
  events |= evt;

  // Wake up the application.
  Semaphore_post(sem);
}

void AccelSensor_sendSensorIntEvent(void)
{
	//AccelSensor_enqueueMsg(SBP_SENSOR_INT_EVT, SBP_SENSOR_INT_MSG);
	AccelSensor_postTaskEvent(SBP_SENSOR_INT_EVT);
}

static void AccelSensor_sendSensorReadEndEvent(void)
{
	AccelSensor_enqueueMsg(SBP_SENSOR_INT_EVT, SBP_SENSOR_READ_END_MSG);
}

static void AccelSensor_processSensorInterruptMsg(uint8_t event)
{
	switch(event)
	{
//	case SBP_SENSOR_INT_MSG :
//		AccelSensor_initSensorInt();
//		break;

	case SBP_SENSOR_READ_END_MSG :
    	AccelSensor_postProcessSensorData();
		break;
	}
}

static void AccelSensor_initCalcResult(void)
{
	int i;

	sens_index = 0;
	for(i=0; i<ACCEL_SENSOR_POINT_COUNT; i++)
	{
		sens_ave.x[i] = 0;
		sens_ave.y[i] = 0;
		sens_ave.z[i] = 0;
	}
#ifdef ACCELSENSOR_AUTOSCALE
	sens_cycle_cnt = 0;
	sens_cycle_done = 0;
#endif
}

static void AccelSensor_putCalcResult(int *ave)
{
	int idx = sens_index;

	sens_ave.x[idx] = (int16)ave[0];
	sens_ave.y[idx] = (int16)ave[1];
	sens_ave.z[idx] = (int16)ave[2];

	sens_index++;
	sens_index %= ACCEL_SENSOR_POINT_COUNT;
}

static int AccelSensor_calcAverage(void)
{
	int i;
	int ave[3], total[3]={0,0,0};

	//total
	for(i=0; i<sensing_count; i++)
	{
		total[0] += (int)sensor.x[i];
		total[1] += (int)sensor.y[i];
		total[2] += (int)sensor.z[i];
	}

	//average
	for(i=0; i<3; i++)
		ave[i] = (int)round((double)total[i]/(double)sensing_count);

	AccelSensor_putCalcResult(ave);

	return 1;
}

static void AccelSensor_FFT_transform(int16_t *in, uint16_t *out)
{
	int i;

	for(i=0; i<sensing_count; i++)
	{
		real_in[i] = (float)in[i];
		img_in[i] = 0.0;
	}
	fft_transform(real_in, img_in);
	fft_getMagnitude(out, real_in, img_in);
}

static void AccelSensor_postProcessSensorData(void)
{
	AccelSensor_setSensorInterrupt(0); //disable interrupt

	// calculate average
	AccelSensor_calcAverage();

	//set sensor power down till next read period
	Acc_setPowerMode(0);

	// perform FFT transform for X, Y, Z
	//test_ACC_output(1);
	AccelSensor_FFT_transform(sensor.x, mag_x);
	AccelSensor_FFT_transform(sensor.y, mag_y);
	AccelSensor_FFT_transform(sensor.z, mag_z);
	//test_ACC_output(0);

#ifdef USE_ACCEL_DEBUG_TASK
	//Display_print1(dispHandle, 2, 0, "AccelSensor_postProcessSensorData n=%d", sensing_count);
	if(AccelDbg_isSensorLogOn())
	{
		int i;

		for(i=0; i<sensing_count/2; i++)
		{
			Display_print4(dispHandle, 2, 0, "F=%d\t%d\t%d\t%d", i, (int)mag_x[i], (int)mag_y[i], (int)mag_z[i]);
		}
		Display_print0(dispHandle, 2, 0, " ");

		for(i=0; i<sensing_count; i++)
		{
			Display_print4(dispHandle, 2, 0, "D=%d\t%d\t%d\t%d", i, (int)sensor.x[i], (int)sensor.y[i], (int)sensor.z[i]);
		}
		Display_print0(dispHandle, 2, 0, " ");
		Display_print0(dispHandle, 2, 0, " ");

		if(AccelDbg_LastLogCount())
			AccelDbg_showPrompt();
	}
	AccelDbg_updateLogCount();
#endif

	//Display_print0(dispHandle, 2, 0, "AccelSensor_postProcessSensorData()");

    // Allow STANDBY mode again
    //Power_releaseConstraint(PowerCC26XX_SB_DISALLOW);
}

static void AccelSensor_processSensorInterrupt(void)
{
//	uint8_t status;
	//int8_t new_x = 0, new_y = 0, new_z = 0;

//	status = Acc_readStatus();
//	if(Acc_isNewDataAvailable(status))
	{// Read data for each axis of the accelerometer
#if 0
		Acc_readAccVal(&new_x, &new_y, &new_z);

#else
		AccelSensor_accelRead(sensing_index);
		if(sensing_index < sensing_count)
			sensing_index++;
		else
		{
			//Display_print0(dispHandle, 2, 0, "sensor interrupt process ended");
			AccelSensor_sendSensorReadEndEvent();
			sensing_index=0;
		}
#endif
	}
	//test_ACC_output(0);
}

static void AccelSensor_initSensorInt(void)
{
	int8_t dummy;

	// Disallow STANDBY mode while using the sensor.
	//Power_setConstraint(PowerCC26XX_SB_DISALLOW);

	AccelSensor_initSensorData();
	//wakeup sensor
	Acc_setPowerMode(1);
	Task_sleep(50);
	Acc_readAccVal(&dummy, &dummy, &dummy); // dummy read to start read interrupt
	AccelSensor_setSensorInterrupt(1); //enable collecting data

}


static int AccelSensor_isBatteryPeriod(uint32_t count)
{
	// advertisement period 0.5sec -> battery check period : 10sec
	return (count%20)?0:1;
}

static void AccelSensor_readBatteryLevel(uint32_t count)
{
	int batt_val=0;

	if(!AccelSensor_isBatteryPeriod(count)) return;

	// perform battery level check
	batt_val = Batt_ReadBattLevel();
    //Display_print1(dispHandle, 2, 0, "batt = %d", batt_val);

}

static int AccelSensor_isTemperaturePeriod(uint32_t count)
{
	// advertisement period 0.5sec -> battery check period : 10sec
	return (count%20)?0:1;
}

static void AccelSensor_readTemperature(uint32_t count)
{
	int temperature;

	if(!AccelSensor_isTemperaturePeriod(count)) return;

	temperature = batt_readInternalTemperature();
//	if(temperature != -100)
//		Display_print1(dispHandle, 2, 0, "temperature = %d", temperature);
}

static int AccelSensor_isLEDOn(uint32_t count)
{
	return (count%4)?0:1;
}

static void AccelSensor_RunLED(uint32_t count, board_led_state *status)
{
	if(!AccelSensor_isLEDOn(count)) return;

	*status = board_led_state_ON;
	Board_Led_control(board_led_type_LED1, *status);
}

void AccelSensor_getZeroG(int *zero_g)
{
	zero_g[0] = (int)grav_x;
	zero_g[1] = (int)grav_y;
	zero_g[2] = (int)grav_z;
}

void AccelSensor_getZeroG_SNV(int *zero_g)
{
	SNV_getMultiItem(SNV_ACCEL_ZERO_G_ITEM, zero_g);
}

void AccelSensor_initZeroG(int *zero_g)
{
	grav_x = (int16_t)zero_g[0];
	grav_y = (int16_t)zero_g[1];
	grav_z = (int16_t)zero_g[2];
}

#define TEST_NUM	100
static void AccelSensor_compensateZeroG(void)
{
	int16_t a_x[TEST_NUM], a_y[TEST_NUM], a_z[TEST_NUM];
	int8_t ax, ay, az;
	int i, gravity[3], total[3]={0,0,0};
	double scale = AccelSensor_getScale();

	AccelSensor_stop();
	Acc_setZeroGMode();

	for(i=0; i<TEST_NUM; i++)
	{
		Acc_readAccVal(&ax, &ay, &az);
		Task_sleep(5);
		a_x[i] = (int16_t)round((double)(ax*scale));
		a_y[i] = (int16_t)round((double)(ay*scale));
		a_z[i] = (int16_t)round((double)(az*scale));
	}

	//total
	for(i=0; i<TEST_NUM; i++)
	{
		total[0] += (int)a_x[i];
		total[1] += (int)a_y[i];
		total[2] += (int)a_z[i];
	}

	//average
	gravity[0] = (int)round((double)total[0]/(double)TEST_NUM);
	gravity[1] = (int)round((double)total[1]/(double)TEST_NUM);
	gravity[2] = (int)round((double)total[2]/(double)TEST_NUM);

	if(SNV_setMultiItem(SNV_ACCEL_ZERO_G_ITEM, gravity) != SNV_SUCCESS)
		Display_print0(dispHandle, 2, 0, "store SNV error");

#ifdef DETECT_MOTOR_ON
	AccelSensor_setSensorMode(ACCEL_SENSOR_MODE_WAITING);
#else
	Acc_identify();
#endif

	// initialize all sensor data
	AccelSensor_initSensorData();
	AccelSensor_initCalcResult();

	AccelSensor_start();
}

void AccelSensor_error(void)
{
	while(1) Board_Led_control(board_led_type_LED1, board_led_state_ON);
}

/*********************************************************************
*********************************************************************/
