/*******************************************************************************
 * INCLUDES
 */
#include <string.h>
#include <xdc/std.h>

#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#ifdef POWER_SAVING
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#endif

#include <ICall.h>

#include <ti/drivers/pin/PINCC26XX.h>

// DriverLib
#include <driverlib/aon_batmon.h>   
#include <driverlib/pwr_ctrl.h>


#include "gatt.h"
//#include "hci.h"
#include "hci_tl.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "osal_snv.h"
#include "ICallBleAPIMSG.h"
#include "util.h"

#include "bsp_i2c.h"
#include "bsp_uart.h"

#include "Board.h"
#include "devinfoservice.h"
#include "sensor.h"
#include "st_util.h"

#include "Controller_Keys.h"
#include "AHRS.h"
#include "ahrsservice.h"
#include "FIG.h"
#include "figservice.h"
#include "recordservice.h"


/*******************************************************************************
 * CONSTANTS
 */

#define FW_VERSION_STR TOSTRING(FW_VERSION)" ("__DATE__")";

// How often to perform periodic event (in msec)
#define ST_PERIODIC_EVT_PERIOD               1000

// What is the advertising interval when device is discoverable
// (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          100

// RSSI Read periodic (units of 1ms, 1000=1s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_RSSI_READ_RATE        1

// Whether to enable automatic parameter update request when a
// connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         1

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D
#define TI_ST_DEVICE_ID                       0x03
#define TI_ST_KEY_DATA_ID                     0x00

// Length of board address
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

// Task configuration
#define ST_TASK_PRIORITY                      1
#define ST_TASK_STACK_SIZE                    700

// Internal Events for RTOS application
#define ST_STATE_CHANGE_EVT                   0x0001
#define ST_CHAR_CHANGE_EVT                    0x0002
#define ST_PERIODIC_EVT                       0x0004
#ifdef FEATURE_OAD
  #define SBP_OAD_WRITE_EVT                     0x0008
#endif //FEATURE_OAD

// Misc.
#define INVALID_CONNHANDLE                    0xFFFF
#define TEST_INDICATION_BLINKS                5  // Number of blinks
#define BLINK_DURATION                        20 // Milliseconds
#define OAD_PACKET_SIZE                       18
#define KEY_STATE_OFFSET                      13 // Offset in advertising data
   
#define PowerLowV                             360
#define PowerTargetV                          3000u
   
//Addition TX Pin to IOID_27
#define Debug_AUX_TX

/*******************************************************************************
 * TYPEDEFS
 */
// App event passed from profiles.
typedef struct
{
  uint8_t event;  // Which profile's event
  uint8_t serviceID; // New status
  uint8_t paramID;
} stEvt_t;

/*******************************************************************************
 * GLOBAL VARIABLES
 */
// Profile state and parameters
gaprole_States_t gapProfileState = GAPROLE_INIT;

// Semaphore globally used to post events to the application thread
ICall_Semaphore sem;

// Global pin resources
PIN_State pinGpioState;
PIN_Handle hGpioPin;

volatile int8 CRSSi;
uint8_t Status;// 0x00: init, 0x01: waitting, 0x02: Connected, 0x03: advertising

/*******************************************************************************
 * LOCAL VARIABLES
 */

// Task configuration
static Task_Struct ControllerTask;
static Char ControllerTaskStack[ST_TASK_STACK_SIZE];


// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// events flag for internal application events.
static uint16_t events;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x15,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'C', 'C', '2', '6', '5', '0', ' ',
  'C', 'o', 'n',  't',  'r',  'o',  'l',  'l',  'e',  'r', ' ', ' ', ' ', 
    
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
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
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_LCD
  LO_UINT16(DISPLAY_SERV_UUID),
  HI_UINT16(DISPLAY_SERV_UUID),
#else
  LO_UINT16(AHRS_SERV_UUID),
  HI_UINT16(AHRS_SERV_UUID),
#endif

  // Manufacturer specific advertising data
  0x06,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  LO_UINT16(TI_COMPANY_ID),
  HI_UINT16(TI_COMPANY_ID),
  TI_ST_DEVICE_ID,
  TI_ST_KEY_DATA_ID,
  0x00                                    // Key state
};

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Controller 2.0";

#if FEATURE_OAD
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// Device information parameters
static const uint8_t devInfoModelNumber[] = "CC2650 Controller";
static const uint8_t devInfoNA[] =          "N.A.";
static const uint8_t devInfoFirmwareRev[] = FW_VERSION_STR;
static const uint8_t devInfoMfrName[] =     "Texas Instruments";
static const uint8_t devInfoHardwareRev[] = "PCB 1.2/1.3";

// Pins that are actively used by the application
static PIN_Config ControllerAppPinTable[] =
{
    Board_LED1       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */
    Board_LED2       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off             */
    Board_KEY_LEFT   | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,        /* Button is active low          */
    Board_KEY_RIGHT  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,        /* Button is active low          */
    Board_RELAY      | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,      /* Relay is active high          */
    Board_BUZZER     | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* Buzzer initially off          */
    PeripheralKey1   | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,
    Board_POWERIO    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
    Board_POWERPSIO  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
#ifdef Debug_AUX_TX
    Board_AUX_TX     | PIN_INPUT_DIS | PIN_PUSHPULL | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH,
#endif
    PIN_TERMINATE
};

static uint8_t BatteryConfigured = 0;
static uint8_t PowerEnable = 0;
static uint8_t CalibWriteScheduled = 0;
/*******************************************************************************
 * LOCAL FUNCTIONS
 */

static void Controller_init( void);
//static void Controller_blinkLed(uint8_t led, uint8_t nBlinks);
static void Controller_taskFxn( UArg a0, UArg a1);
static void Controller_processStackMsg( ICall_Hdr *pMsg);
static void Controller_processGATTMsg(gattMsgEvent_t *pMsg);
static void Controller_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void Controller_processAppMsg( stEvt_t *pMsg);
static void Controller_processStateChangeEvt( gaprole_States_t newState ) ;
static void Controller_processCharValueChangeEvt( uint8_t serviceID, uint8_t paramID ) ;
static void Controller_performPeriodicTask( void);
static void Controller_stateChangeCB( gaprole_States_t newState);
static void Controller_resetAllSensors(void);
static void Controller_clockHandler(UArg arg);
static void Controller_enqueueMsg(uint8_t event, uint8_t serviceID, uint8_t paramID);
static void Controller_callback(PIN_Handle handle, PIN_Id pinId);
static void Controller_setDeviceInfo(void);
static void Controller_PowerEnable(uint8_t Enable);
static void Controller_StoreCalibration(uint8_t paramID);


#ifdef FEATURE_OAD
static void Controller_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

/*******************************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t Controller_gapRoleCBs =
{
  Controller_stateChangeCB,     // Profile State Change Callbacks
};

#ifdef FEATURE_OAD
static gapRolesParamUpdateCB_t paramUpdateCB =
{
  ControllerConnControl_paramUpdateCB
};

static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
  NULL,                       // Read Callback.  Optional.
  Controller_processOadWriteCB // Write Callback.  Mandatory.
};
#endif //FEATURE_OAD


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */

uint32_t GetBatteryVoltage(void)
{
  uint32_t BatteryVoltage;
  if(BatteryConfigured)
  {
    BatteryVoltage = AONBatMonBatteryVoltageGet();
    BatteryVoltage = (BatteryVoltage * 125) >> 5;
    return BatteryVoltage;
  }
  else
    return 0;
}

/*******************************************************************************
 * @fn      Controller_createTask
 *
 * @brief   Task creation function for the Controller.
 *
 * @param   none
 *
 * @return  none
 */
void Controller_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = ControllerTaskStack;
  taskParams.stackSize = ST_TASK_STACK_SIZE;
  taskParams.priority = ST_TASK_PRIORITY;

  Task_construct(&ControllerTask, Controller_taskFxn, &taskParams, NULL);
}
/*******************************************************************************
 * @fn      Controller_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   none
 *
 * @return  none
 */
static void Controller_init(void)
{
  uint8_t selfTestMap;
  uint32_t LocalBatteryVoltage = 0u;
  // Enable Battery Moninter
  AONBatMonEnable();
  BatteryConfigured = 1;
  
  // Setup I2C for sensors
  bspI2cInit();
  bspUartInit();
  
  // Handling of buttons, LED, relay
  hGpioPin = PIN_open(&pinGpioState, ControllerAppPinTable);
  //PIN_registerIntCb(hGpioPin, Controller_callback);
  
  
#ifdef Debug_AUX_TX  
  PINCC26XX_setMux(hGpioPin, Board_AUX_TX, IOC_PORT_MCU_UART0_TX);
#endif
  
  CRSSi = -127;
  Status = 0;
	// ***************************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ***************************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, Controller_clockHandler,
                      ST_PERIODIC_EVT_PERIOD, 0, false, ST_PERIODIC_EVT);


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
    uint16_t desiredRssiReadRate = DEFAULT_DESIRED_RSSI_READ_RATE;

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
    GAPRole_SetParameter(GAPROLE_RSSI_READ_RATE, sizeof(uint16_t),
                         &desiredRssiReadRate);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void*)attDeviceName);
  
#ifdef FEATURE_OAD
  // Register connection parameter update
  GAPRole_RegisterAppCBs( &paramUpdateCB);
#endif

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

  
  
  System_printf("AON_Temp : %d \n\r", AONBatMonTemperatureGetDegC());
  delay_ms(10);
  LocalBatteryVoltage = GetBatteryVoltage() + 1348u;
  LocalBatteryVoltage /= 10;
  System_printf("Battery  : %u.%u V \n\r", LocalBatteryVoltage / 100, LocalBatteryVoltage % 100 );
  
  if( LocalBatteryVoltage <= PowerLowV) // Power Low Err
  {
    uint8_t i;
    System_printf("Low Power!!!.\n\r");
    for(i = 0; i< 5; i++)
    {
      Controller_blinkLed(Board_LED1, TEST_INDICATION_BLINKS);
      delay_ms(100);
    }
    System_printf("System Halt.\n\r");
    PowerCtrlStateSet(PWRCTRL_STANDBY);
    while(1);//Halt
  }
  
  Controller_PowerEnable(0x01);
  
  // Add application specific device information
  Controller_setDeviceInfo();

  // Power on self-test for sensors, flash and DevPack
  selfTestMap = sensorTestExecute(ST_TEST_MAP);
  
  if (selfTestMap == ST_TEST_MAP)
  {
    Controller_blinkLed(Board_LED2,TEST_INDICATION_BLINKS);
    System_printf("selfTestMap Pass.\n\r");
  }
  else
  {
    Controller_blinkLed(Board_LED1,TEST_INDICATION_BLINKS);
    System_printf("selfTestMap Failed(Got :0x%x).\n\r", selfTestMap);
    //FigTestResult
  }
  System_printf("FigTestResult : 0x%x \n\r", FigTestResult());
  
  // Initialize sensors who don't have their own tasks
  ControllerKeys_init();
  ControllerFIG_init();
  
  // Start the Device
  GAPRole_StartDevice(&Controller_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(NULL);
  
  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);
  
  // Enable interrupt handling for keys and relay
  PIN_registerIntCb(hGpioPin, Controller_callback);
}

/*******************************************************************************
 * @fn      Controller_taskFxn
 *
 * @brief   Application task entry point for the Controller
 *
 * @param   a0, a1 (not used)
 *
 * @return  none
 */
static void Controller_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  Controller_init();
  LCD_WRITES_STATUS("Controller Start");
  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signalled when a
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
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          Controller_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        stEvt_t *pMsg = (stEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          Controller_processAppMsg(pMsg);
          
          // Free the space from the message.
          ICall_free(pMsg);
        }
      }

      // Process new data if available
      ControllerKeys_processEvent();
      ControllerFig_processSensorEvent();
    }

    if (!!(events & ST_PERIODIC_EVT))
    {
      events &= ~ST_PERIODIC_EVT;

      if (gapProfileState == GAPROLE_CONNECTED
          || gapProfileState == GAPROLE_ADVERTISING)
      {
        Util_startClock(&periodicClock);
      }

      // Perform periodic application task
      if (gapProfileState == GAPROLE_CONNECTED)
      {
        Status = 0x02;
        Controller_performPeriodicTask();
      }

      // Blink green LED when advertising
      if (gapProfileState == GAPROLE_ADVERTISING)
      {
        Status = 0x03;
        Controller_blinkLed(Board_LED2,1);
        #ifdef FEATURE_LCD
        Controller_displayBatteryVoltage();
        #endif
      }
      if (CalibWriteScheduled != 0x00)
      {
        Controller_StoreCalibration(CalibWriteScheduled);
        CalibWriteScheduled = 0x00;
      }
      // Check Power Status
      if (PowerEnable)
      {
        if(GetBatteryVoltage() < PowerTargetV)
        {
          Controller_PowerEnable(false);
          System_printf("Power On Failed.\n\r");
          Controller_blinkLed(Board_LED1, 40);
          if (gapProfileState == GAPROLE_CONNECTED)
          {
            // Disconnect
            GAPRole_TerminateConnection();
            System_printf("Bluetooth Disconnect.\n\r");
          }
        }
      }
    }
  } // task loop
}


/*******************************************************************************
 * @fn      Controller_setDeviceInfo
 *
 * @brief   Set application specific Device Information
 *
 * @param   none
 *
 * @return  none
 */
static void Controller_setDeviceInfo(void)
{
  DevInfo_SetParameter(DEVINFO_MODEL_NUMBER, sizeof(devInfoModelNumber),
                       (void*)devInfoModelNumber);
  DevInfo_SetParameter(DEVINFO_SERIAL_NUMBER, sizeof(devInfoNA),
                       (void*)devInfoNA);
  DevInfo_SetParameter(DEVINFO_SOFTWARE_REV, sizeof(devInfoNA),
                       (void*)devInfoNA);
  DevInfo_SetParameter(DEVINFO_FIRMWARE_REV, sizeof(devInfoFirmwareRev),
                       (void*)devInfoFirmwareRev);
  DevInfo_SetParameter(DEVINFO_HARDWARE_REV, sizeof(devInfoHardwareRev),
                       (void*)devInfoHardwareRev);
  DevInfo_SetParameter(DEVINFO_MANUFACTURER_NAME, sizeof(devInfoMfrName),
                       (void*)devInfoMfrName);
}

/*******************************************************************************
 * @fn      Controller_PowerEnable
 *
 * @brief   Set Board Bosster, Spin here if Power failed.
 *
 * @param   Enable
 *
 * @return  none
 */
static void Controller_PowerEnable(uint8_t Enable)
{
  if(Enable)
  {
    PIN_setOutputValue(hGpioPin, Board_POWERIO, Board_POWER_ON);
    delay_ms(10);
    
    if(GetBatteryVoltage() < PowerTargetV)
    {
      uint8_t i = 0;
      PIN_setOutputValue(hGpioPin, Board_POWERIO, Board_POWER_OFF);
      delay_ms(1);
      PIN_setOutputValue(hGpioPin, Board_POWERPSIO, Board_POWERPS_OFF);
      delay_ms(1);
      PIN_setOutputValue(hGpioPin, Board_POWERIO, Board_POWER_ON);
      for(i = 0; i < 9 ; i++ )
      {
        if(GetBatteryVoltage() < PowerTargetV)
        {
          Controller_blinkLed(Board_LED1, TEST_INDICATION_BLINKS);
          System_printf("Power On Failed. VDDB = %u mV\n\r", GetBatteryVoltage());
          PIN_setOutputValue(hGpioPin, Board_POWERIO, Board_POWER_OFF);
          delay_ms(1);
          PIN_setOutputValue(hGpioPin, Board_POWERIO, Board_POWER_ON);
          delay_ms(799);
        }
        else
        {
          System_printf("Power Reset Success, VDDB = %u mV.\n\r", GetBatteryVoltage());
          PowerEnable = 0x01;
          return;
        }
      }
      if(GetBatteryVoltage() < PowerTargetV)
      {
        System_printf("Power On Failed.\n\r");
        PIN_setOutputValue(hGpioPin, Board_POWERIO, Board_POWER_OFF);
        
        System_printf("System Halt.\n\r");
        
        Controller_blinkLed(Board_LED1, TEST_INDICATION_BLINKS);
        PowerCtrlStateSet(PWRCTRL_STANDBY);
        while(1);//Halt
      }
    }
    else
    {
      System_printf("Power On Success.\n\r");
    }
    PowerEnable = 0x01;
  }
  else
  {
    PIN_setOutputValue(hGpioPin, Board_POWERIO, Board_POWER_OFF);
    PowerEnable = 0x00;
  }
}
/*******************************************************************************
 * @fn      Controller_StoreCalibration
 *
 * @brief   Write calibration data into Flash
 *
 * @param   none
 *
 * @return  none
 */

static void Controller_StoreCalibration(uint8_t paramID)
{
  System_printf("Controller_StoreCalibration Called with paramID = 0x%x\n\r", paramID);
  return;

}

/*******************************************************************************
 * @fn      Controller_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Controller_processAppMsg(stEvt_t *pMsg)
{
  switch (pMsg->event)
  {
    case ST_STATE_CHANGE_EVT:
      Controller_processStateChangeEvt((gaprole_States_t)pMsg->serviceID);
      break;

    case ST_CHAR_CHANGE_EVT:
      Controller_processCharValueChangeEvt(pMsg->serviceID, pMsg->paramID);
      break;
      
    default:
      // Do nothing.
      break;
  }
}

/*******************************************************************************
 * @fn      Controller_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void Controller_stateChangeCB(gaprole_States_t newState)
{
  Controller_enqueueMsg(ST_STATE_CHANGE_EVT, newState, NULL);
}


/*******************************************************************************
 * @fn      Controller_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void Controller_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch (newState)
  {
  case GAPROLE_STARTED:
    {
      uint8_t ownAddress[B_ADDR_LEN];
      uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

      Controller_blinkLed(Board_LED2, 5);

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
      
      scanRspData[19] = ownAddress[0] / 16;
      scanRspData[20] = ownAddress[0] % 16;
      scanRspData[19] += (scanRspData[19] > 9)? ('a' - 10) : '0';
      scanRspData[20] += (scanRspData[20] > 9)? ('a' - 10) : '0';
      GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
      
      LCD_WRITES_STATUS("Initialized");
    }
    break;

  case GAPROLE_ADVERTISING:
    // Start the clock
    if (!Util_isActive(&periodicClock))
    {
      Util_startClock(&periodicClock);
    }
    if(!PowerEnable)
      Controller_PowerEnable(true);
    // Make sure key presses are not stuck
    Controller_updateAdvertisingData(0);

    LCD_WRITES_STATUS("Advertising");
    break;

  case GAPROLE_CONNECTED:
    {
      // Start the clock
      if (!Util_isActive(&periodicClock))
      {
        Util_startClock(&periodicClock);
      }

      // Turn off LEDs and buzzer
      PIN_setOutputValue(hGpioPin, Board_LED1, Board_LED_OFF);
      PIN_setOutputValue(hGpioPin, Board_LED2, Board_LED_OFF);
      PIN_setOutputValue(hGpioPin, Board_BUZZER, Board_BUZZER_OFF);
#ifdef FEATURE_OAD
      ControllerConnectionControl_update();
#endif

#ifdef PLUS_BROADCASTER
      // Only turn advertising on for this state when we first connect
      // otherwise, when we go from connected_advertising back to this state
      // we will be turning advertising back on.
      if (firstConnFlag == false)
      {
        uint8_t advertEnabled = TRUE; // Turn on Advertising

        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        firstConnFlag = true;
      }
#endif // PLUS_BROADCASTER
    }
    LCD_WRITES_STATUS("Connected");
    break;

  case GAPROLE_CONNECTED_ADV:
    break;

  case GAPROLE_WAITING:
  case GAPROLE_WAITING_AFTER_TIMEOUT:
    Controller_resetAllSensors();
    LCD_WRITES_STATUS("Waiting...");
    if(Status != 0x02)//0x02:Connected
    {
      LCD_WRITES_STATUS("Power off.");
      Controller_PowerEnable(false);
    }
    else
      Status = 0x01;
    break;
    
  case GAPROLE_ERROR:
    Controller_resetAllSensors();
    PIN_setOutputValue(hGpioPin,Board_LED1, Board_LED_ON);
    LCD_WRITES_STATUS("Error");
    break;

  default:
    break;
  }

  gapProfileState = newState;
}

/*******************************************************************************
 * @fn      Controller_charValueChangeCB
 *
 * @brief   Callback from Sensor Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
void Controller_charValueChangeCB(uint8_t serviceID, uint8_t paramID)
{
  Controller_enqueueMsg(ST_CHAR_CHANGE_EVT, serviceID, paramID);
}

/*******************************************************************************
 * @fn      Controller_blinkLed
 *
 * @brief   Blinks a led 'n' times, duty-cycle 50-50
 * @param   led - led identifier
 * @param   nBlinks - number of blinks
 *
 * @return  none
 */
void Controller_blinkLed(uint8_t led, uint8_t nBlinks)
{
  uint8_t i;

  for (i=0; i<nBlinks; i++)
  {
    PIN_setOutputValue(hGpioPin, led, Board_LED_ON);
    delay_ms(BLINK_DURATION);
    PIN_setOutputValue(hGpioPin, led, Board_LED_OFF);
    delay_ms(BLINK_DURATION);
  }
}

/*******************************************************************************
 * @fn      Controller_processCharValueChangeEvt
 *
 * @brief   Process pending Profile characteristic value change
 *          events. The events are generated by the network task (BLE)
 *
 * @param   serviceID - ID of the affected service
 * @param   paramID - ID of the affected parameter
 *
 * @return  none
 */
static void Controller_processCharValueChangeEvt(uint8_t serviceID,
                                                uint8_t paramID)
{
  switch (serviceID)
  {
#ifdef FEATURE_LCD
  case SERVICE_ID_DISPLAY:
    ControllerDisplay_processCharChangeEvt(paramID);
    break;
#endif
  case SERVICE_ID_INERTIA:
    AHRS_processCharChangeEvt(paramID);
    break;
  case SERVICE_ID_FIG:
    FIG_processCharChangeEvt(paramID);
    break;
  case SERVICE_ID_CAL:
    CalibWriteScheduled |= paramID;
    break;
  default:
    break;
  }
}

/*******************************************************************************
 * @fn      Controller_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Controller_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      Controller_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;
      
    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            Controller_processCmdCompleteEvt((hciEvt_CmdComplete_t *)pMsg);
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
}

/*******************************************************************************
 * @fn      Controller_processGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void Controller_processGATTMsg(gattMsgEvent_t *pMsg)
{
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      Controller_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Controller_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
      {
        CRSSi = (int8)pMsg->pReturnParam[3];
        
        //System_printf("RSSI: %d dB\n\r", CRSSi);
      }
      break;
      
    default:
      break;
  }
}

/*******************************************************************************
 * @fn      Controller_performPeriodicTask
 *
 * @brief   Perform a periodic application task.
 *
 * @param   none
 *
 * @return  none
 */
static void Controller_performPeriodicTask(void)
{
  uint16_t ConnectionHandle = INVALID_CONNHANDLE;
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &ConnectionHandle);
  if(ConnectionHandle != INVALID_CONNHANDLE)
  {
    VOID HCI_ReadRssiCmd(ConnectionHandle);
  }
  //System_printf("RSSI:%d\n\r",CRSSi);
  
#ifdef FEATURE_REGISTER_SERVICE
  // Force notification on Register Data (if enabled)
  //Register_setParameter(SENSOR_DATA,0,NULL);
#endif
}

#if FEATURE_OAD
/*******************************************************************************
 * @fn      Controller_processOadWriteCB
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
static void Controller_processOadWriteCB(uint8_t event, uint16_t connHandle,
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

/*******************************************************************************
 * @fn      Controller_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - event type
 *
 * @return  none
 */
static void Controller_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*******************************************************************************
 * @fn      Controller_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   serviceID - service identifier
 * @param   paramID - parameter identifier
 *
 * @return  none
 */
static void Controller_enqueueMsg(uint8_t event, uint8_t serviceID, uint8_t paramID)
{
  stEvt_t *pMsg;

  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(stEvt_t)))
  {
    pMsg->event = event;
    pMsg->serviceID = serviceID;
    pMsg->paramID = paramID;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8_t*)pMsg);
  }
}


/*********************************************************************
 * @fn      Controller_resetAllSensors
 *
 * @brief   Reset all sensors, typically when a connection is intentionally
 *          terminated.
 *
 * @param   none
 *
 * @return  none
 */
static void Controller_resetAllSensors(void)
{
  AHRS_reset();
  FIG_reset();
  ControllerKeys_reset();
}

/*!*****************************************************************************
 *  @fn         Controller_callback
 *
 *  Interrupt service routine for buttons, relay and MPU
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
static void Controller_callback(PIN_Handle handle, PIN_Id pinId)
{
  switch (pinId) {

  case Board_KEY_LEFT:
    ControllerKeys_processKeyLeft();
    break;

  case Board_KEY_RIGHT:
    ControllerKeys_processKeyRight();
    break;

  case Board_RELAY:
    ControllerKeys_processRelay();
    break;

  case PeripheralKey1:
    FIG_UpdKey();
    break;
    
  default:
    /* Do nothing */
    break;
  }
}

/*******************************************************************************
 * @fn      Controller_updateAdvertisingData
 *
 * @brief   Update the advertising data with the latest key press status
 *
 * @return  none
 */
void Controller_updateAdvertisingData(uint8_t keyStatus)
{
  // Record key state in advertising data
  advertData[KEY_STATE_OFFSET] = keyStatus;
  GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
}

/*******************************************************************************
*******************************************************************************/

