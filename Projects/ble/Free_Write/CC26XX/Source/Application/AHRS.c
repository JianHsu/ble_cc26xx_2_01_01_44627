/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp.h"
#include "st_util.h"

#include "AHRS.h"
#include "ahrsservice.h"
#include "recordservice.h"
#include "sensor.h"
#include "Board.h"
#include "sensor_mpu9250_mux.h"

#include "string.h"
#include "math.h"
#include "util.h"
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD   1000


// Length of the data for this sensor
#define SENSOR_DATA_LEN         AHRS_DATA_LEN
#define MOVEMENT_DATA_LEN       18

#define WOM_THR                   10


// Task configuration
#define SENSOR_TASK_PRIORITY    1
#define SENSOR_TASK_STACK_SIZE  600
   
#define         Gravity  9.8                    /*Gravity Acceleration */
//#define         ZGravity 1.015                  /* Normail Gravity(Z:-1G) */
#define         ZGravity 1.14                  /* Normail Gravity(Z:-1G) */
#define 	M_PI     3.14159265358979323846 /* pi */
#define 	M_PI_2   1.57079632679489661923 /* pi/2 */
#define 	M_PI_4   0.78539816339744830962 /* pi/4 */
#define 	M_1_PI   0.31830988618379067154 /* 1/pi */
#define 	M_2_PI   0.63661977236758134308 /* 2/pi */
#define         Ra2De    57.2957795130823208767 /* 180/PI */
#define         De2Ra    0.01745329251994329576 /* PI/180 */
#define         SendRes  100
#define         alpha    0.48f
#define         betaDef	 0.6f		        /* 2 * proportional gain */
#define         calTime  38
#define         StVal    0.08f                   /* Steady val (1 +/- val)*/
//#define         StVal    0.175f                   /* Steady val (1 +/- val)*/
#define         StCnt    20                     /* Steady Cycle*/
#define         Filter   3                      /* Filter Step */
#define         Filter2   3                      /* Small Filter Step */
#define         SleepTime 5                      /* Steady time to enter sleep(s) */
#define         GyrLimit   250.0
#define         GyrDec     0.001
#define         AccVal     0.001             /*Ignore less than 0.001cm/s^2*/
#define         AccPeak    4.0                 /*in m/s^2*/
#define         SpeedDec   0.0004           /* Speed decrement*/
#define         SpeedVal   0.0001           /* Appro Speed to*/
#define         SpeedOffC  0.025
#define         SpeedStopInvC   0.1
#define         SpeedMovingVal  0.3
#define         OffDecPercent 0.99
#define         OffDecPercent2 0.97
#define         SpInvDec   0.3
#define         SpoInvDec  1.0
#define         DicSpZSen  0.2          /*Decreasing Z Senstity*/
#define         DecSen     0.1          /*Decreasing Sensitiy*/
//#define         DecSen     1.0          /*Decreasing Sensitiy*/
#define         DCFliterConst 0.48
#define         DelayCount      3
#define         TempStopVal     8.0f    /*Range of Temp stop*/
#define         TempStopCVal    25.0f

#define         IntgSkipCsn   1
#define         TaskCycle  1000

//Config
#define ResetFlag               0x80
#define CalFlag                 0x40

#define         CMD_TempStop    0x01
#define         CMD_Ready       0x80
#define         CMD_Calibrate   0x40

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
bool FunctionAlterMag = false;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

extern volatile uint16_t Calibrated;
   
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern void FIG_UpdRef(float Roll, float Ax, float Ay, float Az, float Mx, float My, float Mz);
extern void Controller_Capacitive_Touch_Config(uint8_t Enable);

/*********************************************************************
 * Type Define
 */
union ByteFloat
{
  uint8_t bdata[4];
  uint16_t lbdata[2];
  float fdata;
};
union Byte16_8
{
  uint8_t bdata[2];
  uint16_t ldata;
};
/*********************************************************************
 * LOCAL VARIABLES
 */
static Clock_Struct periodicClock;

// Entity ID used to check for source and/or destination of messages
static ICall_EntityID sensorSelfEntity;

// Semaphore used to post events to the application thread
static ICall_Semaphore sensorSem;

// Task setup
static Task_Struct sensorTask;
static Char sensorTaskStack[SENSOR_TASK_STACK_SIZE];

// Parameters
// MPU config:
// bit 0:     WOM enable
// bit 1-2:   accelerometer range (2,4,8,16)
// bit 3-4:   gyroscope range (250,500,1000,2000)
static uint8_t sensorConfig;
static uint16_t sensorPeriod;
static uint16_t MaxUpdatePeriod;
static uint8_t mpuIntStatus;
static volatile bool sensorWriteScheduled;
static volatile bool sensorResetScheduled;

// Application state variables
uint32_t lastUpdate;
uint32_t EStyTime;//Enter Steady time
float deltat;

//static float MagOffset[3] = {15.0f,27.0f,7.0f};//Controller 1
static float MagOffset[3] = {0.0f};
//const static int8_t MagOffset[3] = {28,28,-6};

static float q[4]={1,0,0,0};
static float Dis[3]={0.0f};//Movement
static float DisFil[Filter2]={0};//Movement Filter
static uint8_t FilInd = 0;//Filter data insert Index
static float AccFil[3][Filter + 1] = {0};
static float NAcc[3] = {0.0};
static float MagRAW[3]={0};
static uint8_t AccInd[3] = {0};
static float StoreAcc[3] = {0.0};
static float GryLowPass = 0.0;
static float Speed[3]={0};
static float SpeedOffset[3]={0};
//static uint8_t Status = 0x00;
#define AIMLen (AHRS_DATA_LEN)/2
#define Status (((uint8_t*)AIM)[18])
#define Status2 (((uint8_t*)AIM)[19])
static uint8_t steadycount = 0; // 1 task loop
static int16_t AIM[AIMLen];//Yaw Pitch Roll,dx dy dz (reference from ground and aiming) Sx Sy Sz Ax Ay Az + 1 byte status (0-255)
static uint8_t sensorData[MOVEMENT_DATA_LEN];

static bool TempStopBool = true;

static float TempQ = 0.0f;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void sensorTaskFxn(UArg a0, UArg a1);
void sensorConfigChangeCB(uint8_t paramID);
void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);
static void AHRS_clockHandler(UArg arg);
void AHRS_Convert(int16_t*,float*);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void quatrotate(bool invert, float ax, float ay, float az, float *x, float *y, float *z);
void DelayValue(float x, float y, float z, int _Delay, float *ox, float *oy, float *oz);
float GaussianWindow(float x, float sigma, float h, float v);
float floatabs(float i);

/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
  sensorConfigChangeCB,  // Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AHRS_createTask
 *
 * @brief   Task creation function
 *
 * @param   none
 *
 * @return  none
 */
void AHRS_createTask(void)
{
  Task_Params taskParames;

  // Create the task for the state machine
  Task_Params_init(&taskParames);
  taskParames.stack = sensorTaskStack;
  taskParames.stackSize = SENSOR_TASK_STACK_SIZE;
  taskParames.priority = SENSOR_TASK_PRIORITY;

  Task_construct(&sensorTask, sensorTaskFxn, &taskParames, NULL);
}

/*********************************************************************
 * @fn      AHRS_processCharChangeEvt
 *
 * @brief   AHRS event handling
 *
 */
void AHRS_processCharChangeEvt(uint8_t paramID)
{
  uint8_t newValue;
  switch (paramID)
  {
  case SENSOR_CONF:
    if ((sensorTestResult() & ST_MPU) == 0)
      sensorConfig = ST_CFG_ERROR;
    FunctionAlterMag = false;
    if (sensorConfig != ST_CFG_ERROR)
    {
      AHRS_getParameter(SENSOR_CONF, &newValue);

      if (newValue == ST_CFG_SENSOR_DISABLE)
      {
        System_printf("AHRS Stop.\n\r");
        Util_stopClock(&periodicClock);// Deactivate task, Clock
        sensorSelMpu9250Reset(MPU_Board);
        // Reset characteristics
        //initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
        //Controller_Capacitive_Touch_Config(false);
      }
      else
      {
        if (newValue != ResetFlag)
        {
          // Get interrupt status (clears interrupt)
          //System_printf("AHRS Start(%02x).\n\r", newValue);
          sensorSelMpu9250Reset(MPU_Board);
          sensorSelMpu9250MagReset(MPU_Board);
          sensorSelMpu9250Enable(MPU_Board, MPU_AX_ALL);//Enable ALl axias
          sensorAllMpu9250AccSetRange((newValue&0x06)>>1);
          sensorAllMpu9250GyrSetRange((newValue&0x18)>>3);
          //Setting starting Q, dont need Gyro data now
          Speed[0] = 0.0;
          Speed[1] = 0.0;
          Speed[2] = 0.0;
          SpeedOffset[0] = 0.0;
          SpeedOffset[1] = 0.0;
          SpeedOffset[2] = 0.0;
          Dis[0] = 0.0;
          Dis[1] = 0.0;
          Dis[2] = 0.0;
          Task_setPri(Task_handle(&sensorTask), SENSOR_TASK_PRIORITY);
          Util_startClock(&periodicClock);
          
          if(newValue&CalFlag)
            FunctionAlterMag = true;
          
          //Controller_Capacitive_Touch_Config(true);
        }
        else
        {
          if(sensorConfig)
            sensorResetScheduled = true;
        }
      }
      if (newValue != ResetFlag)
        sensorConfig = newValue;
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof ( uint8_t ));
    }

    // Make sure sensor is disabled
    break;

  case SENSOR_PERI:
    AHRS_getParameter(SENSOR_PERI, &newValue);
    //sensorPeriod = newValue * SENSOR_PERIOD_RESOLUTION;
    sensorPeriod = newValue * 5;
    Util_rescheduleClock(&periodicClock, sensorPeriod);
    
    break;
    
  case SENSOR_CAL_DATA1://calibration data.
    AHRS_getParameter(SENSOR_CAL_DATA1, MagOffset);
    System_printf("Got Board Mag Calibration Data : %f, %f, %f\n\r", MagOffset[0], MagOffset[1], MagOffset[2]);
    Controller_charValueChangeCB(SERVICE_ID_CAL, 0x01);
    break;
    
  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      AHRS_reset
 *
 * @brief   Reset characteristics
 *
 * @param   none
 *
 * @return  none
 */
void AHRS_reset(void)
{
  System_printf("AHRS Reset.\n\r");
  sensorConfig = ST_CFG_SENSOR_DISABLE;
  initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
  initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof(uint8_t));
  sensorWriteScheduled = false;
  // Initialize the driver
  sensorBoardMpu9250PowerOn();
  sensorBoardMpu9250Init();
  Dis[0] = 0.0f;
  Dis[1] = 0.0f;
  Dis[2] = 0.0f;
  Status2 = 0x00;
}

/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      sensorTaskInit
 *
 * @brief   Initialization function for the AHRS Value
 *
 */
static void sensorTaskInit(void)
{
  // Add service
  AHRS_addService();
  Record_addService();

  // Register callbacks with profile
  AHRS_registerAppCBs(&sensorCallbacks);
  Record_registerAppCBs(&sensorCallbacks);

  // Initialize the module state variables
  sensorPeriod = SENSOR_DEFAULT_PERIOD;
  MaxUpdatePeriod = SENSOR_DEFAULT_PERIOD;

  Util_constructClock(&periodicClock, AHRS_clockHandler,
                      1000, MaxUpdatePeriod, false, 0);
  // Initialize characteristics and sensor driver
  AHRS_reset();
  initCharacteristicValue(SENSOR_PERI,
                          SENSOR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION,
                          sizeof ( uint8_t ));
  
}

/*********************************************************************
 * @fn      sensorTaskFxn
 *
 * @brief   The task loop of the AHRS readout task
 *
 * @return  none
 */
void sensorTaskFxn(UArg a0, UArg a1)
{
  float AHR[3] = {0.0};
  uint8_t mpustatus;
  union ByteFloat Conv;
  // Register task with BLE stack
  ICall_registerApp(&sensorSelfEntity, &sensorSem);

  // Initialize the task
  sensorTaskInit();
  //System_printf("AHRS INIT.\n\r");
  
  //System_printf("AHRD didnt find Mag Cal Data. May cause result error.(0x%x)\n\r", mpustatus);
  
  // Deactivate task (active only when measurement is enabled)
  Task_setPri(Task_handle(&sensorTask), -1);
  Status = 0x00;
  Status2 = 0x00;
  Task_sleep(480);
  // Task loop
  while (true)
  {      // Read Interrupt
    if(sensorWriteScheduled)
    {
      AIM[0] = (int16_t)(AHR[0]*SendRes);
      AIM[1] = (int16_t)(AHR[1]*SendRes);
      AIM[2] = (int16_t)(AHR[2]*SendRes);

      
//      Conv.fdata = NAcc[0];
//      AIM[15] = Conv.lbdata[0];
//      AIM[16] = Conv.lbdata[1];
//      Conv.fdata = NAcc[1];
//      AIM[17] = Conv.lbdata[0];
//      AIM[18] = Conv.lbdata[1];
//      Conv.fdata = NAcc[2];
//      AIM[19] = Conv.lbdata[0];
//      AIM[20] = Conv.lbdata[1];
      Conv.fdata = Speed[0];
      AIM[3] = Conv.lbdata[0];
      AIM[4] = Conv.lbdata[1];
      Conv.fdata = Speed[1];
      AIM[5] = Conv.lbdata[0];
      AIM[6] = Conv.lbdata[1];
      Conv.fdata = Speed[2];
      AIM[7] = Conv.lbdata[0];
      AIM[8] = Conv.lbdata[1];
      Record_setParameter(SENSOR_DATA, Record_DATA_LEN, &AIM[3]);
      
      Conv.fdata = NAcc[0];
      //Conv.fdata = AccFil[0][Filter];
      AIM[3] = Conv.lbdata[0];
      AIM[4] = Conv.lbdata[1];
      Conv.fdata = NAcc[1];
      //Conv.fdata = AccFil[1][Filter];
      AIM[5] = Conv.lbdata[0];
      AIM[6] = Conv.lbdata[1];
      Conv.fdata = NAcc[2];
      //Conv.fdata = AccFil[2][Filter];
      AIM[7] = Conv.lbdata[0];
      AIM[8] = Conv.lbdata[1];
      Record_setParameter(SENSOR_DATA2, Record_DATA_LEN, &AIM[3]);
      
      if(FunctionAlterMag == false)
      {
        Conv.fdata = Dis[0];
        AIM[3] = Conv.lbdata[0];
        AIM[4] = Conv.lbdata[1];
        Conv.fdata = Dis[1];
        AIM[5] = Conv.lbdata[0];
        AIM[6] = Conv.lbdata[1];
        Conv.fdata = Dis[2];
        AIM[7] = Conv.lbdata[0];
        AIM[8] = Conv.lbdata[1];
      }
      else
      {
        Conv.fdata = MagRAW[0];
        AIM[3] = Conv.lbdata[0];
        AIM[4] = Conv.lbdata[1];
        Conv.fdata = MagRAW[1];
        AIM[5] = Conv.lbdata[0];
        AIM[6] = Conv.lbdata[1];
        Conv.fdata = MagRAW[2];
        AIM[7] = Conv.lbdata[0];
        AIM[8] = Conv.lbdata[1];
      }
      if ((Calibrated & Calib_Config_Core) != 0)
        Status2 |= CMD_Calibrate;
      
      AHRS_setParameter(SENSOR_DATA, SENSOR_DATA_LEN, AIM);
      Status2 &= ~CMD_TempStop;
      sensorWriteScheduled = false;
//      Dis[0] = 0;
//      Dis[1] = 0;
//      Dis[2] = 0;
    }
    if(sensorResetScheduled)
    {
      sensorResetScheduled = false;
      Dis[0] = 0.0f;
      Dis[1] = 0.0f;
      Dis[2] = 0.0f;
    }
    if(sensorBoardMpu9250PowerIsOn())
    {
      mpuIntStatus = sensorSelMpu9250IntStatus(MPU_Board);

      if (mpuIntStatus & MPU_DATA_READY && (sensorConfig != ST_CFG_SENSOR_DISABLE))
      {
        // Read gyro data
        sensorSelMpu9250GyroRead(MPU_Board, (uint16_t*)sensorData);

        // Read accelerometer data
        sensorSelMpu9250AccRead(MPU_Board, (uint16_t*)&sensorData[6]);
        
        mpustatus = sensorSelMpu9250MagRead(MPU_Board,(int16_t*)&sensorData[12]);
        
        if (mpustatus != MAG_STATUS_OK)
        {
          if (mpustatus != MAG_DATA_NOT_RDY)
          {
            sensorSelMpu9250MagReset(MPU_Board);
          }
          else
          {
            Task_sleep(TaskCycle / 10);
          }
        }
        else
          AHRS_Convert((int16_t*)sensorData,AHR);
      }
    }
    if (sensorConfig != ST_CFG_SENSOR_DISABLE)
    {
      // Next cycle
      if(!Util_isActive(&periodicClock))
        Util_startClock(&periodicClock);
      Task_sleep(TaskCycle);
    }
    else
    {
      if(Util_isActive(&periodicClock))
        Util_stopClock(&periodicClock);
      //delay_ms(SENSOR_DEFAULT_PERIOD);
      Task_setPri(Task_handle(&sensorTask), -1);
    }
  }
}
/*********************************************************************
 * @fn      sensorChangeCB
 *
 * @brief   Callback from AHRS Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void sensorConfigChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  Controller_charValueChangeCB(SERVICE_ID_INERTIA, paramID);
}

/*********************************************************************
 * @fn      initCharacteristicValue
 *
 * @brief   Initialize a characteristic value
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   value - value to initialize with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen)
{
  uint8_t data[SENSOR_DATA_LEN];

  memset(data,value,paramLen);
  AHRS_setParameter( paramID, paramLen, data);
}

/*********************************************************************
 * @fn      SensorTagMov_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - event type
 *
 * @return  none
 */
static void AHRS_clockHandler(UArg arg)
{
  // Schedule readout periodically
  sensorWriteScheduled = true;
}
/*********************************************************************
*********************************************************************/
/***********************AHRS CAL*************************************/

/*********************************************************************
 * @fn      AHRS_Cal
 *
 * @brief   Convert Raw Inertia data to Q and output yaw,pitch roll in intger
 *
 * @param SensData 6 Bytes,outdata 3 integer
 */
void AHRS_Convert(int16_t *data, float *ahr)
{
  float BRG;//Before remove gravity
  float GrySum;
  float datas[9] = {0.0f};
  float accsum;
  float avgaccsum;
  float GryLim;
  float MovementEng = 0.0f;
  uint8_t i;
  uint32_t Now = Clock_getTicks();
  
  datas[0] = sensorAllMpu9250GyroConvert(data[0]);
  datas[1] = sensorAllMpu9250GyroConvert(data[1]);
  datas[2] = sensorAllMpu9250GyroConvert(data[2]);
  datas[3] = sensorAllMpu9250AccConvert(data[3]);
  datas[4] = sensorAllMpu9250AccConvert(data[4]);
  datas[5] = sensorAllMpu9250AccConvert(data[5]);
  datas[6] = sensorAllMpu9250MagConvert(data[7]);//Mag ax-y eq Acc ax-x
  datas[7] = sensorAllMpu9250MagConvert(data[6]);//Mag ax-x eq Acc ax-y
  datas[8] = -1 * sensorAllMpu9250MagConvert(data[8]);//Mag ax-z eq Acc -ax-z
  
  datas[6] -= MagOffset[0];
  datas[7] -= MagOffset[1];
  datas[8] -= MagOffset[2];
  
  MagRAW[0] = datas[6];
  MagRAW[1] = datas[7];
  MagRAW[2] = datas[8];
  
  accsum = 1.0 / invSqrt(datas[3] * datas[3] + datas[4] * datas[4] + datas[5] * datas[5]);
  BRG = accsum;
  GrySum = invSqrt(datas[0]*datas[0] + datas[1]*datas[1] + datas[2]*datas[2]);
  
  GryLim = GyrLimit * GrySum; //GryLim = GyrLimit / (X^2+Y^2+Z^2)^0.5
  GrySum = 1.0 / ( GrySum * 500 );
  GryLim = (GryLim>1.0)?1.0:GryLim;
  if(GryLowPass<GryLim)
    GryLowPass += GyrDec;
  
  if(GryLim < GryLowPass)
    GryLowPass = GryLim;
  
  GryLim = (GryLowPass + GryLim) / 2.0;
  //System_printf("|Gry|:%10f\n\r",1.0 / invSqrt(datas[0]*datas[0] + datas[1]*datas[1] + datas[2]*datas[2]));
  /*DeltaAcc = accsum - LAccSum;
  if( (accsum < 1.0 && LAccSum > 1.0) || (accsum > 1.0 && LAccSum < 1.0) )
    Crossing1 = 1;
  
  LAccSum = accsum;*/
  
  DisFil[FilInd++] = 1.0 / invSqrt(accsum*accsum + datas[0]*datas[0] + datas[1]*datas[1] + datas[2]*datas[2]);
  FilInd %= Filter2;
  MovementEng = 0;
  for(i=0;i<Filter2;i++)
    MovementEng += DisFil[i]/Filter2;
  
  deltat = ((Now - lastUpdate + calTime)/100000.0f);//1 Tick = 10us
  //---------------------------Update First-------------------------------------
  MadgwickAHRSupdate(datas[0]*De2Ra, datas[1]*De2Ra, datas[2]*De2Ra,
                     datas[3],datas[4],datas[5],datas[6],datas[7],datas[8]);
  lastUpdate = Clock_getTicks();
  
//  sensorSelMpu9250AccRead(MPU_Board, (uint16_t*)&data[3]);
//  datas[3] = sensorAllMpu9250AccConvert(data[3]);
//  datas[4] = sensorAllMpu9250AccConvert(data[4]);
//  datas[5] = sensorAllMpu9250AccConvert(data[5]);
  
  //'ZYX' Convert
  ahr[0] = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) + M_PI;
  ahr[1] =  asin(2.0f * (q[0] * q[2] - q[1] * q[3]));
  ahr[2] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  
  if(ahr[0] >M_PI)
    ahr[0]-= M_PI * 2;
  ahr[0] *= Ra2De;
  ahr[1] *= Ra2De;
  ahr[2] *= Ra2De;
  FIG_UpdRef(ahr[2], datas[3], datas[4],datas[5], datas[6], datas[7], datas[8]);
  
  /*if(Crossing1)
    System_printf("Corssing 1G, Delte Acc = %10f\n\r", DeltaAcc);*/
  //System_printf("fabs(BRG - 1) + GrySum : %f \n\r", fabs(BRG - 1) + GrySum);
  if( ( fabs(BRG - 1) + GrySum ) < StVal )
  {
    if(steadycount>StCnt)
    {
      if(Status == Status_Running)
      {
        System_printf("Steady.\n\r");
        EStyTime = Now;
        Status = Status_Steady;
        
        Speed[0] = 0;
        Speed[1] = 0;
        Speed[2] = 0;
      }
    }
    if(steadycount < SleepTime*StCnt)
      steadycount++;
    else
    {
      if(Status != Status_Sleep)
      {
        System_printf("Sleep.\n\r");
        Status = Status_Sleep;
        sensorResetScheduled = true;
      }
    }
  }
  else
  {
    if(Status != Status_Running)
    {
      System_printf("Moving.\n\r");
      Status = Status_Running;
    }
    steadycount = 0;
  }
  if(Status != Status_Sleep)
  {
//    quatrotate(false, 0, 0, ZGravity, &datas[0], &datas[1], &datas[2]);
//    quatrotate(true, datas[3] - datas[0], datas[4] - datas[1], datas[5] - datas[2], &datas[0], &datas[1], &datas[2]);
    //System_printf("Acc_R: %10f %10f %10f\t", datas[3]*100, datas[4]*100, datas[5]*100);
    DelayValue(datas[3], datas[4], datas[5], 1, &datas[3], &datas[4], &datas[5]);
    quatrotate(true, datas[3], datas[4], datas[5], &datas[0], &datas[1], &datas[2]);
    datas[2] -= ZGravity;
    //System_printf("Acc_RNZ: %10f %10f %10f\n\r", datas[0]*100, datas[1]*100, datas[2]*100);
    //datas[2] *= -1.0;
    //ARG = 1.0 / invSqrt(datas[0]*datas[0] + datas[1]*datas[1] + datas[2]*datas[2]);
    //System_printf("BRG-ARG: %10f\n\r", (BRG-ARG));
    if(datas[0]<AccVal&&datas[0]>-AccVal)
      datas[0] = 0.0;
    if(datas[1]<AccVal&&datas[1]>-AccVal)
      datas[1] = 0.0;
    if(datas[2]<AccVal&&datas[2]>-AccVal)
      datas[2] = 0.0;
    //datas[0] = datas[3];
    //datas[1] = datas[4];
    //datas[2] = datas[5];
    datas[3] = (StoreAcc[0] + datas[0]*GryLim)/2;
    datas[4] = (StoreAcc[1] + datas[1]*GryLim)/2;
    datas[5] = (StoreAcc[2] + datas[2]*GryLim)/2;
//    datas[3] = StoreAcc[0];
//    datas[4] = StoreAcc[1];
//    datas[5] = StoreAcc[2];
    StoreAcc[0] = datas[3];
    StoreAcc[1] = datas[4];
    StoreAcc[2] = datas[5];
    datas[0] = datas[3]*Gravity;
    datas[1] = datas[4]*Gravity;
    datas[2] = datas[5]*Gravity;
    NAcc[0] = 0.0;
    NAcc[1] = 0.0;
    NAcc[2] = 0.0;
    
    accsum = 1.0 / invSqrt(Speed[0]*Speed[0] + Speed[1]*Speed[1]+ Speed[2]*Speed[2]);
    //System_printf("Acc: %10f %10f %10f\n\r", datas[0], datas[1], datas[2]);
    
    if(Status == Status_Running)
    {
      datas[3] = datas[0] - AccFil[0][Filter] * DCFliterConst;
      datas[4] = datas[1] - AccFil[1][Filter] * DCFliterConst;
      datas[5] = datas[2] - AccFil[2][Filter] * DCFliterConst;
//      datas[3] = AccFil[0][Filter];
//      datas[4] = AccFil[1][Filter];
//      datas[5] = AccFil[2][Filter];
//      datas[3] = datas[0];
//      datas[4] = datas[1];
//      datas[5] = datas[2];
      avgaccsum = 1.0 / (invSqrt(AccFil[0][Filter] * AccFil[0][Filter] + AccFil[1][Filter] * AccFil[1][Filter] + AccFil[2][Filter] * AccFil[2][Filter]) * Gravity);
      accsum = GaussianWindow(avgaccsum,1.6,0.0 - accsum,0)*5.0;
      //accsum = GaussianWindow(BRG,0.8,2.2,0)*1.95 + GaussianWindow(BRG,0.8,0.5,0)*1.95;
      /*{
        float tempx, tempy, tempz;
        tempx = Speed[0] + datas[3] * deltat;
        tempy = Speed[1] + datas[4] * deltat;
        tempz = Speed[2] + datas[5] * deltat;
        tempx = 1.0 / invSqrt(tempx * tempx + tempy * tempy + tempz * tempz);//next value
        //System_printf("AvgAcc : %4.2f g  CSpeed : %4.2f    delta V : %4.2f\n\r", avgaccsum, accsum, tempx - accsum);
        //accsum is |Speed|
        //accsum = GaussianWindow(avgaccsum,0.8,2.35,0)*2.5 + GaussianWindow(avgaccsum,0.9,0.5,0)*2.5;
        accsum = GaussianWindow(avgaccsum,1.6,0.0 - accsum,0)*3.5;
        if(tempx > accsum) // speed increasing
        {
          accsum = GaussianWindow(avgaccsum,0.8,2.35,0)*2.5 + GaussianWindow(avgaccsum,0.9,0.5,0)*2.5;
          //accsum = GaussianWindow(avgaccsum,0.8,2.3,0)*1.6 + GaussianWindow(avgaccsum,0.9,0.5,0)*2.05;
          //accsum = 1.0;
        }
        else//speed decreasing
          accsum = GaussianWindow(avgaccsum,1.6,0.0 - accsum,0)*3.5;
      }
      */
//      NAcc[0] = datas[3];
//      NAcc[1] = datas[4];
//      NAcc[2] = datas[5];
//      datas[3] *= accsum * GaussianWindow(datas[3],1.5,0.0,0.3)*3.75;
//      datas[4] *= accsum * GaussianWindow(datas[4],1.5,0.0,0.3)*3.75;
//      datas[5] *= accsum * GaussianWindow(datas[5],1.5,0.0,0.3)*3.75;
      //AccFil[0][Filter]
      datas[3] = accsum * (AccFil[0][Filter] + datas[3]) / 2.0;
      datas[4] = accsum * (AccFil[1][Filter] + datas[4]) / 2.0;
      datas[5] = accsum * (AccFil[2][Filter] + datas[5]) / 2.0;
//      datas[3] *= accsum;
//      datas[4] *= accsum;
//      datas[5] *= accsum;
      NAcc[0] = datas[3];
      NAcc[1] = datas[4];
      NAcc[2] = datas[5];
      
      Speed[0] += datas[3] * deltat;
      Speed[1] += datas[4] * deltat;
      Speed[2] += datas[5] * deltat;
      
      SpeedOffset[0] += SpeedOffC*Speed[0];
      SpeedOffset[1] += SpeedOffC*Speed[1];
      SpeedOffset[2] += SpeedOffC*Speed[2];
      
//      SpeedOffset[0] += SpeedOffC*(Speed[0] - SpeedOffset[0]);
//      SpeedOffset[1] += SpeedOffC*(Speed[1] - SpeedOffset[1]);
//      SpeedOffset[2] += SpeedOffC*(Speed[2] - SpeedOffset[2]);
//      SpeedOffset[0] += SpeedOffC*Speed[0] * GaussianWindow(datas[3],1.5,0.0,0)*3.75;
//      SpeedOffset[1] += SpeedOffC*Speed[1] * GaussianWindow(datas[4],1.5,0.0,0)*3.75;
//      SpeedOffset[2] += SpeedOffC*Speed[2] * GaussianWindow(datas[5],1.5,0.0,0)*3.75;
//      SpeedOffset[0] += SpeedOffC * Speed[0] * ( 1.2 - GaussianWindow(Speed[0],3.0,SpeedOffset[0]*0.5,0.0)*7.5 );
//      SpeedOffset[1] += SpeedOffC * Speed[1] * ( 1.2 - GaussianWindow(Speed[1],3.0,SpeedOffset[1]*0.5,0.0)*7.5 );
//      SpeedOffset[2] += SpeedOffC * Speed[2] * ( 1.2 - GaussianWindow(Speed[2],3.0,SpeedOffset[2]*0.5,0.0)*7.5 );
      
      
      //--------------------------------Decreasing Offset
      if(datas[3] > 0 && Speed[0] < 0 || datas[3] < 0 && Speed[0] > 0)
      {
        Speed[0] *= SpInvDec;
        if(Speed[0]>0 && Speed[0]>SpeedOffset[0] || Speed[0]<0 && Speed[0]<SpeedOffset[0])
          SpeedOffset[0] *= SpoInvDec;
      }
      else
      {
        Speed[0]-= SpeedOffset[0];
      }
      //if(Speed[1]<SpeedOffset[1] && Speed[1] >0 || Speed[1]>SpeedOffset[1] && Speed[1] <0)
      if(datas[4] > 0 && Speed[1] < 0 || datas[4] < 0 && Speed[1] > 0)
      {
        Speed[1] *= SpInvDec;
        if(Speed[1]>0 && Speed[1]>SpeedOffset[1] || Speed[1]<0 && Speed[1]<SpeedOffset[1])
          SpeedOffset[1] *= SpoInvDec;
      }
      else
      {
          Speed[1]-= SpeedOffset[1];
      }
      //if(Speed[2]<SpeedOffset[2] && Speed[2] >0 || Speed[2]>SpeedOffset[2] && Speed[2] <0)
      if(datas[5] > 0 && Speed[2] < 0 || datas[5] < 0 && Speed[2] > 0)
      {
        Speed[2] *= SpInvDec;
        if(Speed[2]>0 && Speed[2]>SpeedOffset[2] || Speed[2]<0 && Speed[2]<SpeedOffset[2])
          SpeedOffset[2] *= SpoInvDec;
      }
      else
      {
        Speed[2]-= SpeedOffset[2];
      }
      
//      Speed[0]-= SpeedOffset[0];
//      Speed[1]-= SpeedOffset[1];
//      Speed[2]-= SpeedOffset[2];
//      Speed[0] += datas[3] * deltat;
//      Speed[1] += datas[4] * deltat;
//      Speed[2] += datas[5] * deltat;
      
//      if(Speed[0]>DiscSpeed || Speed[0]<-DiscSpeed)
//        Dis[0] += Speed[0] * deltat;
//      if(Speed[1]>DiscSpeed || Speed[1]<-DiscSpeed)
//        Dis[1] += Speed[1] * deltat;
//      if(Speed[2]>DiscSpeed || Speed[2]<-DiscSpeed)
//        Dis[2] += Speed[2] * deltat;
      if(Status == Status_Running)
      {
        Dis[0] += Speed[0] * deltat;
        Dis[1] += Speed[1] * deltat;
        Dis[2] += Speed[2] * deltat;
//        Dis[0] += (Speed[0]-SpeedOffset[0]) * deltat;
//        Dis[1] += (Speed[1]-SpeedOffset[1]) * deltat;
//        Dis[2] += (Speed[2]-SpeedOffset[2]) * deltat;
      }
      
    }
    
    //AccFil[0][AccInd[0]++] = (datas[0]>AccDNo || datas[0]<-AccDNo)?datas[0]:0.0f;
    AccFil[0][AccInd[0]++] = datas[0];
    AccInd[0] %= Filter;
    AccFil[0][Filter] = 0;
    for(i=0;i<Filter;i++)
      AccFil[0][Filter] += AccFil[0][i] * 1.0 /Filter;
    
    //AccFil[1][AccInd[1]++] = (datas[1]>AccDNo || datas[1]<-AccDNo)?datas[1]:0.0f;
    AccFil[1][AccInd[1]++] = datas[1];
    AccInd[1] %= Filter;
    AccFil[1][Filter] = 0;
    for(i=0;i<Filter;i++)
      AccFil[1][Filter] += AccFil[1][i] * 1.0 /Filter;
    
    //AccFil[2][AccInd[2]++] = (datas[2]>AccDNo || datas[2]<-AccDNo)?datas[2]:0.0f;
    AccFil[2][AccInd[2]++] = datas[2];
    AccInd[2] %= Filter;
    AccFil[2][Filter] = 0;
    for(i=0;i<Filter;i++)
      AccFil[2][Filter] += AccFil[2][i] * 1.0 /Filter;
    
    //System_printf("ACC:,X:%10f Y:%10f Z:%10f\t",AccFil[0][Filter],AccFil[1][Filter],AccFil[2][Filter]);
    //System_printf("ACC(m/s^2):,X:%10f Y:%10f Z:%10f  ",datas[0],datas[1],datas[2]);
    //System_printf("Speed:,X:%10f Y:%10f Z:%10f\t",Speed[0],Speed[1],Speed[2]);
    //System_printf("BRG:%10f ARG:%10f BRG-ARG:%10f\n\r", BRG, ARG, BRG-ARG);
    //System_printf("Displacement:,X:%10f Y:%10f Z:%10f\n\r",Dis[0],Dis[1],Dis[2]);
    if(MovementEng < TempStopVal)
    {
      if(!TempStopBool)
      {
        TempStopBool = true;
        Status2 |= CMD_TempStop;
        sensorWriteScheduled = true;
      }
    }
    else
    {
      if(MovementEng > TempStopCVal)
        TempStopBool = false;
    }
    if( (Status2&CMD_Ready) == 0)
    {
      if(TempQ < 1.0f)
      {
        Status2 |= CMD_Ready;
        System_printf("Ready.\n\r");
        sensorWriteScheduled = true;
      }
    }
  }
  else
  {
    DelayValue(datas[3], datas[4], datas[5], 0, NULL, NULL, NULL);
  }
  //reuse accsum for |speed| now
  accsum = 1.0 / invSqrt(Speed[0]*Speed[0] + Speed[1]*Speed[1] + Speed[2]*Speed[2]);
  if(accsum > SpeedVal)
  {
    accsum = accsum / 8;//nolinear discreasement
    Speed[0] -= accsum * SpeedDec * Speed[0];
    Speed[1] -= accsum * SpeedDec * Speed[1];
    Speed[2] -= accsum * SpeedDec * Speed[2];
    SpeedOffset[0] *= OffDecPercent;
    SpeedOffset[1] *= OffDecPercent;
    SpeedOffset[2] *= OffDecPercent;
  }
  else if(accsum < 1)
  {
    SpeedOffset[0] *= OffDecPercent2;
    SpeedOffset[1] *= OffDecPercent2;
    SpeedOffset[2] *= OffDecPercent2;
  }
  //System_printf("GrySum:%f\n\r", GrySum);
//  if(GrySum < 0.01)
//  {
//    SpeedOffset[0] *= 0.4;
//    SpeedOffset[1] *= 0.4;
//    SpeedOffset[2] *= 0.4;
//  }
  /*
  ahr[0]:Yaw
  ahr[1]:Pitch
  ahr[2]:Roll
  */
  //System_printf("GryLim:%10f\n\r",GryLim);
  //System_printf("Q[0]:%10f Q[1]:%10f Q[2]:%10f Q[3]:%10f  \n\r",q[0],q[1],q[2],q[3]);
  //System_printf("Gryo:%f %f %f\tACC(N):%f %f %f\tMAG: %f %f %f\n\r"
                        //,datas[0],datas[1],datas[2]
                        //,datas[3],datas[4],datas[5]
                        //,datas[6],datas[7],datas[8]);
  //System_printf("Y:%10f P:%10f roll:%10f ",ahr[0],ahr[1],ahr[2]);
  //System_printf("Acc:%10f %10f %10f\n\r",datas[3],datas[4],datas[5]);
  //System_printf("Gry:%10f %10f %10f  ",datas[0],datas[1],datas[2]);
  //System_printf("Mag:%10f %10f %10f\n\r",datas[6],datas[7],datas[8]);
  //System_printf("Gravity:%10f %10f %10f\n\r",GravityDec[0], GravityDec[1], GravityDec[2]);
}
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float q0 = q[0];
  float q1 = q[1];
  float q2 = q[2];
  float q3 = q[3];
  // Rate of change of quaternion from gyroscope
  qDot1 = alpha * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = alpha * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = alpha * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = alpha * (q0 * gz + q1 * gy - q2 * gx);
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= betaDef * s0;
    qDot2 -= betaDef * s1;
    qDot3 -= betaDef * s2;
    qDot4 -= betaDef * s3;
    TempQ = 1.0f - fabs(qDot1) + fabs(qDot2) + fabs(qDot3) + fabs(qDot4);
  }
  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * deltat;
  q1 += qDot2 * deltat;
  q2 += qDot3 * deltat;
  q3 += qDot4 * deltat;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q[0] = q0*recipNorm;
  q[1] = q1*recipNorm;
  q[2] = q2*recipNorm;
  q[3] = q3*recipNorm;
}
void DelayValue(float x, float y, float z, int _Delay, float *ox, float *oy, float *oz)
{
  static int i;
  static float StorageValue[DelayCount][3];
  if(_Delay >= DelayCount)
    _Delay = DelayCount - 1;
  else if(_Delay < 0)
    _Delay = 1;
  _Delay = i - _Delay;
  if(_Delay < 0)
    _Delay += DelayCount;
  if(ox != NULL && oy != NULL && oz != NULL)
  {
    *ox = StorageValue[_Delay][0];
    *oy = StorageValue[_Delay][1];
    *oz = StorageValue[_Delay][2];
  }
  StorageValue[i][0] = x;
  StorageValue[i][1] = y;
  StorageValue[i][2] = z;
  i++;
  i%=DelayCount;
}
void quatrotate(bool invert, float ax, float ay, float az, float *x, float *y, float *z)//convert given value from current q value
{
  float q0q1 = q[0] * q[1];
  float q0q2 = q[0] * q[2];
  float q0q3 = q[0] * q[3];
  float q1q1 = q[1] * q[1];
  float q1q2 = q[1] * q[2];
  float q1q3 = q[1] * q[3];
  float q2q2 = q[2] * q[2];
  float q2q3 = q[2] * q[3];
  float q3q3 = q[3] * q[3];
  if(invert)
  {
    q0q1 *= -1.0;
    q0q2 *= -1.0;
    q0q3 *= -1.0;
  }
  *x = ax*(1-2*q2q2-2*q3q3) + ay*2*(q1q2+q0q3) + az*2*(q1q3-q0q2);
  *y = ax*2*(q1q2-q0q3) + ay*(1-2*q1q1-2*q3q3) + az*2*(q2q3+q0q1);
  *z = ax*2*(q1q3+q0q2) + ay*2*(q2q3-q0q1) + az*(1-2*q1q1-2*q2q2);
}
float GaussianWindow(float x, float sigma, float h, float v)
{
  return (0.3989/sigma)*exp(-1*(x-h)*(x-h)/(2*sigma*sigma)) + v;
}
float floatabs(float i)
{
  return (i >= 0)? i : -1.0 * i;
}