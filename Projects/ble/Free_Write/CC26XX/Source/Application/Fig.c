/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp.h"

#include "Fig.h"
#include "figservice.h"
#include "./../Board/sensor.h"
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
//#define         Dprintf(...) System_printf(__VA_ARGS__)
#define         Dprintf(...) 

/*********************************************************************
 * CONSTANTS
 */
// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD   1000
#define SENSOR_TIME_RES         10
#define SENSOR_DATA_LEN         FIG_DATA_LEN

#define 	M_PI     3.14159265358979323846 /* pi */
#define 	M_PI_2   1.57079632679489661923 /* pi/2 */
#define         Ra2De    57.2957795130823208767 /* 180/PI */
#define         De2Ra    0.01745329251994329576 /* PI/180 */
#define         Filter   5                      /* Filter Step */
#define         Filter2  3                      /* Small Filter Step */
#define         RollLim  70.0
#define         ResendVal 30        
//#define         MagZResetV  0.98

//MPUSettingStat
#define MPUDone     0x00
#define MPUReset    0x01
#define MPUStart    0x02

/*********************************************************************
 * TYPEDEFS
 */
union Byte16_8
{
  uint8_t bdata[2];
  uint16_t ldata;
};
/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern bool FunctionAlterMag;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern bStatus_t AHRS_getParameter(uint8_t param, void *value);
extern void quatrotate(bool invert, float ax, float ay, float az, float *x, float *y, float *z);
/*********************************************************************
 * LOCAL VARIABLES
 */
static Clock_Struct periodicClock;

static uint8_t sensorConfig;
static uint16_t sensorPeriod;
static const uint16_t MaxUpdatePeriod = SENSOR_DEFAULT_PERIOD;
static uint16_t mpuTestResult;
static volatile bool sensorReadScheduled;
static volatile bool sensorWriteScheduled;
static volatile bool ResetLock = false;

static volatile bool ReadLock = false;
static volatile bool DumpMag = false;
static uint8_t MPUSettingStat = 0x00;
static float DRoll = 0.0;
static float DAccRef[3] = {0.0};
static float DMagRef[3] = {0.0};
int8_t MagOffset[FigCount * 3] = { 0 };
int8_t MagOffset2[FigCount * 3] = { 0 };
static const uint8_t Sens[5] = {MPU_FIG1,MPU_FIG2,MPU_FIG3,MPU_FIG4,MPU_FIG5};
static const uint8_t SensShift = 8;
static const uint8_t SensMask[5] = {0x01,0x02,0x04,0x08,0x10};
static float AngFil[FigCount][Filter + 1] = {0.0};
static uint8_t AngInd[FigCount] = {0};

static float AngFil2[FigCount][Filter2 + 1] = {0.0};
static uint8_t AngInd2[FigCount] = {0};
static int8_t MagRead[FigCount*2][3];//ID, X, Y, Z

static int16_t SAng[FigCount+1] = {0};
#define KeyStatus       SAng[FigCount]
static uint16_t timecounter = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorConfigChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);
static bool PeriCheck(void);//Check if AHRS Disable
static void FIG_clockHandler(UArg arg);
static void GetAngleDiff(void);

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
void ControllerFIG_init(void)
{
  // Add service
  FIG_addService();
  
  // Register callbacks with profile
  FIG_registerAppCBs(&sensorCallbacks);

  // Initialize the module state variables
  sensorPeriod = SENSOR_DEFAULT_PERIOD;

  Util_constructClock(&periodicClock, FIG_clockHandler,
                      100, MaxUpdatePeriod, false, 0);
  // Initialize characteristics and sensor driver
  initCharacteristicValue(SENSOR_PERI,
                          SENSOR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION,
                          sizeof ( uint8_t ));
  mpuTestResult = FigTestResult();
  FIG_setParameter(SENSOR_CAL_DATA1, FIG_CAL_LEN1, MagOffset);
  FIG_setParameter(SENSOR_CAL_DATA2, FIG_CAL_LEN2, MagOffset2);
  FIG_reset();
}

void FIG_processCharChangeEvt(uint8_t paramID)
{
  uint8_t newValue;
  switch (paramID)
  {
  case SENSOR_CONF:
    // Make sure sensor is disabled
    if(FigTestResult() == 0)
      sensorConfig = ST_CFG_ERROR;
    if(sensorConfig != ST_CFG_ERROR)
    {
      FIG_getParameter(SENSOR_CONF, &newValue);
      if (newValue == ST_CFG_SENSOR_DISABLE)
      {
        Util_stopClock(&periodicClock);// Deactivate task, Clock
        initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
        initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof(uint8_t));
        MPUSettingStat = MPUReset;
      }
      else
      {
        MPUSettingStat = MPUReset | MPUStart;
        if( (newValue&FigCalCom) != 0x00)
          DumpMag = true;
        else
          DumpMag = false;
        Util_startClock(&periodicClock);
      }
      sensorConfig = newValue;
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof ( uint8_t ));
    }
    break;

  case SENSOR_PERI:
    FIG_getParameter(SENSOR_PERI, &newValue);
    sensorPeriod = newValue * SENSOR_PERIOD_RESOLUTION / SENSOR_TIME_RES;
    Util_rescheduleClock(&periodicClock, sensorPeriod);
    break;
    
  case SENSOR_CAL_DATA1:
    FIG_getParameter(SENSOR_CAL_DATA1, MagOffset);
    System_printf("Got Mag Cal datas Section 1.\n\r");
    Controller_charValueChangeCB(SERVICE_ID_CAL, 0x02);
    break;
    
  case SENSOR_CAL_DATA2:
    FIG_getParameter(SENSOR_CAL_DATA2, MagOffset2);
    System_printf("Got Mag Cal datas Section 2.\n\r");
    Controller_charValueChangeCB(SERVICE_ID_CAL, 0x04);
    break;

  default:
    // Should not get here
    break;
  }
}
void FIG_reset(void)
{
  uint8_t i;
  sensorConfig = ST_CFG_SENSOR_DISABLE;
  initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
  initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof(uint8_t));
  sensorWriteScheduled = false;
  sensorReadScheduled = false;
  if (Util_isActive(&periodicClock))
    Util_stopClock(&periodicClock);// Deactivate task, Clock
  MPUSettingStat = MPUDone;
  if(!sensorBoardMpu9250PowerIsOn())
    return;
  for(i=0;i<FigCount;i++)
  {
    if((SensMask[i] & mpuTestResult) != 0)
    {
      sensorSelMpu9250Reset(Sens[i]);//put into sleep
      if(( (SensMask[i]<<SensShift) & mpuTestResult) != 0)
      {
        sensorSelMpu9250Reset(Sens[i] | MPU9250_Serial_MASK);//put into sleep
      }

    }
  }
  I2CMUX_RESET();
  KeyStatus = 0x00;
}

void FIG_UpdRef(float R, float Ax, float Ay, float Az, float Mx, float My, float Mz)
{
  if(ReadLock)
    return;
  ReadLock = true;
  DRoll = R;
  DAccRef[0] = Ax;
  DAccRef[1] = Ay;
  DAccRef[2] = Az;
  DMagRef[0] = Mx;
  DMagRef[1] = My;
  DMagRef[2] = Mz;
  ReadLock = false;
}

void FIG_UpdKey()
{
  uint32_t _now, delta_t;
  uint16_t Keys = (KeyStatus & FigKeyMask(FigKey1));
  _now = Clock_getTicks();
  Dprintf("Key status : 0x%04x, ", Keys);
  if (PIN_getInputValue(PeripheralKey1) == PeripheralKeyPress)
  {
    static uint32_t last_release;
    delta_t = (_now - last_release);
    if(delta_t > ((uint32_t)10000u))
    {
      Keys |= FigKey1;
      Dprintf("Key Press time difference : %u", delta_t);
      if( delta_t < ((uint32_t)20000u))//Double Click
      {
        Keys |= (FigKey1<<8);
        Dprintf(" - Double Clicked.");
      }
      Dprintf("\n\r");
    }
    last_release = _now;
  }
  
  if(KeyStatus != Keys)
  {
    KeyStatus = Keys;
    Dprintf("Key Changed : 0x%04x \n\r", Keys);
    sensorWriteScheduled = true;
  }
}

void ControllerFig_processSensorEvent(void)
{
  uint8_t i;
  PeriCheck();
  if (sensorWriteScheduled)
  {
    sensorWriteScheduled = false;
    if(FunctionAlterMag == false)
    {
      for(i=0;i<FigCount;i++)
      {
        SAng[i] = ((uint8_t)AngFil2[i][Filter2])<<8 | ((uint8_t)AngFil[i][Filter]);
        //System_printf("\tSAng[%d] = 0x%04x\n\r", i,SAng[i]);
        //SAng[i] = (int16_t)(AngFil[i][Filter]*100);
      }
      FIG_setParameter(SENSOR_DATA,FIG_DATA_LEN,SAng);
    }
    else
    {
      static int16_t MData[FigCount+1] = {0};
      for(i=0;i<FigCount;i++)
      {
        MData[0] = i;
        MData[1] = ((int8_t)MagRead[i][0])<<8 | ((int8_t)MagRead[i + FigCount][0])&0xff;
        MData[2] = ((int8_t)MagRead[i][1])<<8 | ((int8_t)MagRead[i + FigCount][1])&0xff;
        MData[3] = ((int8_t)MagRead[i][2])<<8 | ((int8_t)MagRead[i + FigCount][2])&0xff;
        FIG_setParameter(SENSOR_DATA,FIG_DATA_LEN,MData);
      }
    }
  }
  if(sensorReadScheduled)
  {
    sensorReadScheduled = false;
    GetAngleDiff();
  }
}

/*********************************************************************
* Private functions
*/
static void sensorConfigChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  Controller_charValueChangeCB(SERVICE_ID_FIG, paramID);
}
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen)
{
  uint8_t data[SENSOR_DATA_LEN];

  memset(data,value,paramLen);
  FIG_setParameter( paramID, paramLen, data);
}
static bool PeriCheck()
{
  uint8_t newValue,i;
  if(ResetLock == false)
  {
    ResetLock = true;
    if((MPUSettingStat & MPUReset) != 0)
    {
      Dprintf("PeriCheck_MPUReset\n\r");
      for(i=0;i<FigCount;i++)
      {
        if((SensMask[i] & mpuTestResult) != 0)
        { 
          sensorSelMpu9250Reset(Sens[i]);
          if((MPUSettingStat & MPUStart) != 0)
          {
            sensorSelMpu9250Enable(Sens[i], MPU_AX_ACC | MPU_AX_MAG);//Enable ALl axias but Gyro
            //Dprintf("MPU[%d] ACC,MAG Enable.\n\r",Sens[i]);
          }
          if(( (SensMask[i]<<SensShift) & mpuTestResult) != 0)
          {
            sensorSelMpu9250Enable(Sens[i] | MPU9250_Serial_MASK, MPU_AX_ACC | MPU_AX_MAG);//Enable ALl axias but Gyro
          }
        }
      }
      I2CMUX_RESET();
      MPUSettingStat = MPUDone;
    }
    ResetLock = false;
  }
  
  AHRS_getParameter(SENSOR_CONF, &newValue);
  if(sensorConfig == ST_CFG_SENSOR_DISABLE)
    return false;
  else
  {
    if(!sensorBoardMpu9250PowerIsOn() || newValue == ST_CFG_SENSOR_DISABLE)
    {
      FIG_reset();
      return false;
    }
    else
    {
      if (!Util_isActive(&periodicClock))
        Util_startClock(&periodicClock);// Activate task, Clock
    }
  }
  return true;
}
static void FIG_clockHandler(UArg arg)
{
  // Schedule readout periodically
  if(PeriCheck())
  {
    timecounter++;
    sensorReadScheduled = true;
    if(timecounter>SENSOR_TIME_RES)
    {
      timecounter = 0;
      sensorWriteScheduled = true;
    }
    Semaphore_post(sem);
  }
}
static void GetAngleDiff()
{
  float recipNorm = 1.0;
  float Roll = 0.0;//Y-Z Axis
  float Roll_M = 0.0;
  float AccRef[3] = {0.0};
  float MagRef[3] = {0.0};
  float *CalAcc;
  float acc[3];
  float acc2[3];
  float mag[3];
  float mag2[3];
  float Dot[2] = {0.0};
  uint8_t NeedReSend = 0x00;
  uint8_t i;
  uint8_t tempj = 0;
  uint8_t _Sel;
  uint8_t k = 0;
  uint8_t mpustatus;
  uint8_t mpuIntStatus;
  uint8_t SuccessRead = 0;
  uint16_t data[6];
  if(ReadLock)
    return;
  
  ReadLock = true;
  Roll = DRoll;
  AccRef[0] = DAccRef[0];
  AccRef[1] = DAccRef[1];
  AccRef[2] = DAccRef[2];
  MagRef[0] = DMagRef[0];
  MagRef[1] = DMagRef[1];
  MagRef[2] = DMagRef[2];
  ReadLock = false;
  if(DumpMag == true)
    System_printf("Board Mag(Raw):%10f %10f %10f  \n\r", MagRef[0],MagRef[1],MagRef[2]);
  quatrotate(true, MagRef[0], MagRef[1], MagRef[2], &MagRef[0], &MagRef[1], &MagRef[2]);
  
  //Dprintf("Board Mag(Rot):%10f %10f %10f", i, MagRef[0],MagRef[1],MagRef[2]);
  
  
  recipNorm = invSqrt( AccRef[0]*AccRef[0] + AccRef[1]*AccRef[1] + AccRef[2]*AccRef[2]);
  AccRef[0] *= recipNorm;
  AccRef[1] *= recipNorm;
  AccRef[2] *= recipNorm;
  
  recipNorm = invSqrt( MagRef[0]*MagRef[0] + MagRef[1]*MagRef[1]);
  MagRef[0] *= recipNorm;
  MagRef[1] *= recipNorm;
  
  Roll = fabs(cos(DRoll*De2Ra));
  Roll_M = fabs(sin(DRoll*De2Ra));
  for(i = 0; i<FigCount && (sensorConfig != ST_CFG_SENSOR_DISABLE); i++)
  {
    tempj = 0;
    _Sel = Sens[i];
    if(SensMask[i] & mpuTestResult)
    {
      do
      {
        float AngleM = 0.0;
        float AngleG = 0.0;
        
        if(tempj == 1)
        {
          _Sel = ( Sens[i] | MPU9250_Serial_MASK );
          memcpy(mag2, mag, 12);
          memcpy(acc2, acc, 12);
          CalAcc = acc2;
        }
        else
          CalAcc = AccRef;
        
        mpuIntStatus = sensorSelMpu9250IntStatus(_Sel);
        if (mpuIntStatus & MPU_DATA_READY)
        {
          mpustatus = sensorSelMpu9250MagRead(_Sel, (int16_t*)&data[3]);
          if (mpustatus != MAG_STATUS_OK)
          {
            if (mpustatus != MAG_DATA_NOT_RDY)
              sensorSelMpu9250MagReset(_Sel);
          }
          else
          {
            if(!sensorSelMpu9250AccRead(_Sel, data))
              break;
            
            acc[0] = sensorAllMpu9250AccConvert(data[0]);
            acc[1] = sensorAllMpu9250AccConvert(data[1]);
            acc[2] = sensorAllMpu9250AccConvert(data[2]);
            mag[0] = sensorAllMpu9250MagConvert(data[4]);//Mag ax-y eq Acc ax-x   Use X-Z plane
            mag[1] = sensorAllMpu9250MagConvert(data[3]);//Mag ax-x eq Acc ax-y
            mag[2] = -1.0 * sensorAllMpu9250MagConvert(data[5]);//Mag ax-z eq Acc -ax-z
            if(tempj == 0)
            {
              mag[0] -= MagOffset[i * 3];
              mag[1] -= MagOffset[i * 3 + 1];
              mag[2] -= MagOffset[i * 3 + 2];
            }
            else
            {
              mag[0] -= MagOffset2[i * 3];
              mag[1] -= MagOffset2[i * 3 + 1];
              mag[2] -= MagOffset2[i * 3 + 2];
            }
            if(DumpMag == true)
              System_printf("Mag[%d]-%d:%10f %10f %10f\n\r", i, tempj, mag[0],mag[1],mag[2]);
            
            if(i == 0 && tempj == 1)
            {
              acc[0] = -1 * acc[0];
              acc[1] = -1 * acc[1];
              mag[0] = -1 * mag[0];
              mag[1] = -1 * mag[1];
              CalAcc = AccRef;
            }
            if(FunctionAlterMag)
            {
              uint8_t selmag = i;
              if(tempj)
                selmag += FigCount;
              MagRead[selmag][0] = (int8_t)floor(mag[0]);
              MagRead[selmag][1] = (int8_t)floor(mag[1]);
              MagRead[selmag][2] = (int8_t)floor(mag[2]);
            }
            
            recipNorm = invSqrt( acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
            acc[0] *= recipNorm;
            acc[1] *= recipNorm;
            acc[2] *= recipNorm;
            quatrotate(true, mag[0], mag[1], mag[2], &mag[0], &mag[1], &mag[2]);
//            recipNorm = invSqrt( mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
//            mag[0] *= recipNorm;
//            mag[1] *= recipNorm;
//            mag[2] *= recipNorm;
            recipNorm = invSqrt( mag[0]*mag[0] + mag[1]*mag[1]);
            mag[0] *= recipNorm;
            mag[1] *= recipNorm;
            
            
            SuccessRead++;         
            if(_Sel == MPU_FIG1)
            {
              Dot[0] = acc[0]*CalAcc[0] + acc[2]*CalAcc[1];
              Dot[1] = invSqrt(acc[0]*acc[0] + acc[2]*acc[2]) * invSqrt(CalAcc[0]*CalAcc[0] + CalAcc[1]*CalAcc[1]);
            }
            else
            {
              Dot[0] = acc[0]*CalAcc[0] + acc[2]*CalAcc[2];
              Dot[1] = invSqrt(acc[0]*acc[0] + acc[2]*acc[2]) * invSqrt(CalAcc[0]*CalAcc[0] + CalAcc[2]*CalAcc[2]);
            }
            AngleG = acos(Dot[0]*Dot[1]) * Ra2De;
            if(_Sel == MPU_FIG1)
            {
              Dot[0] = mag[0]*MagRef[0] + mag[1]*MagRef[2];
              Dot[1] = invSqrt(mag[0]*mag[0] + mag[1]*mag[1]) * invSqrt(MagRef[0]*MagRef[0] + MagRef[2]*MagRef[2]);
            }
            else
            {
              Dot[0] = mag[0]*MagRef[0] + mag[1]*MagRef[1];
              Dot[1] = invSqrt(mag[0]*mag[0] + mag[1]*mag[1]) * invSqrt(MagRef[0]*MagRef[0] + MagRef[1]*MagRef[1]);
            }
            AngleM = acos(Dot[0]*Dot[1]);
//            AngleM =  atan2(mag[1],mag[0]);
            
            /*if(i == 1 && tempj == 0)
            {
              System_printf("B: %4.2f %4.2f %4.2f, F: %4.2f %4.2f %4.2f\n\r", MagRef[0], MagRef[1], MagRef[2], mag[0], mag[1], mag[2]);
            }*/
            
            if(tempj == 1 && i != 0)
              AngleM =  atan2(mag[1],mag[0]) - atan2(mag2[1],mag2[0]);
            
            AngleM *= Ra2De;
            
            if(_Sel == MPU_FIG1) 
              AngleG = Roll_M * AngleG + Roll * AngleM;
            else
              AngleG = Roll * AngleG + Roll_M * AngleM;
          }
        }
        while(AngleG>180)
          AngleG-=360;
        while(AngleG<-180)
          AngleG+=360;
        AngleG = fabs(AngleG);
        
        if(tempj != 1)
        {
          AngFil[i][AngInd[i]++] = AngleG;
          AngInd[i] %= Filter;
          AngFil[i][Filter] = 0;
          for(k=0;k<Filter;k++)
            AngFil[i][Filter] += AngFil[i][k] * 1.0 / Filter;
          Dprintf("Deg Diff[%d]: %10f", _Sel, AngleG);
          
          if(abs((int16_t)(AngleG - (SAng[i]&0xff))) > ResendVal )
            NeedReSend++;
          if( ( (SensMask[i]<<SensShift) & mpuTestResult ) == 0)
            break;
          tempj = 1;
        }
        else
        {
          AngFil2[i][AngInd2[i]++] = AngleG;
          AngInd2[i] %= Filter2;
          AngFil2[i][Filter2] = 0;
          for(k=0;k<Filter2;k++)
            AngFil2[i][Filter2] += AngFil2[i][k] * 1.0 / Filter2;
          Dprintf("Deg Diff[%d-2]: %10f", _Sel, AngleG);
          
          if(abs((int16_t)(AngleG - (SAng[i]>>8))) > ResendVal )
            NeedReSend++;
          tempj = 0;
          break;
        }
      } while(1);
    }
  }
  if(SuccessRead>0)
  {
    Dprintf("\n\r");
  }
  if(DumpMag == true)
    DumpMag = false;
  if(NeedReSend)
    sensorWriteScheduled = true;
  I2CMUX_RESET();
}