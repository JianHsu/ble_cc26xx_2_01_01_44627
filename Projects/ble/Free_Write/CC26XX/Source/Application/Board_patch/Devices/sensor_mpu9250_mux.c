/*******************************************************************************
*  Filename:       sensor_mpu9250.c
*  Revised:        $Date: 2015-02-05 10:47:02 +0100 (on, 05 feb 2015) $
*  Revision:       $Revision: 12066 $
*
*  Description:    Driver for the InvenSense MPU9250 Motion processing Unit
*
*  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* -----------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------
*/
#include "Board.h"
#include "sensor_mpu9250_mux.h"
#include "sensor.h"
#include "bsp_i2c.h"
#include <xdc/runtime/System.h>
/* -----------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------
*/

// Sensor I2C address
#define SENSOR_I2C_ADDRESS            0x68
#define SENSOR_I2C_ADDRESS2           0x69
#define SENSOR_MAG_I2_ADDRESS         0x0C
#define SENSOR_MUX_I2_ADDRESS         0x70

// Registers
#define SELF_TEST_X_GYRO              0x00 // R/W
#define SELF_TEST_Y_GYRO              0x01 // R/W
#define SELF_TEST_Z_GYRO              0x02 // R/W
#define SELF_TEST_X_ACCEL             0x0D // R/W
#define SELF_TEST_Z_ACCEL             0x0E // R/W
#define SELF_TEST_Y_ACCEL             0x0F // R/W

#define XG_OFFSET_H                   0x13 // R/W
#define XG_OFFSET_L                   0x14 // R/W
#define YG_OFFSET_H                   0x15 // R/W
#define YG_OFFSET_L                   0x16 // R/W
#define ZG_OFFSET_H                   0x17 // R/W
#define ZG_OFFSET_L                   0x18 // R/W

#define SMPLRT_DIV                    0x19 // R/W
#define CONFIG                        0x1A // R/W
#define GYRO_CONFIG                   0x1B // R/W
#define ACCEL_CONFIG                  0x1C // R/W
#define ACCEL_CONFIG_2                0x1D // R/W
#define LP_ACCEL_ODR                  0x1E // R/W
#define WOM_THR                       0x1F // R/W
#define FIFO_EN                       0x23 // R/W

// .. registers 0x24 - 0x36 are not applicable to the SensorTag HW configuration

#define INT_PIN_CFG                   0x37 // R/W
#define INT_ENABLE                    0x38 // R/W
#define INT_STATUS                    0x3A // R
#define ACCEL_XOUT_H                  0x3B // R
#define ACCEL_XOUT_L                  0x3C // R
#define ACCEL_YOUT_H                  0x3D // R
#define ACCEL_YOUT_L                  0x3E // R
#define ACCEL_ZOUT_H                  0x3F // R
#define ACCEL_ZOUT_L                  0x40 // R
#define TEMP_OUT_H                    0x41 // R
#define TEMP_OUT_L                    0x42 // R
#define GYRO_XOUT_H                   0x43 // R
#define GYRO_XOUT_L                   0x44 // R
#define GYRO_YOUT_H                   0x45 // R
#define GYRO_YOUT_L                   0x46 // R
#define GYRO_ZOUT_H                   0x47 // R
#define GYRO_ZOUT_L                   0x48 // R

// .. registers 0x49 - 0x60 are not applicable to the SensorTag HW configuration
// .. registers 0x63 - 0x67 are not applicable to the SensorTag HW configuration

#define SIGNAL_PATH_RESET             0x68 // R/W
#define ACCEL_INTEL_CTRL              0x69 // R/W
#define USER_CTRL                     0x6A // R/W
#define PWR_MGMT_1                    0x6B // R/W
#define PWR_MGMT_2                    0x6C // R/W
#define FIFO_COUNT_H                  0x72 // R/W
#define FIFO_COUNT_L                  0x73 // R/W
#define FIFO_R_W                      0x74 // R/W
#define WHO_AM_I                      0x75 // R/W

// Masks is mpuConfig valiable
#define ACC_CONFIG_MASK               0x38
#define GYRO_CONFIG_MASK              0x07

// Values PWR_MGMT_1
#define MPU_SLEEP                     0x4F  // Sleep + stop all clocks
#define MPU_WAKE_UP                   0x09  // Disable temp. + intern osc

// Values PWR_MGMT_2
#define ALL_AXES                      0x3F
#define GYRO_AXES                     0x07
#define ACC_AXES                      0x38

// Data sizes
#define DATA_SIZE                     6

// Output data rates
#define INV_LPA_0_3125HZ              0
#define INV_LPA_0_625HZ               1
#define INV_LPA_1_25HZ                2
#define INV_LPA_2_5HZ                 3
#define INV_LPA_5HZ                   4
#define INV_LPA_10HZ                  5
#define INV_LPA_20HZ                  6
#define INV_LPA_40HZ                  7
#define INV_LPA_80HZ                  8
#define INV_LPA_160HZ                 9
#define INV_LPA_320HZ                 10
#define INV_LPA_640HZ                 11
#define INV_LPA_STOPPED               255

// Bit values
#define BIT_ANY_RD_CLR                0x10
#define BIT_RAW_RDY_EN                0x01
#define BIT_WOM_EN                    0x40
#define BIT_LPA_CYCLE                 0x20
#define BIT_STBY_XA                   0x20
#define BIT_STBY_YA                   0x10
#define BIT_STBY_ZA                   0x08
#define BIT_STBY_XG                   0x04
#define BIT_STBY_YG                   0x02
#define BIT_STBY_ZG                   0x01
#define BIT_STBY_XYZA                 (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG                 (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

// User control register
#define BIT_LATCH_EN                  0x20
#define BIT_ACTL                      0x80

// INT Pin / Bypass Enable Configuration
#define BIT_BYPASS_EN                 0x02
#define BIT_AUX_IF_EN                 0x20

// Magnetometer registers
#define MAG_WHO_AM_I                  0x00  // Should return 0x48
#define MAG_INFO                      0x01
#define MAG_ST1                       0x02  // Data ready status: bit 0
#define MAG_XOUT_L	                  0x03  // Data array
#define MAG_XOUT_H	                  0x04
#define MAG_YOUT_L	                  0x05
#define MAG_YOUT_H	                  0x06
#define MAG_ZOUT_L	                  0x07
#define MAG_ZOUT_H	                  0x08
#define MAG_ST2                       0x09  // Overflow(bit 3), read err(bit 2)
#define MAG_CNTL1                     0x0A  // Mode bits 3:0, resolution bit 4
#define MAG_CNTL2                     0x0B  // System reset, bit 0
#define MAG_ASTC                      0x0C  // Self test control
#define MAG_I2CDIS                    0x0F  // I2C disable
#define MAG_ASAX                      0x10  // x-axis sensitivity adjustment
#define MAG_ASAY                      0x11  // y-axis sensitivity adjustment
#define MAG_ASAZ                      0x12  // z-axis sensitivity adjustment

#define MAG_DEVICE_ID                 0x48

// Mode
#define MAG_MODE_OFF                  0x00
#define MAG_MODE_SINGLE               0x01
#define MAG_MODE_CONT1                0x02
#define MAG_MODE_CONT2                0x06
#define MAG_MODE_FUSE                 0x0F

// Resolution
#define MFS_14BITS                    0     // 0.6 mG per LSB
#define MFS_16BITS                    1     // 0.15 mG per LSB

// Sensor selection/de-selection
#define SENSOR_DESELECT()             bspI2cDeselect()

/* -----------------------------------------------------------------------------
*                           Typedefs
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                           Macros
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                           Local Functions
* ------------------------------------------------------------------------------
*/
static void sensorMuxMpuSleep(uint8_t sel);
static void sensorMuxMpu9250WakeUp(uint8_t sel);
static void sensorMuxMpu9250SelectAxes(uint8_t sel);
static void sensorAllMagInit(void);
static void sensorSelMagInit(uint8_t Sel);
static void sensorSelMagEnable(uint8_t sel, bool);
static bool sensorMuxMpu9250SetBypass(uint8_t sel, uint8_t Enable);
static bool SENSOR_SELECT(uint8_t sel);
static bool SENSOR_SELECT_MAG(uint8_t sel);

/* -----------------------------------------------------------------------------
*                           Local Variables
* ------------------------------------------------------------------------------
*/
static uint8_t mpuConfig[MPU_Count * 2];
static uint8_t magStatus[MPU_Count * 2];
static bool    needDly = true;
static uint8_t accRange;
static uint8_t accRangeReg;
static uint8_t gyrRange;
static uint8_t gyrRangeReg;
static uint8_t val;
static uint8_t muxCh;

// Magnetometer calibration
static int16_t calX[MPU_Count * 2] = {0};
static int16_t calY[MPU_Count * 2] = {0};
static int16_t calZ[MPU_Count * 2] = {0};

// Magnetometer control
static uint8_t scale = MFS_16BITS;      // 16 bit resolution
static uint8_t mode = MAG_MODE_CONT2;  // Operating mode


/*******************************************************************************
* @fn          SENSOR_SELECT
*
* @brief       This function siwtch I2C MUX
*
* @return      if sueccess
*/
static bool SENSOR_SELECT(uint8_t RSel)
{
  uint8_t sel = (RSel&MPU9250_Count_MASK);
  ST_ASSERT((sel&MPU9250_Count_MASK)<MPU_Count);
  if(sel == MPU_Board)
    return bspI2cSelect(BSP_I2C_INTERFACE_1,SENSOR_I2C_ADDRESS);
  else
  {
    if(muxCh != sel)
    {
      if(bspI2cSelect(BSP_I2C_INTERFACE_0,SENSOR_MUX_I2_ADDRESS))
      {
        /*
        uint8_t MuxSel = MUX_INVALID;
        uint8_t trytimes = 0;
        while( MuxSel != (0x01 << sel) )
        {
          bspI2cWriteSingle( 0x01 << sel );
          bspI2cRead(&MuxSel, 1);
          if( (++trytimes) >3 )
          {
            SENSOR_DESELECT();
            muxCh = MUX_INVALID;
            return false;
          }
        }*/
        bspI2cWriteSingle( 0x01 << sel );
        muxCh = sel;
        SENSOR_DESELECT();
      }
      else
      {
        SENSOR_DESELECT();
        return false;
      }
    }
    if( sel != RSel )
      return bspI2cSelect(BSP_I2C_INTERFACE_0,SENSOR_I2C_ADDRESS2);
    else
      return bspI2cSelect(BSP_I2C_INTERFACE_0,SENSOR_I2C_ADDRESS);
  }
}
static bool SENSOR_SELECT_MAG(uint8_t sel)
{
  sel &= MPU9250_Count_MASK;
  ST_ASSERT(sel<MPU_Count);
  if(sel == MPU_Board)
    return bspI2cSelect(BSP_I2C_INTERFACE_1,SENSOR_MAG_I2_ADDRESS);
  else
  {
    if(bspI2cSelect(BSP_I2C_INTERFACE_0,SENSOR_MUX_I2_ADDRESS))
    {
      /*
      uint8_t MuxSel = MUX_INVALID;
      uint8_t trytimes = 0;
      while( MuxSel != (0x01 << sel) )
      {
        bspI2cWriteSingle( 0x01 << sel );
        bspI2cRead(&MuxSel, 1);
        if( (++trytimes) >3 )
        {
          SENSOR_DESELECT();
          muxCh = MUX_INVALID;
          return false;
        }
      }*/
      bspI2cWriteSingle( 0x01 << sel );
      muxCh = sel;
      SENSOR_DESELECT();
    }
    else
    {
      SENSOR_DESELECT();
      return false;
    }
    return bspI2cSelect(BSP_I2C_INTERFACE_0,SENSOR_MAG_I2_ADDRESS);
  }
}

void I2CMUX_RESET()
{
  if(muxCh != MUX_INVALID)
  {
    if(bspI2cSelect(BSP_I2C_INTERFACE_0,SENSOR_MUX_I2_ADDRESS))
    {
      bspI2cWriteSingle( 0x00 );
    }
    SENSOR_DESELECT();
    muxCh = MUX_INVALID;
  }
}

/*******************************************************************************
* @fn          sensorBoardMpu9250PowerOn
*
* @brief       This function turns on the power supply to MPU9250
*
* @return      none
*/
void sensorBoardMpu9250PowerOn(void)
{
  sensorSelMpu9250Reset(MPU_Board);
}

/*******************************************************************************
* @fn          sensorBoardMpu9250PowerOff
*
* @brief       This function turns off the power supply to MPU9250
*
* @return      none
*/
void sensorBoardMpu9250PowerOff(void)
{
  // Force an access on I2C bus #0 (sets the I2C lines to a defined state)
  bspI2cSelect(BSP_I2C_INTERFACE_0,SENSOR_MUX_I2_ADDRESS);
  needDly = true;
}

/*******************************************************************************
* @fn          sensorBoardMpu9250PowerIsOn
*
* @brief       Return 'true' if MPU power is on
*
* @return      state of MPU power
*/
bool sensorBoardMpu9250PowerIsOn(void)
{
 return PIN_getOutputValue(Board_PER_POWER) == Board_PER_POWER_ON;
}

/*******************************************************************************
* @fn          sensorBoardMpu9250Init
*
* @brief       This function initializes the MPU abstraction layer.
*
* @return      True if success
*/
bool sensorBoardMpu9250Init(void)
{
  return sensorSelMpu9250Reset(MPU_Board);
}


/*******************************************************************************
* @fn          sensorAllMpu9250Reset
*
* @brief       This function resets the MPU
*
* @return      True if success
*/
bool sensorAllMpu9250Reset(void)
{
  bool ret;
  uint8_t i;
  accRange = ACC_RANGE_4G;
  gyrRange = GYR_RANGE_250;
  muxCh    = MUX_INVALID;
  for(i=0;i<MPU_Count;i++)
  {
    mpuConfig[i] = 0;   // All axes off
    magStatus[i] = 0;
    
    mpuConfig[i + MPU_Count] = 0;
    magStatus[i + MPU_Count] = 0;
  }
  for(i=0; i<MPU_Count; i++)
  {
    if (!SENSOR_SELECT(i))
    {
      return false;
    }

    // Device reset
    val = 0x80;
    sensorWriteReg(PWR_MGMT_1, &val, 1);
    
//    val = 0x01;
//    sensorWriteReg(CONFIG, &val, 1);//Set low-pass filter to 184 Hz
    val = 0x00;
    sensorWriteReg(CONFIG, &val, 1);//Set Gyroscope low-pass filter to 250 Hz
    val = 0x03;//Set Acc low-pass filter to 41 Hz
    sensorWriteReg(ACCEL_CONFIG_2, &val, 1);
    
    val = 0x00;
    sensorWriteReg(SMPLRT_DIV, &val, 1);// Set sample rate to 1 kHz
    
    SENSOR_DESELECT();
  }
  delay_ms(5);

  ret = sensorAllMpu9250Test();
  if (ret)
  {
    // Initial configuration
    //sensorAllMpu9250AccSetRange(ACC_RANGE_8G);
    sensorAllMpu9250AccSetRange(ACC_RANGE_8G);
    sensorAllMpu9250GyrSetRange(GYR_RANGE_250);
    
    sensorAllMagInit();

    // Power save
    for(i=0; i<MPU_Count; i++)
    {
      sensorMuxMpuSleep(i);
    }
  }
  muxCh = MUX_INVALID;
  return ret;
}
/*******************************************************************************
* @fn          sensorSelMpu9250Reset
*
* @brief       This function resets the Main MPU
*
* @return      True if success
*/
bool sensorSelMpu9250Reset(uint8_t RSel)
{
  bool ret;
  uint8_t Sel = RSel&MPU9250_Count_MASK;
  if(Sel != RSel)
    Sel += MPU_Count;
  if (!SENSOR_SELECT(RSel))
    return false;
  
  muxCh    = MUX_INVALID;
  
  mpuConfig[Sel] = 0;   // All axes off
  magStatus[Sel] = 0;
  // Device reset
  val = 0x80;
  sensorWriteReg(PWR_MGMT_1, &val, 1);
  
  if(Sel == MPU_Board)
  {
    val = 0x00;
    sensorWriteReg(CONFIG, &val, 1);//Set Gyroscope low-pass filter to 250 Hz
//    val = 0x0f;//Set Acc low-pass filter to 1.13 K Hz  
//    val = 0x03;//Set Acc low-pass filter to 41 Hz  
    val = 0x05;//Set Acc low-pass filter to 10 Hz  
    sensorWriteReg(ACCEL_CONFIG_2, &val, 1);
    val = 0x00;// Set sample rate to 1 kHz
//    val = 0x01;// Set sample rate to 0.5 kHz
    sensorWriteReg(SMPLRT_DIV, &val, 1);
  }
  else
  {
    val = 0x04;
    sensorWriteReg(CONFIG, &val, 1);//Set Gyroscope low-pass filter to 20 Hz
//    val = 0x04;//Set Acc low-pass filter to 20 Hz  
    val = 0x06;//Set Acc low-pass filter to 5 Hz  
    sensorWriteReg(ACCEL_CONFIG_2, &val, 1);
    val = 0x04;
    sensorWriteReg(SMPLRT_DIV, &val, 1);// Set sample rate to 62.5 Hz
  }
  
  SENSOR_DESELECT();
  //delay_ms(1);

  ret = sensorSelMpu9250Test(RSel);
  if (ret)
  {
    // Initial configuration
    
    if (!SENSOR_SELECT(RSel))
    {
      SENSOR_DESELECT();
      return false;
    }
  
    accRangeReg = (accRange << 3);
    sensorWriteReg(ACCEL_CONFIG, &accRangeReg, 1);
    
    gyrRangeReg = (gyrRange << 3);
    sensorWriteReg(GYRO_CONFIG, &gyrRangeReg, 1);
    
    SENSOR_DESELECT();
    
    delay_ms(2);
    sensorSelMagInit(RSel);
    sensorSelMpu9250MagReset(RSel);
    // Power save
    sensorMuxMpuSleep(RSel);
  }
  return ret;
}


/*******************************************************************************
* @fn          sensorBoardMpu9250WomEnable
*
* @brief       Enable Wake On Motion functionality
*
* @param       threshold - wake-up trigger threshold (unit: 4 mg, max 1020mg)
*
* @return      True if success
*/
bool sensorBoardMpu9250WomEnable(uint8_t threshold)
{
  ST_ASSERT(sensorBoardMpu9250PowerIsOn());

  if (!SENSOR_SELECT(MPU_Board))
  {
    return false;
  }

  // Make sure accelerometer is running
  val = 0x09;
  ST_ASSERT(sensorWriteReg(PWR_MGMT_1, &val, 1));

  // Enable accelerometer, disable gyro
  val = 0x07;
  ST_ASSERT(sensorWriteReg(PWR_MGMT_2, &val, 1));

  // Set Accel LPF setting to 184 Hz Bandwidth
  val = 0x01;
  ST_ASSERT(sensorWriteReg(ACCEL_CONFIG_2, &val, 1));

  // Enable Motion Interrupt
  val = BIT_WOM_EN;
  ST_ASSERT(sensorWriteReg(INT_ENABLE, &val, 1));

  // Enable Accel Hardware Intelligence
  val = 0xC0;
  ST_ASSERT(sensorWriteReg(ACCEL_INTEL_CTRL, &val, 1));

  // Set Motion Threshold
  val = threshold;
  ST_ASSERT(sensorWriteReg(WOM_THR, &val, 1));

  // Set Frequency of Wake-up
  val = INV_LPA_20HZ;
  ST_ASSERT(sensorWriteReg(LP_ACCEL_ODR, &val, 1));

  // Enable Cycle Mode (Accel Low Power Mode)
  val = 0x29;
  ST_ASSERT(sensorWriteReg(PWR_MGMT_1, &val, 1));

  // Select the current range
  ST_ASSERT(sensorWriteReg(ACCEL_CONFIG, &accRangeReg, 1));

  // Clear interrupt
  sensorReadReg(INT_STATUS,&val,1);

  SENSOR_DESELECT();

  mpuConfig[MPU_Board] = 0;


  return true;
}

/*******************************************************************************
* @fn          sensorBoardMpu9250IntStatus
*
* @brief       Check whether a data or wake on motion interrupt has occurred
*
* @return      Return interrupt status
*/
uint8_t sensorBoardMpu9250IntStatus(void)
{
  return sensorSelMpu9250IntStatus(MPU_Board);
}

/*******************************************************************************
* @fn          sensorSelMpu9250IntStatus
*
* @brief       Check whether a data or wake on motion interrupt has occurred
*
* @return      Return interrupt status
*/
uint8_t sensorSelMpu9250IntStatus(uint8_t Sel)
{
  uint8_t intStatus;
  intStatus = 0;
  ST_ASSERT(sensorBoardMpu9250PowerIsOn());

  if (SENSOR_SELECT(Sel))
  {
    if (!sensorReadReg(INT_STATUS,&intStatus,1))
    {
      intStatus = 0;
    }
    SENSOR_DESELECT();
  }

  return intStatus;
}

/*******************************************************************************
* @fn          sensorSelMpu9250Enable
*
* @brief       Enable accelerometer readout
*
* @param       Axes: Gyro bitmap [0..2], X = 1, Y = 2, Z = 4. 0 = gyro off
* @                  Acc  bitmap [3..5], X = 8, Y = 16, Z = 32. 0 = accelerometer off
*                    MPU  bit [6], all axes
*
* @return      None
*/
void sensorSelMpu9250Enable(uint8_t RSel, uint16_t axes)
{
  uint8_t select = RSel&MPU9250_Count_MASK;
  ST_ASSERT_V(sensorBoardMpu9250PowerIsOn());
  ST_ASSERT_V(select<MPU_Count);
  if( select != RSel )
    select += MPU_Count;
  if (mpuConfig[select] == 0 && axes != 0)
  {
    // Wake up the sensor if it was off
    sensorMuxMpu9250WakeUp(RSel);
  }

  mpuConfig[select] = axes;

  if (mpuConfig[select] != 0)
  {
    // Enable gyro + accelerometer + magnetometer readout
    sensorMuxMpu9250SelectAxes(RSel);
  }
  else if (mpuConfig[select] == 0)
  {
     sensorMuxMpuSleep(RSel);
  }
}


/*******************************************************************************
* @fn          sensorAllMpu9250AccSetRange
*
* @brief       Set the range of the accelerometer
*
* @param       newRange: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*
* @return      true if write succeeded
*/
bool sensorAllMpu9250AccSetRange(uint8_t newRange)
{
  bool success = true;
  uint8_t i;
  if(accRange == newRange)
    return true;
  
  accRangeReg = (newRange << 3);
  ST_ASSERT(sensorBoardMpu9250PowerIsOn());
  if (!SENSOR_SELECT(MPU_Board))
    return false;
    
  // Apply the range
  if(!sensorWriteReg(ACCEL_CONFIG, &accRangeReg, 1))
    success = false;
  SENSOR_DESELECT();
  if (success)
    accRange = newRange;
  else
    return false;
  
  if(MPU_Count>1)
  {
    for(i=1; i<MPU_Count; i++)
    {
      sensorSelMpu9250AccSetRange(i);
    }
  }
  return success;
}

/*******************************************************************************
* @fn          sensorSelMpu9250AccSetRange
*
* @brief       Set the range of the accelerometer by using Global value
*
* @param       Sel Target
*
* @return      true if write succeeded
*/
bool sensorSelMpu9250AccSetRange(uint8_t Sel)
{
  bool Success;
  ST_ASSERT(sensorBoardMpu9250PowerIsOn());
  if (!SENSOR_SELECT(Sel))
    return false;
  
  Success = sensorWriteReg(ACCEL_CONFIG, &accRangeReg, 1);
  
  SENSOR_DESELECT();
  return Success;
}

/*******************************************************************************
* @fn          sensorAllMpu9250AccReadRange
*
* @brief       Apply the selected accelerometer range
*
* @param       none
*
* @return      range: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*/
uint8_t sensorAllMpu9250AccReadRange(void)
{
  ST_ASSERT(sensorBoardMpu9250PowerIsOn());

  if (!SENSOR_SELECT(MPU_Board))
  {
    return false;
  }

  // Apply the range
  sensorReadReg(ACCEL_CONFIG, &accRangeReg, 1);
  SENSOR_DESELECT();

  accRange = (accRangeReg>>3) & 3;

  return accRange;
}


/*******************************************************************************
* @fn          sensorMuxMpu9250AccRead
*
* @brief       Read data from the accelerometer - X, Y, Z - 3 words
*
* @return      True if data is valid
*/
bool sensorSelMpu9250AccRead(uint8_t select, uint16_t *data)
{
  bool success;

  ST_ASSERT(sensorBoardMpu9250PowerIsOn());

  // Burst read of all accelerometer values
  if (!SENSOR_SELECT(select))
    return false;

  success = sensorReadReg(ACCEL_XOUT_H, (uint8_t*)data, DATA_SIZE);
  SENSOR_DESELECT();

  if (success)
  {
    convertToLe((uint8_t*)data,DATA_SIZE);
  }

  return success;
}


/*******************************************************************************
* @fn          sensorAllMpu9250GyrSetRange
*
* @brief       Set the range of the gyroscope
*
* @param       newRange: GYR_RANGE_250, GYR_RANGE_500, GYR_RANGE_1000, GYR_RANGE_2000
*
* @return      true if write succeeded
*/
bool sensorAllMpu9250GyrSetRange(uint8_t newRange)
{
  bool success = true;
  uint8_t i;
  if(gyrRange == newRange)
    return true;
  ST_ASSERT(sensorBoardMpu9250PowerIsOn());
  
  if (!SENSOR_SELECT(MPU_Board))
    return false;
    
  gyrRangeReg = (newRange << 3);
  // Apply the range
  if(!sensorWriteReg(GYRO_CONFIG, &gyrRangeReg, 1))
    success = false;
  SENSOR_DESELECT();
  
  if (success)
    gyrRange = newRange;
  
  if(MPU_Count>1)
  {
    for(i=0; i<MPU_Count; i++)
    {
      sensorSelMpu9250GyrSetRange(i);
    }
  }
  return success;
}

/*******************************************************************************
* @fn          sensorSelMpu9250GyrSetRange
*
* @brief       Set the range of the gyroscope By Using Global Configure(Above)
*
* @param       Target
*
* @return      true if write succeeded
*/
bool sensorSelMpu9250GyrSetRange(uint8_t Sel)
{
  bool Success;
  ST_ASSERT(sensorBoardMpu9250PowerIsOn());
  if (!SENSOR_SELECT(Sel))
    return false;
  
  // Apply the range
  Success = sensorWriteReg(GYRO_CONFIG, &gyrRangeReg, 1);

  SENSOR_DESELECT();
  return Success;
}

/*******************************************************************************
* @fn          sensorAllMpu9250GyrReadRange
*
* @brief       Apply the selected gyroscope range
*
* @param       none
*
* @return      range: GYR_RANGE_250, GYR_RANGE_500, GYR_RANGE_1000, GYR_RANGE_2000
*/
uint8_t sensorAllMpu9250GyrReadRange(void)
{
  ST_ASSERT(sensorBoardMpu9250PowerIsOn());

  if (!SENSOR_SELECT(MPU_Board))
  {
    return false;
  }

  // Apply the range
  sensorReadReg(GYRO_CONFIG, &gyrRangeReg, 1);
  SENSOR_DESELECT();

  gyrRange = (gyrRangeReg>>3) & 3;

  return gyrRange;
}

/*******************************************************************************
* @fn          sensorSelMpu9250GyroRead
*
* @brief       Read data from the gyroscope - X, Y, Z - 3 words
*
* @return      TRUE if valid data, FALSE if not
*/
bool sensorSelMpu9250GyroRead(uint8_t select, uint16_t *data)
{
  bool success;

  ST_ASSERT(sensorBoardMpu9250PowerIsOn());
  // Select this sensor
  if (!SENSOR_SELECT(select))
  {
    return false;
  }

  // Burst read of all gyroscope values
  success = sensorReadReg(GYRO_XOUT_H, (uint8_t*)data, DATA_SIZE);

  SENSOR_DESELECT();

  if (success)
  {
    convertToLe((uint8_t*)data,DATA_SIZE);
  }

  return success;
}


/*******************************************************************************
 * @fn          sensorAllMpu9250Test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool sensorAllMpu9250Test(void)
{
  uint8_t i;
  ST_ASSERT(sensorBoardMpu9250PowerIsOn());
  for(i=0; i<MPU_Count; i++)
  {
    // Select Gyro/Accelerometer
    if (!SENSOR_SELECT(i))
    {
      return false;
    }

    // Make sure power is ramped up
    if (needDly)
    {
      delay_ms(50);
      needDly = false;
    }

    // Check the WHO AM I register
    ST_ASSERT(sensorReadReg(WHO_AM_I, &val, 1));
    ST_ASSERT(val == 0x71);

    SENSOR_DESELECT();
  }
  return true;
}

/*******************************************************************************
 * @fn          sensorSelMpu9250Test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool sensorSelMpu9250Test(uint8_t Sel)
{
  ST_ASSERT(sensorBoardMpu9250PowerIsOn());
  // Select Gyro/Accelerometer
  if (!SENSOR_SELECT(Sel))
    return false;

  // Make sure power is ramped up
  if (needDly)
  {
    needDly = false;
    delay_ms(50);
  }
  // Check the WHO AM I register
  ST_ASSERT(sensorReadReg(WHO_AM_I, &val, 1));
  ST_ASSERT(val == 0x71);
  
  SENSOR_DESELECT();
  return true;
}


/*******************************************************************************
 * @fn          sensorAllMpu9250AccConvert
 *
 * @brief       Convert raw data to G units
 *
 * @param       rawData - raw data from sensor
 *
 * @return      Converted value
 ******************************************************************************/
float sensorAllMpu9250AccConvert(int16_t rawData)
{
  float v;
  switch (accRange)
  {
  case ACC_RANGE_2G:
    //-- calculate acceleration, unit G, range -2, +2
    v = (rawData * 1.0) / (32768/2);
    break;

  case ACC_RANGE_4G:
    //-- calculate acceleration, unit G, range -4, +4
    v = (rawData * 1.0) / (32768/4);
    break;

  case ACC_RANGE_8G:
    //-- calculate acceleration, unit G, range -8, +8
    v = (rawData * 1.0) / (32768/8);
    break;

  case ACC_RANGE_16G:
    //-- calculate acceleration, unit G, range -16, +16
    v = (rawData * 1.0) / (32768/16);
    break;
  default:
    v = 0.0;
    break;
  }

  return v;
}

/*******************************************************************************
 * @fn          sensorMuxMpu9250GyroConvert
 *
 * @brief       Convert raw data to deg/sec units
 *
 * @param       data - raw data from sensor
 *
 * @return      converted data
 ******************************************************************************/
float sensorAllMpu9250GyroConvert(int16_t data)
{
  float v;
  switch(gyrRange)
  {
    case GYR_RANGE_250:
      //-- calculate rotation, unit deg/s, range -250, +250
      v = ((data * 1.0) / 32768 )* 250;
      break;
    case GYR_RANGE_500:
      //-- calculate rotation, unit deg/s, range -500, +500
      v = ((data * 1.0) / 32768 )* 500;
      break;
    case GYR_RANGE_1000:
      //-- calculate rotation, unit deg/s, range -1000, +1000
      v = ((data * 1.0) / 32768 )* 1000;
      break;
    case GYR_RANGE_2000:
      //-- calculate rotation, unit deg/s, range -2000, +2000
      v = ((data * 1.0) / 32768 )* 2000;
      break;
    default:
      break;
  }
  return v;
}

/*******************************************************************************
 * @fn          sensorAllMpu9250MagConvert
 *
 * @brief       Convert raw data to deg/sec units
 *
 * @param       data - raw data from sensor
 *
 * @return      converted data
 ******************************************************************************/

float sensorAllMpu9250MagConvert(int16_t data)
{
  return ((data * 1.0) / 32768 )* 4800;
}

/*******************************************************************************
* @fn          sensorMuxMpuSleep
*
* @brief       Place the MPU in low power mode
*
* @return
*/
static void sensorMuxMpuSleep(uint8_t Sel)
{
  bool success;

  ST_ASSERT_V(sensorBoardMpu9250PowerIsOn());
  ST_ASSERT_V(Sel<MPU_Count);
  if (!SENSOR_SELECT(Sel))
  {
    return;
  }

  val = ALL_AXES;
  success = sensorWriteReg(PWR_MGMT_2, &val, 1);
  if (success)
  {
    val = MPU_SLEEP;
    success = sensorWriteReg(PWR_MGMT_1, &val, 1);
  }

  SENSOR_DESELECT();
}


/*******************************************************************************
* @fn          sensorMuxMpu9250WakeUp
*
* @brief       Exit low power mode
*
* @return      none
*/

static void sensorMuxMpu9250WakeUp(uint8_t RSel)
{
  bool success;
  uint8_t Sel = RSel&MPU9250_Count_MASK;
  if( Sel != RSel )
    Sel += MPU_Count;
  ST_ASSERT_V(sensorBoardMpu9250PowerIsOn());
  if (!SENSOR_SELECT(RSel))
  {
    return;
  }
  val = MPU_WAKE_UP;
  success = sensorWriteReg(PWR_MGMT_1, &val, 1);

  if (success)
  {
    // All axis initially disabled
    val = ALL_AXES;
    success = sensorWriteReg(PWR_MGMT_2, &val, 1);
    mpuConfig[Sel] = 0;
  }

  if (success)
  {
    // Restore the range
    success = sensorWriteReg(ACCEL_CONFIG, &accRangeReg, 1);

    if (success)
    {
      // Clear interrupts
      success = sensorReadReg(INT_STATUS,&val,1);
    }
  }

  SENSOR_DESELECT();
}


/*******************************************************************************
* @fn          sensorMuxMpu9250SelectAxes
*
* @brief       Select gyro, accelerometer, magnetometer
*
* @return      none
*/
static void sensorMuxMpu9250SelectAxes(uint8_t RSel)
{
  uint8_t Sel = RSel&MPU9250_Count_MASK;
  ST_ASSERT_V(Sel<MPU_Count);
  
  if( Sel != RSel )
    Sel += MPU_Count;
  val = ~mpuConfig[Sel];

  if (!SENSOR_SELECT(RSel))
  {
    return;
  }

  sensorWriteReg(PWR_MGMT_2, &val, 1);

  SENSOR_DESELECT();

  sensorSelMagEnable(RSel,!!(mpuConfig[Sel] & MPU_AX_MAG));
}


/*******************************************************************************
* @fn          sensorMuxMpu9250SetBypass
*
* @brief       Allow the I2C bus to control the compass
*
* @return      none
*/
static bool sensorMuxMpu9250SetBypass(uint8_t Sel, uint8_t Enable)
{
  bool success;
  ST_ASSERT((Sel&MPU9250_Count_MASK)<MPU_Count);
  if (SENSOR_SELECT(Sel))
  {
    val = BIT_LATCH_EN;
    if(Enable)
      val |= BIT_BYPASS_EN;
    success = sensorWriteReg(INT_PIN_CFG, &val, 1);
    SENSOR_DESELECT();
  }
  else
    success = false;
  
  return success;
}


/*******************************************************************************
* @fn          sensorMagInit
*
* @brief       Initialize the compass
*
* @return      none
*/
static void sensorAllMagInit(void)
{
  uint8_t i;
  ST_ASSERT_V(sensorBoardMpu9250PowerIsOn());
  for(i=0;i<MPU_Count;i++)
  {
    sensorSelMagInit(i);
  }
}
/*******************************************************************************
* @fn          sensorSelMagInit
*
* @brief       Initialize the compass
*
* @return      none
*/
static void sensorSelMagInit(uint8_t RSel)
{
  uint8_t Sel = RSel&MPU9250_Count_MASK;
  ST_ASSERT_V(sensorBoardMpu9250PowerIsOn());
  
  if (!sensorMuxMpu9250SetBypass(RSel, true))
    return;
  if( Sel != RSel)
    Sel += MPU_Count;
  if (SENSOR_SELECT_MAG(RSel))
  {
    static uint8_t rawData[3];
    
    // Enter Fuse ROM access mode
    val = MAG_MODE_FUSE;
    sensorWriteReg(MAG_CNTL1, &val, 1);
    delay_ms(10);

    // Get calibration data
    if (sensorReadReg(MAG_ASAX, &rawData[0], 3))
    {
      calX[Sel] =  ((int16_t)rawData[0]) + 128;   // Return x-axis sensitivity adjustment values, etc.
      calY[Sel] =  ((int16_t)rawData[1]) + 128;
      calZ[Sel] =  ((int16_t)rawData[2]) + 128;
    }

    // Turn off the sensor by doing a reset
    val = 0x01;
    sensorWriteReg(MAG_CNTL2, &val, 1);
    
    SENSOR_DESELECT();
  }
  sensorMuxMpu9250SetBypass(RSel, false);
}


/*******************************************************************************
 * @fn          sensorAllMpu9250MagReset
 *
 * @brief       Reset the magnetometer
 *
 * @return      none
 */
void sensorAllMpu9250MagReset(void)
{
  uint8_t i;
  ST_ASSERT_V(sensorBoardMpu9250PowerIsOn());
  
  for(i=0; i< MPU_Count; i++)
  {
    if (sensorMuxMpu9250SetBypass(i, true))
    {
      if (SENSOR_SELECT_MAG(i))
      {
        // Turn off the sensor by doing a reset
        val = 0x01;
        sensorWriteReg(MAG_CNTL2, &val, 1);
        // Re-enable if already active
        if (mpuConfig[i] & MPU_AX_MAG)
        {
          val = (scale << 4) | mode;
          sensorWriteReg(MAG_CNTL1, &val, 1); // Set magnetometer data resolution and sample ODR
        }
        SENSOR_DESELECT();
      }
      if(i != MPU_Board)
        sensorMuxMpu9250SetBypass(i, false);
    }
  }
}

/*******************************************************************************
 * @fn          sensorSelMpu9250MagReset
 *
 * @brief       Reset the magnetometer
 *
 * @return      none
 */
void sensorSelMpu9250MagReset(uint8_t RSel)
{
  uint8_t Sel = RSel&MPU9250_Count_MASK;
  ST_ASSERT_V(sensorBoardMpu9250PowerIsOn()); 
  if( Sel != RSel )
    Sel += MPU_Count;
  if (sensorMuxMpu9250SetBypass(RSel, true))
  {
    if (SENSOR_SELECT_MAG(RSel))
    {
      // Turn off the sensor by doing a reset
      val = 0x01;
      sensorWriteReg(MAG_CNTL2, &val, 1);
      delay_ms(10);

      // Re-enable if already active
      if (mpuConfig[Sel] & MPU_AX_MAG)
      {
        val = (scale << 4) | mode;
        sensorWriteReg(MAG_CNTL1, &val, 1); // Set magnetometer data resolution and sample ODR
      }
      SENSOR_DESELECT();
    }
    if(RSel != MPU_Board)
      sensorMuxMpu9250SetBypass(RSel, false);
  }
}


/*******************************************************************************
 * @fn          sensorSelMagEnable
 *
 * @brief       Enable or disable the compass part of the MPU9250
 *
 * @return      none
 */
static void sensorSelMagEnable(uint8_t Sel,bool enable)
{
  uint8_t val;

  ST_ASSERT_V(sensorBoardMpu9250PowerIsOn());
  if (!sensorMuxMpu9250SetBypass(Sel, true))
    return;
  if (SENSOR_SELECT_MAG(Sel))
  {
    if (enable)
    {
      // Set magnetometer data resolution and sample ODR
      val = (scale << 4) | mode;
    }
    else
    {
      // Power down magnetometer
      val = 0x00;
    }
    sensorWriteReg(MAG_CNTL1, &val, 1);

    SENSOR_DESELECT();
  }
  if(Sel != MPU_Board)
    sensorMuxMpu9250SetBypass(Sel, false);
}

/*******************************************************************************
 * @fn          sensorSelMpu9250MagTest
 *
 * @brief       Run a magnetometer self test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool sensorSelMpu9250MagTest(uint8_t Sel)
{
  ST_ASSERT(sensorBoardMpu9250PowerIsOn());
  // Connect magnetometer internally in MPU9250
  if(sensorMuxMpu9250SetBypass(Sel, true))
  {
    // Select magnetometer
    SENSOR_SELECT_MAG(Sel);
    delay_ms(10);
    // Check the WHO AM I register
    val = 0xFF;
    ST_ASSERT(sensorReadReg(MAG_WHO_AM_I, &val, 1));
    ST_ASSERT(val == MAG_DEVICE_ID);
    SENSOR_DESELECT();
    if(Sel != MPU_Board)
      sensorMuxMpu9250SetBypass(Sel, false);
    return true;
  }
  else
    return false;
}

/*******************************************************************************
* @fn          sensorSelMpu9250MagRead
*
* @brief       Read data from the compass - X, Y, Z - 3 words
*
* @return      Magnetometer status
*/
uint8_t sensorSelMpu9250MagRead(uint8_t RSel, int16_t *data)
{
  uint8_t val;
  uint8_t rawData[7];  // x/y/z compass register data, ST2 register stored here,
                       // must read ST2 at end of data acquisition
  uint8_t Sel = RSel&MPU9250_Count_MASK;
  ST_ASSERT(Sel<MPU_Count);
  
  if(Sel != RSel)
    Sel += MPU_Count;
  
  magStatus[Sel] = MAG_NO_POWER;
  ST_ASSERT(sensorBoardMpu9250PowerIsOn());

  magStatus[Sel] = MAG_STATUS_OK;

  // Connect magnetometer internally in MPU9250
  /*SENSOR_SELECT(RSel);
  val = BIT_BYPASS_EN | BIT_LATCH_EN;
  if (!sensorWriteReg(INT_PIN_CFG, &val, 1))
  {
    magStatus[Sel] = MAG_BYPASS_FAIL;
  }
  SENSOR_DESELECT();
  */
  if(RSel != MPU_Board) {
    if(!sensorMuxMpu9250SetBypass(RSel, true))
      magStatus[Sel] = MAG_BYPASS_FAIL;
  }
  if (magStatus[Sel] != MAG_STATUS_OK)
    return MAG_READ_ST_ERR;

  // Select this sensor
  SENSOR_SELECT_MAG(RSel);

  if (sensorReadReg(MAG_ST1,&val,1))
  {
    // Check magnetometer data ready bit
    if (val & 0x01)
    {
      // Burst read of all compass values + ST2 register
      if (sensorReadReg(MAG_XOUT_L, &rawData[0],7))
      {
        val = rawData[6]; // ST2 register

        // Check if magnetic sensor overflow set, if not then report data
        if(!(val & 0x08))
        {
          //data[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
          //data[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
          //data[2] = ((int16_t)rawData[5] << 8) | rawData[4];
          data[0] = ((int16_t*)rawData)[0];
          data[1] = ((int16_t*)rawData)[1];
          data[2] = ((int16_t*)rawData)[2];

          // Sensitivity adjustment
          data[0] = data[0] * calX[Sel] >> 8;
          data[1] = data[1] * calY[Sel] >> 8;
          data[2] = data[2] * calZ[Sel] >> 8;
        }
        else
        {
          magStatus[Sel] = MAG_OVERFLOW;
        }
      }
      else
      {
        magStatus[Sel] = MAG_READ_DATA_ERR;
      }
    }
    else
    {
      magStatus[Sel] = MAG_DATA_NOT_RDY;
    }
  }
  else
  {
    magStatus[Sel] = MAG_READ_ST_ERR;
  }

  // Start new conversion
  val = (scale << 4) | mode;
  sensorWriteReg(MAG_CNTL1, &val, 1); // Set magnetometer data resolution and sample ODR

  SENSOR_DESELECT();
  
  if(Sel != MPU_Board)
    sensorMuxMpu9250SetBypass(RSel, false);
  return magStatus[Sel];
}


/*******************************************************************************
* @fn          sensorMuxMpu9250MagStatus
*
* @brief       Return the magnetometer status
*
* @return      mag status
*/
uint8_t sensorSelMpu9250MagStatus(uint8_t Sel)
{
  return magStatus[Sel&MPU9250_Count_MASK];
}

void sensorSelMpu9250MagOffsetCal(uint8_t Sel, uint8_t step, int16_t *Offset)
{
  int16_t TempData[3];
  if(sensorSelMpu9250MagRead(Sel,TempData) == MAG_STATUS_OK)
  {
    switch(step)
    {
    case 0:
      Offset[Sel] = TempData[0];
      Offset[Sel] = TempData[1];
      Offset[Sel] = TempData[2];
      break;
    case 1:
      Offset[Sel] = (Offset[Sel] + TempData[0])/2;
      Offset[Sel] = (Offset[Sel] + TempData[1])/2;
      Offset[Sel] = (Offset[Sel] + TempData[2])/2;
      break;
    default:
      break;
    }
  }
}