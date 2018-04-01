/*******************************************************************************
*  Filename:       sensor.c
*  Revised:        $Date: 2015-09-21 15:30:38 +0200 (fr, 21 sep 2015) $
*  Revision:       $Revision: 31581 $
*
*  Description:    Sensor driver shared code.
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
#include "sensor.h"
#include "bsp_i2c.h"

#include "sensor_mpu9250_mux.h"
#include <xdc/runtime/System.h>

/* -----------------------------------------------------------------------------
*                                           Macros and constants
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------
*/
static uint8_t buffer[32];
static uint8_t selfTestResult;
static uint16_t FigSelfTestResult;

/*******************************************************************************
 * @fn          sensorReadReg
 *
 * @brief       This function implements the I2C protocol to read from a sensor.
 *              The sensor must be selected before this routine is called.
 *
 * @param       addr - which register to read
 * @param       pBuf - pointer to buffer to place data
 * @param       nBytes - number of bytes to read
 *
 * @return      TRUE if the required number of bytes are received
 ******************************************************************************/
bool sensorReadReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes)
{
  return bspI2cWriteRead(&addr,1,pBuf,nBytes);
}

/*******************************************************************************
* @fn          sensorWriteReg
* @brief       This function implements the I2C protocol to write to a sensor.
*              The sensor must be selected before this routine is called.
*
* @param       addr - which register to write
* @param       pBuf - pointer to buffer containing data to be written
* @param       nBytes - number of bytes to write
*
* @return      TRUE if successful write
*/
bool sensorWriteReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes)
{
  uint8_t i;
  uint8_t *p = buffer;

  /* Copy address and data to local buffer for burst write */
  *p++ = addr;
  for (i = 0; i < nBytes; i++)
  {
    *p++ = *pBuf++;
  }
  nBytes++;

  /* Send data */
  return bspI2cWrite(buffer,nBytes);
}

/*******************************************************************************
 * @fn      sensorTestExecute
 *
 * @brief   Run a self-test on all the sensors
 *
 * @param   testMap - bit map of sensor to include in the test
 *
 * @return  bit-mask of passed flags, one bit set for each sensor
 */
uint8_t sensorTestExecute(uint8_t testMap)
{
  uint8_t i;
  selfTestResult = 0;
  FigSelfTestResult = 0;
  sensorBoardMpu9250Init();
  I2CMUX_RESET();
  // 4. MPU test
  if (testMap & ST_MPU)
  {
    if (sensorSelMpu9250Test(MPU_Board))
    {
      selfTestResult |= ST_MPU;
    }
    if(MPU_Count>1)
    {
      for(i = 1;i < MPU_Count; i++)
      {
        if (sensorSelMpu9250Test(i))
        {
          FigSelfTestResult |= 1<<(i-1);
          if (sensorSelMpu9250Test(i|MPU9250_Serial_MASK))
            FigSelfTestResult |= 0x100<<(i-1);
        }
      }
      I2CMUX_RESET();
    }
  }
  // 5. Magnetometer test
  if (testMap & ST_MAG)
  {
    if (sensorSelMpu9250MagTest(MPU_Board))
    {
      selfTestResult |= ST_MAG;
    }
    if(MPU_Count>1)
    {
      for(i = 1;i < MPU_Count; i++)
      {
        if ( (FigSelfTestResult & (1<<(i-1))) != 0x00 )
        {
          if(sensorSelMpu9250MagTest(i) == false)
          {
            System_printf("Device[%d] Mag dead.\n\r", i);
            FigSelfTestResult &= (0xffff^(1<<(i-1)));
          }          
          if ( (FigSelfTestResult & (0x100<<(i-1))) != 0x00 )
          {
            if(sensorSelMpu9250MagTest(i|MPU9250_Serial_MASK) == false)
            {
              System_printf("Device[%d]-2 Mag dead.\n\r", i);
              FigSelfTestResult &= (0xffff^(0x100<<(i-1)));
            }
          }
        }
      }
      I2CMUX_RESET();
    }
  }

  return selfTestResult;
}

/*******************************************************************************
 * @fn      sensorTestResult
 *
 * @brief   Return the self-test result
 *
 * @param   none
 *
 * @return  bit-mask of passed flags, one bit set for each sensor
 */
uint8_t sensorTestResult(void)
{
  return selfTestResult;
}

uint16_t FigTestResult(void)
{
  return FigSelfTestResult;
}


/*******************************************************************************
 * @fn      sensorSetErrorData
 *
 * @brief   Fill a result buffer with dummy error data
 *
 * @param   pBuf - pointer to buffer containing data to be written
 * @param   n - number of bytes to fill
 *
 * @return  none
 */
void sensorSetErrorData(uint8_t *pBuf, uint8_t n)
{
  while (n > 0)
  {
    n--;
    pBuf[n] = ST_ERROR_DATA;
  }
}


//
//  Various utilities
//
#include "math.h"


/*******************************************************************************
 * @fn      convertToLe
 *
 * @brief   Convert 16-bit words form big-endian to little-endian
 *
 * @param   none
 *
 * @return  none
 */
void convertToLe(uint8_t *data, uint8_t len)
{
  uint8_t i;

  for (i=0; i<len; i+=2)
  {
    uint8_t tmp;
    tmp = data[i];
    data[i] = data[i+1];
    data[i+1] = tmp;
  }
}


#define PRECISION 100.0
#define IPRECISION 100

/*******************************************************************************
 * @fn      floatToSfloat
 *
 * @brief   Convert a float to a short float
 *
 * @param   data - floating point number to convert
 *
 * @return  converted value
 */
uint16_t floatToSfloat(float data)
{
    double sgn = data > 0 ? +1 : -1;
    double mantissa = fabs(data) * PRECISION;
    int exponent = 0;
    bool scaled = false;

    // Scale if mantissa is too large
    while (!scaled)
    {
      if (mantissa <= (float)0xFFF)
      {
        scaled = true;
      }
      else
      {
        exponent++;
        mantissa /= 2.0;
      }
    }

    uint16_t int_mantissa = (int) round(sgn * mantissa);
    uint16_t sfloat = ((exponent & 0xF) << 12) | (int_mantissa & 0xFFF);

    return sfloat;
}


/*******************************************************************************
 * @fn      floatToSfloat
 *
 * @brief   Convert a short float to a float
 *
 * @param   data - floating point number to convert
 *
 * @return  converted value
 */
float sfloatToFloat(uint16_t rawData)
{
  uint16_t e, m;

  m = rawData & 0x0FFF;
  e = (rawData & 0xF000) >> 12;

  return m * exp2(e) * (1.0/PRECISION);
}

/*******************************************************************************
 * @fn      intToSfloat
 *
 * @brief   Convert an integer to a short float
 *
 * @param   data - integer to convert
 *
 * @return  converted value
 */
uint16_t intToSfloat(int data)
{
    int sgn = data > 0 ? +1 : -1;
    int mantissa = data * IPRECISION;
    int exponent = 0;
    bool scaled = false;

    // Scale if mantissa is too large
    while (!scaled)
    {
      if (mantissa <= 0xFFF)
      {
        scaled = true;
      }
      else
      {
        exponent++;
        mantissa /= 2;
      }
    }

    uint16_t int_mantissa = sgn * mantissa;
    uint16_t sfloat = ((exponent & 0xF) << 12) | (int_mantissa & 0xFFF);

    return sfloat;
}

/*******************************************************************************
 * @fn      invSqrt
 *
 * @brief   Fast inverse square root
 *
 * @param   data - integer to convert
 *
 * @return  converted value
 */
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
/*******************************************************************************
*******************************************************************************/
