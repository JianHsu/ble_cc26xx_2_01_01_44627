/*******************************************************************************
*  Filename:       sensor_mpu9250.h
*  Revised:        $Date: 2015-03-07 10:33:11 +0100 (fr, 07 mar 2015) $
*  Revision:       $Revision: 12329 $
*
*  Description:    Driver for the InvenSense MPU9250 Motion Processing Unit
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
*
*******************************************************************************/
#ifndef SENSOR_MPU9250_MUX_H
#define SENSOR_MPU9250_MUX_H

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------------
 *                                          Includes
 * -----------------------------------------------------------------------------
 */
#include "stdint.h"
#include "stdbool.h"

/* -----------------------------------------------------------------------------
 *                                          Constants
 * -----------------------------------------------------------------------------
 */
// Accelerometer ranges
#define ACC_RANGE_2G      0
#define ACC_RANGE_4G      1
#define ACC_RANGE_8G      2
#define ACC_RANGE_16G     3
#define ACC_RANGE_INVALID 0xFF
  
// Gyroscope ranges
#define GYR_RANGE_250     0
#define GYR_RANGE_500     1
#define GYR_RANGE_1000    2
#define GYR_RANGE_2000    3
#define GYR_RANGE_INVALID 0xFF

// Axis bitmaps
#define MPU_AX_GYR        0x07
#define MPU_AX_ACC        0x38
#define MPU_AX_MAG        0x40
#define MPU_AX_ALL        0x7F

// Interrupt status bit
#define MPU_DATA_READY    0x01
#define MPU_MOVEMENT      0x40

// Magnetometer status
#define MAG_STATUS_OK     0x00
#define MAG_READ_ST_ERR   0x01
#define MAG_DATA_NOT_RDY  0x02
#define MAG_OVERFLOW      0x03
#define MAG_READ_DATA_ERR 0x04
#define MAG_BYPASS_FAIL   0x05
#define MAG_NO_POWER      0x06
  
//Sensor Position
#define MUX_INVALID       0x00
#define MPU_Board         0x00
#define MPU_FIG1          0x01
#define MPU_FIG2          0x02
#define MPU_FIG3          0x03
#define MPU_FIG4          0x04
#define MPU_FIG5          0x05
#define MPU_Count         6

/* ----------------------------------------------------------------------------
 *                                           Typedefs
 * -----------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
 *                                          Functions
 * -----------------------------------------------------------------------------
 */
bool sensorBoardMpu9250Init(void);
bool sensorAllMpu9250Reset(void);
bool sensorSelMpu9250Reset(uint8_t Sel);
bool sensorAllMpu9250Test(void);
bool sensorSelMpu9250Test(uint8_t Sel);

void sensorBoardMpu9250PowerOn(void);
void sensorBoardMpu9250PowerOff(void);
bool sensorBoardMpu9250PowerIsOn(void);

void sensorSelMpu9250Enable(uint8_t select,uint16_t config);
bool sensorBoardMpu9250WomEnable(uint8_t threshold);//For board sensor only

bool sensorAllMpu9250AccSetRange(uint8_t range);//Update for all
bool sensorSelMpu9250AccSetRange(uint8_t Sel);//Update for Sel by Global Range(Updated by AllSet(Above))
uint8_t sensorAllMpu9250AccReadRange(void);
bool sensorSelMpu9250AccRead(uint8_t select, uint16_t *rawData);
float sensorAllMpu9250AccConvert(int16_t rawValue);

bool sensorAllMpu9250GyrSetRange(uint8_t range);//Update for all
bool sensorSelMpu9250GyrSetRange(uint8_t Sel);//Update for Sel by Global Range(Updated by AllSet(Above))
uint8_t sensorAllMpu9250GyrReadRange(void);
bool sensorSelMpu9250GyroRead(uint8_t select, uint16_t *rawData);
float sensorAllMpu9250GyroConvert(int16_t rawValue);

uint8_t sensorBoardMpu9250IntStatus(void);//Board only
uint8_t sensorSelMpu9250IntStatus(uint8_t Sel);

bool sensorSelMpu9250MagTest(uint8_t Sel);
uint8_t sensorSelMpu9250MagRead(uint8_t select, int16_t *pRawData);
uint8_t sensorSelMpu9250MagStatus(uint8_t select);
void sensorSelMpu9250MagOffsetCal(uint8_t Sel, uint8_t step, int16_t *Offset); // Step 0 : Read normal data, Step 1: Read Z Flip (Roll 180 ,Yaw 180)
float sensorAllMpu9250MagConvert(int16_t rawValue);
void sensorAllMpu9250MagReset(void);
void sensorSelMpu9250MagReset(uint8_t Sel);

void I2CMUX_RESET(void);

/*******************************************************************************
*/

#ifdef __cplusplus
};
#endif

#endif
